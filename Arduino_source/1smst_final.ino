#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <VL53L1X.h>         // ToF 독립 버스용 라이브러리
#include <Adafruit_PN532.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- [1. 하드웨어 설정: 도트 매트릭스 & 네오픽셀] ---
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN     5
#define DATA_PIN  23
#define CLK_PIN   18
#define NEO_PIN   15   
#define NUM_LEDS  16   

// --- [2. 하드웨어 설정: OLED & NFC (I2C 공유)] ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SDA_PIN 21
#define SCL_PIN 22

// --- [3. 하드웨어 설정: 모터 및 진동모터 핀] ---
const int ENA = 14; 
const int IN1 = 27; 
const int IN2 = 26; 
#define PIN_VIBRATION 25   // 하차 알림 및 경고용 진동 모터 핀

// --- [4. 하드웨어 설정: FSR 압력 센서 및 상태 변수] ---
#define PIN_FSR       34   
const int FSR_THRESHOLD = 4000; // 압력 임계값 4000 반영 완료
bool isOccupied = false;    

// --- [비블로킹 제어용 타이머 및 상태 변수들] ---
enum MotorState { MOTOR_IDLE, MOTOR_OPENING, MOTOR_CLOSING };
MotorState motorState = MOTOR_IDLE; 
unsigned long motorStartTime = 0;   
String pendingSeatInfo = "";        

unsigned long seatOpenTime = 0;           
bool waitingForInitialOccupancy = false;  

bool waitingForVacantFold = false;        
unsigned long vacantStartTime = 0;        

unsigned long oledResetTime = 0;          
bool waitingForOledReset = false;         

bool isVibrating = false;          
unsigned long vibrationStartTime = 0; 
bool vibTriggeredForThisLeg = false; 

// --- [5. 객체 선언] ---
MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
Adafruit_NeoPixel strip(NUM_LEDS, NEO_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);
VL53L1X tof; 

// --- [6. 기차 노선 및 상태 변수] ---
const int stationSeoulIdx = 0;   
const int stationDaeguIdx = 7;   
const int stationBusanIdx = 15;  
enum TrainState { SEOUL_STOP, TO_DAEGU, DAEGU_STOP, TO_BUSAN, BUSAN_STOP };
TrainState currentState = SEOUL_STOP;

unsigned long stateTimer = 0;
unsigned long pixelTimer = 0;
const long stopDuration = 10000;   
const long travelDuration = 7000;  // 주행 시간 7초 고정
float trainPos = 0.0;

// --- [7. 좌석 및 NFC 변수] ---
bool isSeatOpen = false;
const int MOVE_TIME = 1800; 
uint8_t standingCard[] = {0x04, 0x1A, 0x85, 0xB2, 0xAF, 0x1D, 0x90};
uint8_t standardCard[] = {0x04, 0x16, 0x85, 0xB2, 0xAF, 0x1D, 0x90};
uint8_t groupCard[]    = {0x04, 0x15, 0x86, 0xB2, 0xAF, 0x1D, 0x90};

// 기능 함수 선언
void updateMetroMap(TrainState state, int destinationIdx);
void setMirrorPixelColor(int logicalIdx, uint32_t color);

// --- [기능 함수: OLED 표시] ---
void showOLED(String line1, String line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(line1);
  display.setCursor(0, 35);
  display.setTextSize(2);
  display.println(line2);
  display.display();
}

// --- [기능 함수: 모터 정지] ---
void stopMotor() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
}

// --- [기능 함수: 비블로킹 모터 시동] ---
void startOperateSeat(String seatInfo) {
  if (motorState != MOTOR_IDLE) return; 
  
  pendingSeatInfo = seatInfo;
  motorStartTime = millis(); 

  if (!isSeatOpen) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    analogWrite(ENA, 230);
    motorState = MOTOR_OPENING;
    showOLED(seatInfo, "Opening...");
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
    analogWrite(ENA, 230);
    motorState = MOTOR_CLOSING;
    showOLED(seatInfo, "Closing...");
  }
}

// --- [기능 함수: 비블로킹 진동 작동] ---
void startVibration(unsigned long duration) {
  digitalWrite(PIN_VIBRATION, HIGH);
  isVibrating = true;
  vibrationStartTime = millis();
  Serial.println("[HAPTIC] 미하차 승객 대상 깨움 진동 모터 가동!!");
}

// --- [기능 함수: 네오픽셀 미러링 반전] ---
void setMirrorPixelColor(int logicalIdx, uint32_t color) {
  int physicalIdx = (NUM_LEDS - 1) - logicalIdx; 
  strip.setPixelColor(physicalIdx, color);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ===================================================
  // ⭐ [전력 분산 치트키] 순차 부팅용 딜레이 완전 매립
  // ===================================================
  P.begin();
  P.setIntensity(5);
  P.displayClear();
  delay(300); 

  strip.begin();
  strip.setBrightness(50); // 전력 저하 방지를 위해 밝기 50 제한
  strip.show();
  delay(300); 

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PIN_FSR, INPUT); 
  pinMode(PIN_VIBRATION, OUTPUT); 
  digitalWrite(PIN_VIBRATION, LOW); 
  stopMotor();
  delay(200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED 시작 실패"));
  }
  display.clearDisplay();
  showOLED("Smart Seat", "SCAN TICKET!");
  delay(300);

  nfc.begin();
  nfc.SAMConfig();
  delay(300);

  Wire1.begin(32, 33);
  Wire1.setTimeOut(50); 
  tof.setBus(&Wire1);
  if (!tof.init()) {
    Serial.println("ToF 센서 초기화 실패! (안전을 위해 우회)");
  } else {
    Serial.println("ToF 독립 채널 매립 성공!");
    tof.setTimeout(200);          
    tof.startContinuous(50);      
  }

  stateTimer = millis();
  P.displayText("SEOUL", PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
}

void loop() {
  unsigned long currentMillis = millis();

  // 진동 모터 자동 시간 오프 감시
  if (isVibrating) {
    if (currentMillis - vibrationStartTime >= 2000) { 
      digitalWrite(PIN_VIBRATION, LOW);
      isVibrating = false;
      Serial.println("[HAPTIC] 진동 타임아웃 종료");
    }
  }

  // JGY-370 좌석 폴딩 모터 시간 감시
  if (motorState == MOTOR_OPENING) {
    if (currentMillis - motorStartTime >= MOVE_TIME) { 
      stopMotor();
      isSeatOpen = true;
      motorState = MOTOR_IDLE;
      showOLED(pendingSeatInfo, "Opened");
      seatOpenTime = currentMillis;
      waitingForInitialOccupancy = true;
    }
  } 
  else if (motorState == MOTOR_CLOSING) {
    if (currentMillis - motorStartTime >= MOVE_TIME) { 
      stopMotor();
      isSeatOpen = false;
      motorState = MOTOR_IDLE;
      showOLED(pendingSeatInfo, "Closed");
    }
  }

  // --- [센서 데이터 실시간 계측 및 필터링] ---
  int fsrValue = analogRead(PIN_FSR); 
  
  uint16_t distance = 600;
  if (!tof.timeoutOccurred()) {
    distance = tof.read();
    if (distance == 65535) distance = 600;
  }
  
  // 압력 센서 변화 감지
  if (fsrValue >= FSR_THRESHOLD) {
    if (!isOccupied) {
      isOccupied = true;
      waitingForInitialOccupancy = false; 
      waitingForVacantFold = false;       
      Serial.print("[SENSOR] 승객 착좌 확인. FSR: "); Serial.println(fsrValue);
    }
  } 
  else {
    if (isOccupied) {
      isOccupied = false;
      Serial.print("[SENSOR] 승객 하차(공석 전환) 감지! FSR: "); Serial.println(fsrValue);
      
      // 만약 진동벨이 울리는 도중에 일어났다면 진동을 즉시 멈추고 제어권 반환
      if (isVibrating) {
        digitalWrite(PIN_VIBRATION, LOW);
        isVibrating = false;
        Serial.println("[HAPTIC] 진동 중 승객 기립 포착 ➔ 진동 즉시 정지 후 하차 시퀀스 연동");
      }
    }
  }

  // --- [정밀 하차 준비 페이즈 조건 검사] ---
  bool isArrivalPhase = (currentState == SEOUL_STOP || currentState == DAEGU_STOP || currentState == BUSAN_STOP ||
                        ((currentState == TO_DAEGU || currentState == TO_BUSAN) && (currentMillis - stateTimer >= travelDuration - 3000)));

  // 조건: 의자가 열려있고 + 공석 상태이고 + 초기 착석 대기중이 아니며 + 모터 대기중 + 하차 준비 페이즈(4초~) 진입 완료 시
  if (isSeatOpen && !isOccupied && !waitingForInitialOccupancy && !waitingForVacantFold && motorState == MOTOR_IDLE && isArrivalPhase) {
    waitingForVacantFold = true;
    vacantStartTime = currentMillis; 
    Serial.println("[SYSTEM] 하차 준비 시퀀스 돌입 ➔ 1.5초 안전 대기 시작 (ToF 연동)");
    showOLED("Vacant State", "no item !");
  }

  // 1.5초 대기 후 ToF 센서 분실물 최종 감지 및 자동 폴딩 처리
  if (waitingForVacantFold && !isOccupied && isArrivalPhase) {
    if (currentMillis - vacantStartTime >= 1500) {
      if (distance < 400) { 
        showOLED("WARNING!", "lost item !");
        
        // 유실물 감지 시 툭-툭-툭 끊어 치는 스타카토 경고 진동 실행
        static unsigned long staccatoTimer = 0;
        static bool staccatoState = false;
        if (currentMillis - staccatoTimer >= 200) {
          staccatoTimer = currentMillis;
          staccatoState = !staccatoState;
          digitalWrite(PIN_VIBRATION, staccatoState ? HIGH : LOW);
        }
      } 
      else {
        digitalWrite(PIN_VIBRATION, LOW);
        waitingForVacantFold = false;
        Serial.println("[SYSTEM] 공석 및 ToF 무결성 검증 완료 ➔ 최종 오토 폴딩 기구 구동");
        startOperateSeat("Seat 1A");
      }
    }
  }

  // 의자 개방 후 3초 미착석 타임아웃 제어
  if (isSeatOpen && waitingForInitialOccupancy && motorState == MOTOR_IDLE) {
    if (currentMillis - seatOpenTime >= 3000) { 
      Serial.println("[SYSTEM] 경고: 3초간 미착석 ➔ 자동 리셋 폴딩");
      showOLED("No Passenger", "Auto Reset");
      waitingForInitialOccupancy = false; 
      oledResetTime = currentMillis;
      waitingForOledReset = true;
      startOperateSeat("Seat 1A"); 
    }
  }

  if (waitingForOledReset && motorState == MOTOR_IDLE) {
    if (currentMillis - oledResetTime >= 2000) {
      waitingForOledReset = false;
      showOLED("Smart Seat", "SCAN TICKET!");
    }
  }

  // --- [Part A: 도트 매트릭스 애니메이션] ---
  if (P.displayAnimate()) {
    if (currentState == TO_DAEGU || currentState == TO_BUSAN) {
      P.displayReset(); 
    }
  }

  // --- [Part B: 기차 노선도 및 타이머 로직 + 진동 제어 연동] ---
  switch (currentState) {
    case SEOUL_STOP:
      updateMetroMap(SEOUL_STOP, -1);
      if (currentMillis - stateTimer >= stopDuration) {
        currentState = TO_DAEGU; stateTimer = currentMillis;
        trainPos = (float)stationSeoulIdx + 1.0;
        vibTriggeredForThisLeg = false; 
        P.displayText(" seo >>>> dae ", PA_CENTER, 60, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
      }
      break;

    case TO_DAEGU:
      updateMetroMap(TO_DAEGU, stationDaeguIdx);
      
      // 주행 5초 시점(도착 2초 전)에 도달했을 때, 아직 안 일어난 손님(isOccupied)만 골라서 진동 트리거!
      if (!vibTriggeredForThisLeg && (currentMillis - stateTimer >= travelDuration - 2000)) {
        if (isOccupied) {
          startVibration(2000); 
          showOLED("Approaching", "Stand up !");
        } else {
          Serial.println("[SYSTEM] 승객이 이미 4초 시점에 기립하였으므로 깨움 진동을 패스합니다.");
        }
        vibTriggeredForThisLeg = true; 
      }

      if (currentMillis - stateTimer >= travelDuration) {
        currentState = DAEGU_STOP; stateTimer = currentMillis;
        P.displayText("DAEGU", PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
      }
      break;

    case DAEGU_STOP:
      updateMetroMap(DAEGU_STOP, -1);
      if (currentMillis - stateTimer >= stopDuration) {
        currentState = TO_BUSAN; stateTimer = currentMillis;
        trainPos = (float)stationDaeguIdx + 1.0;
        vibTriggeredForThisLeg = false; 
        P.displayText(" dae >>>> bus ", PA_CENTER, 60, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
      }
      break;

    case TO_BUSAN:
      updateMetroMap(TO_BUSAN, stationBusanIdx);
      
      // 부산역 도착 2초 전, 아직 안 일어난 손님(isOccupied)만 진동 모터 울림
      if (!vibTriggeredForThisLeg && (currentMillis - stateTimer >= travelDuration - 2000)) {
        if (isOccupied) {
          startVibration(2000); 
          showOLED("Approaching", "Stand up !");
        } else {
          Serial.println("[SYSTEM] 승객이 이미 4초 시점에 기립하였으므로 깨움 진동을 패스합니다.");
        }
        vibTriggeredForThisLeg = true; 
      }

      if (currentMillis - stateTimer >= travelDuration) {
        currentState = BUSAN_STOP; stateTimer = currentMillis;
        P.displayText("BUSAN", PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
      }
      break;

    case BUSAN_STOP:
      updateMetroMap(BUSAN_STOP, -1);
      if (currentMillis - stateTimer >= stopDuration) {
        currentState = SEOUL_STOP; stateTimer = currentMillis;
        P.displayText("SEOUL", PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
      }
      break;
  }

  // --- [Part C: 발표 시나리오 연동 역별 카드 조건 분기 통합] ---
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50);

  if (success) {
    if (memcmp(uid, standingCard, 7) == 0) {
      // [시나리오 1] 서울역 정차 중일 때만 입석카드를 통한 대구행 발권 허용
      if (currentState == SEOUL_STOP) {
        showOLED("Standing Pass", "Seoul->Daegu OK");
        startOperateSeat("Seat 1A");
      } else {
        showOLED("Standing Pass", "No Auth.");
        oledResetTime = currentMillis;
        waitingForOledReset = true;
      }
    }
    else if (memcmp(uid, standardCard, 7) == 0) {
      // [시나리오 2] 대구역 정차 중일 때만 새 일반 지정석 승객 탑승 허용
      if (currentState == DAEGU_STOP) {
        showOLED("Standard Pass", "Moving...");
        startOperateSeat("Seat 1A");
      } else {
        showOLED("Standard Pass", "No Auth.");
        oledResetTime = currentMillis;
        waitingForOledReset = true;
      }
    }
    else if (memcmp(uid, groupCard, 7) == 0) {
      // [시나리오 3] 회차 완료 후 서울역 복귀 상태에서 단체석 일괄 가동 연동
      if (currentState == SEOUL_STOP) {
        showOLED("Group Ticket", "1A, 2A, 3A Open");
        startOperateSeat("Group Seats");
      } else {
        showOLED("Group Ticket", "No Auth.");
        oledResetTime = currentMillis;
        waitingForOledReset = true;
      }
    }
  }
}

// --- [기능 함수: 네오픽셀 노선도 업데이트 (요청하신 베이직 원본 상태 100% 동일)] ---
void updateMetroMap(TrainState state, int destinationIdx) {
  for (int i = 0; i < NUM_LEDS; i++) setMirrorPixelColor(i, strip.Color(40, 40, 40)); 
  setMirrorPixelColor(stationSeoulIdx, strip.Color(255, 0, 0));
  setMirrorPixelColor(stationDaeguIdx, strip.Color(255, 0, 0));
  setMirrorPixelColor(stationBusanIdx, strip.Color(255, 0, 0));

  unsigned long currentMillis = millis();
  if (state == TO_DAEGU || state == TO_BUSAN) {
    if (currentMillis - pixelTimer > 400) { 
      pixelTimer = currentMillis;
      trainPos += 1.0;
      if (trainPos >= (float)destinationIdx) trainPos = (float)destinationIdx - 1.0;
    }
    setMirrorPixelColor((int)trainPos, strip.Color(0, 255, 0));
  } else {
    int targetIdx = (state == SEOUL_STOP) ? stationSeoulIdx : (state == DAEGU_STOP) ? stationDaeguIdx : stationBusanIdx;
    int blueBlink = (sin(currentMillis / 250.0) * 127) + 128; 
    setMirrorPixelColor(targetIdx, strip.Color(0, 0, blueBlink));
  }
  strip.show();
}
