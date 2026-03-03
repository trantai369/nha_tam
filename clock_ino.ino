
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include "time.h"
#include <ESP32Servo.h>
#include <stdarg.h>

//#include "ESP8266WiFi.h"//
//#include "PubSubClient.h"//
//#include "IRremoteESP8266.h"//
//#include "IRsend.h"//
#include "iot47_wifi_ota.h" //
//#include "ESPAsyncTCP.h"//
#include "ESPAsyncWebServer.h"//


#define TFT_MOSI 7
#define TFT_SCLK 15
#define TFT_CS   5
#define TFT_DC   6
#define TFT_RST  4
#define USE_HSPI_PORT

TFT_eSPI tft = TFT_eSPI();

WiFiClient espClient;
//PubSubClient client(espClient);
AsyncWebServer server(80);




const char* ssid = "VIETTEL_Thuy Ngan";
const char* password = "147258369";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;
String statusLine = "IP: CHUA KET NOI";
String lastHomeDisplayedStatus = "";
String lastWaterDisplayedStatus = "";

// ================== GPIO ==================
#define RELAY_PIN 8
#define TEMP_SENSOR_PIN 1
#define SERVO_WATER_PIN 13
#define LIGHT_PIN 12
#define IR_SENSOR_PIN 3
#define SERVO_FLUSH_PIN 14
#define BUTTON_WATER_PIN 21
#define BUTTON_UP_PIN 46
#define BUTTON_DOWN_PIN 48
#define LD2410_OUT_PIN 16

Servo servoWater;
Servo servoFlush;

// ================== MODE & TEMPERATURE ==================
#define HOME_MODE 0
#define WATER_CONTROL_MODE 1

int currentMode = HOME_MODE;
int lastMode = 99;

float currentTemp = 25.0;
float lastDisplayedCurrentTemp = -999.0;
float setTemp = 25.0;
float lastDisplayedSetTemp = -999.0;
float initialWaterTemp = 25.0;  // Lưu nhiệt độ ban đầu khi bật nước
const float SET_TEMP_MIN = 15.0f;
const float SET_TEMP_MAX = 45.0f;

int lastMinute = -1;
int lastHour = -1;
int lastDay = -1;

// ================== CALIBRATION DATA ==================
struct CalibrationData {
  float minTemp;      // Nhiệt độ @ góc 0°
  float maxTemp;      // Nhiệt độ @ góc 180°
  int minAngle;       // Góc tối thiểu
  int maxAngle;       // Góc tối đa
};

CalibrationData calib = {20.0, 45.0, 0, 180};  // Giá trị mặc định

bool inCalibrationMode = false;
int calibStep = 0;

int servoAngle = 90;

// ================== STATE ==================
bool waterOn = false;
bool lastWaterOn = !waterOn;
bool lightOn = false;
bool lastLightOn = !lightOn;
bool personDetected = false;
bool lastPersonDetected = !personDetected;
bool toiletOccupied = false;
bool lastToiletOccupied = !toiletOccupied;

unsigned long toiletDetectTime = 0;
const unsigned long TOILET_OCCUPY_CONFIRM_TIME = 15000;
const unsigned long TOILET_SENSOR_STABLE_TIME = 350;
const unsigned long TOILET_LEAVE_FLUSH_DELAY = 5000;
bool toiletSignalStable = false;
bool autoFlushPending = false;
unsigned long autoFlushRequestedTime = 0;

unsigned long waterStartTime = 0;
const unsigned long WATER_TIMEOUT = 1800000;

unsigned long lastActivityTime = 0;
const unsigned long SCREEN_OFF_TIMEOUT = 10000;
bool screenOn = true;

volatile bool buttonWaterPressed = false;
unsigned long lastButtonTime = 0;
unsigned long buttonWaterPressTime = 0;  // Thời điểm nhấn nút WATER
bool buttonWaterHeld = false;             // Flag: nút được giữ >= 2s
unsigned long lastUpButtonTime = 0;
unsigned long lastDownButtonTime = 0;
const unsigned long REPEAT_DELAY = 100;
const unsigned long WATER_HOLD_TIME = 2000;  // 2 giây để bật relay

void IRAM_ATTR handleButtonUp() {
  if (millis() - lastButtonTime > 200) {
    lastButtonTime = millis();
  }
}

void IRAM_ATTR handleButtonDown() {
  if (millis() - lastButtonTime > 200) {
    lastButtonTime = millis();
  }
}

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define CENTER_X 120
#define CENTER_Y 120
#define CIRCLE_RADIUS 110

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_YELLOW 0xFFE0
#define COLOR_ORANGE 0xFE20
#define COLOR_CYAN 0x07FF
#define COLOR_GOLD 0xFEA0
#define COLOR_GOLD_DARK 0x8B40
#define HOME_STATUS_BOX_X 20
#define HOME_STATUS_BOX_Y (CENTER_Y + 24)
#define HOME_STATUS_BOX_W 200
#define HOME_STATUS_BOX_H 24
#define WATER_STATUS_BOX_X 60
#define WATER_STATUS_BOX_Y 184
#define WATER_STATUS_BOX_W 120
#define WATER_STATUS_BOX_H 16
#define STATUS_MARQUEE_STEP_MS 180
#define TFT_SERIAL_QUEUE_SIZE 80
#define TFT_SERIAL_MIN_HOLD_MS 1200

void drawCircleBorder(uint16_t color) {
  tft.drawCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, color);
}

float getSetTempMinBound();
void enforceSetTempNotLowerThanCurrent();

struct StatusMarqueeState {
  String text;
  int offset;
  unsigned long lastStepMs;
};

StatusMarqueeState homeStatusMarquee = {"", 0, 0};
StatusMarqueeState waterStatusMarquee = {"", 0, 0};
String lastRuntimeStatusSerial = "";
String lastFullStatusSerial = "";
String tftSerialQueue[TFT_SERIAL_QUEUE_SIZE];
int tftSerialQueueHead = 0;
int tftSerialQueueCount = 0;
String serialMirrorPartialLine = "";
String currentTftSerialLine = "";
unsigned long currentTftSerialLineSince = 0;

String toVietnameseNoDau(String text)
{
  const char* fromChars[] = {
    "á","à","ả","ã","ạ","ă","ắ","ằ","ẳ","ẵ","ặ","â","ấ","ầ","ẩ","ẫ","ậ",
    "Á","À","Ả","Ã","Ạ","Ă","Ắ","Ằ","Ẳ","Ẵ","Ặ","Â","Ấ","Ầ","Ẩ","Ẫ","Ậ",
    "é","è","ẻ","ẽ","ẹ","ê","ế","ề","ể","ễ","ệ",
    "É","È","Ẻ","Ẽ","Ẹ","Ê","Ế","Ề","Ể","Ễ","Ệ",
    "í","ì","ỉ","ĩ","ị","Í","Ì","Ỉ","Ĩ","Ị",
    "ó","ò","ỏ","õ","ọ","ô","ố","ồ","ổ","ỗ","ộ","ơ","ớ","ờ","ở","ỡ","ợ",
    "Ó","Ò","Ỏ","Õ","Ọ","Ô","Ố","Ồ","Ổ","Ỗ","Ộ","Ơ","Ớ","Ờ","Ở","Ỡ","Ợ",
    "ú","ù","ủ","ũ","ụ","ư","ứ","ừ","ử","ữ","ự",
    "Ú","Ù","Ủ","Ũ","Ụ","Ư","Ứ","Ừ","Ử","Ữ","Ự",
    "ý","ỳ","ỷ","ỹ","ỵ","Ý","Ỳ","Ỷ","Ỹ","Ỵ",
    "đ","Đ"
  };
  const char* toChars[] = {
    "a","a","a","a","a","a","a","a","a","a","a","a","a","a","a","a","a",
    "A","A","A","A","A","A","A","A","A","A","A","A","A","A","A","A","A",
    "e","e","e","e","e","e","e","e","e","e","e",
    "E","E","E","E","E","E","E","E","E","E","E",
    "i","i","i","i","i","I","I","I","I","I",
    "o","o","o","o","o","o","o","o","o","o","o","o","o","o","o","o","o",
    "O","O","O","O","O","O","O","O","O","O","O","O","O","O","O","O","O",
    "u","u","u","u","u","u","u","u","u","u","u",
    "U","U","U","U","U","U","U","U","U","U","U",
    "y","y","y","y","y","Y","Y","Y","Y","Y",
    "d","D"
  };

  const int mapCount = sizeof(fromChars) / sizeof(fromChars[0]);
  for (int i = 0; i < mapCount; i++) {
    text.replace(fromChars[i], toChars[i]);
  }

  String ascii = "";
  for (unsigned int i = 0; i < text.length(); i++) {
    unsigned char c = text[i];
    if ((c >= 32 && c <= 126) || c == '\t') {
      ascii += (c == '\t') ? ' ' : (char)c;
    }
  }

  while (ascii.indexOf("  ") >= 0) {
    ascii.replace("  ", " ");
  }
  ascii.trim();
  return ascii;
}

void enqueueTftSerialLine(const String& line)
{
  String sanitized = toVietnameseNoDau(line);
  sanitized.replace("\r", "");
  if (sanitized.length() == 0) {
    return;
  }

  if (tftSerialQueueCount >= TFT_SERIAL_QUEUE_SIZE) {
    tftSerialQueueHead = (tftSerialQueueHead + 1) % TFT_SERIAL_QUEUE_SIZE;
    tftSerialQueueCount--;
  }

  int writeIdx = (tftSerialQueueHead + tftSerialQueueCount) % TFT_SERIAL_QUEUE_SIZE;
  tftSerialQueue[writeIdx] = sanitized;
  tftSerialQueueCount++;
}

bool popTftSerialLine(String& lineOut)
{
  if (tftSerialQueueCount <= 0) {
    return false;
  }
  lineOut = tftSerialQueue[tftSerialQueueHead];
  tftSerialQueueHead = (tftSerialQueueHead + 1) % TFT_SERIAL_QUEUE_SIZE;
  tftSerialQueueCount--;
  return true;
}

void appendSerialMirrorChunk(const String& chunk, bool forceNewline)
{
  for (unsigned int i = 0; i < chunk.length(); i++) {
    char c = chunk[i];
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      enqueueTftSerialLine(serialMirrorPartialLine);
      serialMirrorPartialLine = "";
    } else {
      serialMirrorPartialLine += c;
    }
  }

  if (forceNewline) {
    enqueueTftSerialLine(serialMirrorPartialLine);
    serialMirrorPartialLine = "";
  }
}

void tickTftSerialLine()
{
  if (currentTftSerialLine.length() == 0) {
    String nextLine = "";
    if (popTftSerialLine(nextLine)) {
      currentTftSerialLine = nextLine;
      currentTftSerialLineSince = millis();
    } else {
      return;
    }
  }

  int maxChars = (WATER_STATUS_BOX_W - 8) / 6;
  if (maxChars < 1) {
    maxChars = 1;
  }
  bool isLong = ((int)currentTftSerialLine.length() > maxChars);
  unsigned long requiredMs = TFT_SERIAL_MIN_HOLD_MS;
  if (isLong) {
    requiredMs = ((currentTftSerialLine.length() + 3 + maxChars) * STATUS_MARQUEE_STEP_MS) + 300;
  }

  if (millis() - currentTftSerialLineSince >= requiredMs && tftSerialQueueCount > 0) {
    String nextLine = "";
    if (popTftSerialLine(nextLine)) {
      currentTftSerialLine = nextLine;
      currentTftSerialLineSince = millis();
    }
  }
}

String getCurrentTftSerialLine()
{
  if (currentTftSerialLine.length() == 0) {
    String nextLine = "";
    if (popTftSerialLine(nextLine)) {
      currentTftSerialLine = nextLine;
      currentTftSerialLineSince = millis();
    }
  }
  return currentTftSerialLine;
}

class MirrorSerialStream {
public:
  explicit MirrorSerialStream(HardwareSerial& serial) : serial_(serial) {}

  void begin(unsigned long baud) {
    serial_.begin(baud);
  }

  void begin(unsigned long baud, uint32_t config) {
    serial_.begin(baud, config);
  }

  int available() {
    return serial_.available();
  }

  int read() {
    return serial_.read();
  }

  String readStringUntil(char terminator) {
    return serial_.readStringUntil(terminator);
  }

  template <typename T>
  size_t print(const T& value) {
    appendSerialMirrorChunk(String(value), false);
    return serial_.print(value);
  }

  template <typename T>
  size_t println(const T& value) {
    appendSerialMirrorChunk(String(value), true);
    return serial_.println(value);
  }

  size_t println() {
    appendSerialMirrorChunk("", true);
    return serial_.println();
  }

  int printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    serial_.print(buffer);
    appendSerialMirrorChunk(String(buffer), false);
    return n;
  }

private:
  HardwareSerial& serial_;
};

HardwareSerial& serialHw = ::Serial;
MirrorSerialStream SerialMirror(serialHw);
#define Serial SerialMirror

String buildRuntimeStatusLine()
{
  String runtimeStatus = "";
  if (personDetected) runtimeStatus = "NGUOI";
  if (lightOn) runtimeStatus += (runtimeStatus.length() > 0 ? " | DEN" : "DEN");
  if (toiletOccupied) runtimeStatus += (runtimeStatus.length() > 0 ? " | WC" : "WC");
  if (waterOn) runtimeStatus += (runtimeStatus.length() > 0 ? " | NUOC" : "NUOC");
  if (runtimeStatus.length() == 0) runtimeStatus = "SAN SANG";
  return runtimeStatus;
}

String buildSystemStatusLine()
{
  String fullStatus = "MODE:";
  fullStatus += (currentMode == HOME_MODE) ? "HOME" : "WATER";
  fullStatus += " | R:";
  fullStatus += buildRuntimeStatusLine();
  if (currentMode == WATER_CONTROL_MODE || waterOn) {
    fullStatus += " | T:";
    fullStatus += String(currentTemp, 1);
    fullStatus += "/";
    fullStatus += String(setTemp, 1);
  }
  fullStatus += " | ";
  fullStatus += statusLine;
  return fullStatus;
}

String buildHomeStatusLine()
{
  String mirroredSerial = getCurrentTftSerialLine();
  if (mirroredSerial.length() > 0) {
    return mirroredSerial;
  }
  return buildSystemStatusLine();
}

void publishRuntimeStatusUpdate()
{
  String runtimeStatus = buildRuntimeStatusLine();
  String fullStatus = buildSystemStatusLine();
  if (runtimeStatus != lastRuntimeStatusSerial) {
    Serial.println(String("[RUNTIME] ") + runtimeStatus);
    lastRuntimeStatusSerial = runtimeStatus;
  }
  if (fullStatus != lastFullStatusSerial) {
   // Serial.println(String("[STATUS] ") + fullStatus);
    lastFullStatusSerial = fullStatus;
  }
}

bool drawStatusTextMarquee(const String& text,
                           int boxX,
                           int boxY,
                           int boxW,
                           int boxH,
                           uint16_t textColor,
                           StatusMarqueeState& state,
                           bool forceReset = false)
{
  const int padding = 4;
  const int charWidth = 6;
  const int charHeight = 8;

  int innerX = boxX + padding;
  int innerY = boxY + (boxH - charHeight) / 2;
  int innerW = boxW - (padding * 2);
  int maxChars = innerW / charWidth;
  if (maxChars < 1) {
    return false;
  }

  bool changed = forceReset || (text != state.text);
  if (changed) {
    state.text = text;
    state.offset = 0;
    state.lastStepMs = millis();
  }

  bool needStep = false;
  bool isLong = ((int)text.length() > maxChars);
  unsigned long now = millis();
  if (isLong && (now - state.lastStepMs >= STATUS_MARQUEE_STEP_MS)) {
    int cycleLen = text.length() + 3;
    state.offset--;
    if (state.offset < 0) {
      state.offset = cycleLen - 1;
    }
    state.lastStepMs = now;
    needStep = true;
  }

  if (!changed && !needStep) {
    return false;
  }

  tft.fillRect(innerX, innerY, innerW, charHeight, COLOR_BLACK);
  tft.setTextColor(textColor);
  tft.setTextSize(1);

  if (!isLong) {
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, boxX + boxW / 2, boxY + boxH / 2);
    tft.setTextDatum(MC_DATUM);
    return true;
  }

  String loopText = text + "   " + text;
  String visible = loopText.substring(state.offset, state.offset + maxChars);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(visible, innerX, innerY);
  tft.setTextDatum(MC_DATUM);
  return true;
}

void refreshNetworkStatusLine()
{
  if (WiFi.status() == WL_CONNECTED) {
    statusLine = "IP: " + WiFi.localIP().toString();
  } else {
    statusLine = "IP: CHUA KET NOI";
  }
}

// ================== CALIBRATION MODE ==================
void startCalibration()
{
  inCalibrationMode = true;
  calibStep = 1;
  
  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║     SERVO CALIBRATION MODE             ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  Serial.println("📌 BƯỚC 1: Set servo về 0° (nhiệt độ THẤP)");
  Serial.println("   → Vị trí: van mở hoàn toàn (nước LẠNH)");
  Serial.println("   → Nhấn nút UP khi sẵn sàng\n");
  
  servoWater.write(0);
  Serial.println("   ✓ Servo ở 0°\n");
}

void calibrationStep1_RecordMin()
{
  calib.minTemp = currentTemp;
  calib.minAngle = 0;
  
  Serial.printf("✓ Lưu: minTemp = %.1f°C @ 0°\n\n", calib.minTemp);
  
  calibStep = 2;
  Serial.println("📌 BƯỚC 2: Set servo về 180° (nhiệt độ CAO)");
  Serial.println("   → Vị trí: van khóa hoàn toàn (nước NÓNG)");
  Serial.println("   → Nhấn nút DOWN khi sẵn sàng\n");
  
  servoWater.write(180);
  Serial.println("   ✓ Servo ở 180°\n");
}

void calibrationStep2_RecordMax()
{
  calib.maxTemp = currentTemp;
  calib.maxAngle = 180;
  
  Serial.printf("✓ Lưu: maxTemp = %.1f°C @ 180°\n\n", calib.maxTemp);
  
  Serial.println("╔═════════════════════════════════════════╗");
  Serial.println("║    ✓ CALIBRATION HOÀN TẤT              ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  Serial.printf("📊 Dữ liệu Calibration:\n");
  Serial.printf("   0°:    %.1f°C\n", calib.minTemp);
  Serial.printf("   180°:  %.1f°C\n", calib.maxTemp);
  Serial.printf("   Range: %.1f°C / 180°\n\n", calib.maxTemp - calib.minTemp);
  
  inCalibrationMode = false;
  calibStep = 0;
  
  Serial.println("✓ Sẵn sàng! Servo sẽ tự động điều chỉnh.\n");
  
  servoAngle = 90;
  servoWater.write(90);
}

// ================== PHÁT HIỆN NGƯỜI ==================
bool detectPerson()
{
  static unsigned long lastChangeTime = 0;
  static bool lastState = LOW;
  
  bool currentState = (digitalRead(LD2410_OUT_PIN) == HIGH);
  
  if (currentState != lastState) {
    lastChangeTime = millis();
    lastState = currentState;
    return personDetected;
  }
  
  if (millis() - lastChangeTime >= 500) {
    if (currentState) {
      if (!personDetected) {
        Serial.println("[LD2410-OUT] ✓ NGƯỜI PHÁT HIỆN");
      }
      return true;
    } else {
      if (personDetected) {
        Serial.println("[LD2410-OUT] ► Người đã rời");
      }
      return false;
    }
  }
  
  return personDetected;
}

// ================== SERVO CONTROL VỚI CALIBRATION ==================
/*
 * Map nhiệt độ → góc servo dựa vào calibration
 * Formula: angle = minAngle + (setTemp - minTemp) / (maxTemp - minTemp) * (maxAngle - minAngle)
 */
// ================== SERVO CONTROL - LINEAR + PID SMART ADJUSTMENT ==================
/*
 * Logic servo tuyến tính thông minh:
 * 
 * 1. LINEAR MAPPING:
 *    angle = minAngle + (setTemp - minTemp) / (maxTemp - minTemp) * (maxAngle - minAngle)
 *    → Servo luôn ở góc tương ứng với setTemp
 * 
 * 2. SMART MONITORING (PID):
 *    - Đọc sai số: error = setTemp - currentTemp
 *    - Nếu |error| < 0.3°C → ON DINH (ổn định)
 *    - Nếu error > 0.3°C → TANG (cần tăng nước nóng)
 *    - Nếu error < -0.3°C → GIAM (cần giảm nước nóng)
 * 
 * 3. ADAPTIVE ADJUSTMENT:
 *    - Theo dõi tốc độ thay đổi nhiệt độ
 *    - Điều chỉnh góc servo dựa trên tính toán thông minh
 */

struct ServoState {
  float lastError;
  float errorIntegral;
  unsigned long lastUpdateTime;
};

ServoState servoState = {0.0f, 0.0f, 0};

void updateServo()
{
  if (inCalibrationMode) {
    return;
  }
  
  // ===== KIỂM TRA CALIBRATION =====
  float tempRange = calib.maxTemp - calib.minTemp;
  if (tempRange <= 0) {
    Serial.println("[SERVO-ERR] Chưa calibration! Chạy: tempcalib");
    return;
  }
  
  unsigned long currentTime = millis();
  float dt = (currentTime - servoState.lastUpdateTime) / 1000.0f;
  if (dt < 0.01f) dt = 0.01f;  // Min 10ms
  servoState.lastUpdateTime = currentTime;
  
  // ===== TÍNH SAI SỐ NHIỆT ĐỘ =====
  float error = setTemp - currentTemp;  // Dương = cần tăng, âm = cần giảm
  
  // ===== LINEAR MAPPING: setTemp → servo angle =====
  // Servo luôn ở vị trí tương ứng với setTemp (không phụ thuộc currentTemp)
  float targetAngle = calib.minAngle + 
                      ((setTemp - calib.minTemp) / tempRange) * 
                      (calib.maxAngle - calib.minAngle);
  
  // ===== PID ADAPTIVE BOOST: điều chỉnh góc dựa trên error =====
  // Mục đích: bám sát setTemp càng gần càng tốt (±0.1°C)
  // Cho phép góc servo quay rộng để điều chỉnh tinh tế
  float pidBoost = 0.0f;
  
  if (error > 5.0f) {
    // Error rất lớn (>5°C) → boost tối đa 40°
    pidBoost = constrain(error * 5.0f, 0.0f, 40.0f);
  } else if (error > 3.0f) {
    // Error lớn (3-5°C) → boost 20-35°
    pidBoost = constrain(error * 5.0f, 0.0f, 35.0f);
  } else if (error > 2.0f) {
    // Error vừa phải (2-3°C) → boost 15-25°
    pidBoost = constrain(error * 4.5f, 0.0f, 25.0f);
  } else if (error > 1.0f) {
    // Error trung bình (1-2°C) → boost 8-18°
    pidBoost = constrain(error * 4.0f, 0.0f, 18.0f);
  } else if (error > 0.5f) {
    // Error nhỏ (0.5-1°C) → boost 5-10°
    pidBoost = constrain(error * 6.0f, 0.0f, 10.0f);
  } else if (error > 0.2f) {
    // Error rất nhỏ (0.2-0.5°C) → boost nhẹ 2-5°
    pidBoost = constrain(error * 8.0f, 0.0f, 5.0f);
  } else if (error > 0.0f) {
    // Error cực nhỏ (0-0.2°C) → boost rất nhẹ 0-1°
    pidBoost = constrain(error * 5.0f, 0.0f, 1.0f);
  } else if (error > -0.2f) {
    // Gần đúng (-0.2 đến 0°C) → không boost
    pidBoost = 0.0f;
  } else if (error > -0.5f) {
    // Hơi cao (-0.2 đến -0.5°C) → giảm rất nhẹ -1 đến 0°
    pidBoost = constrain(error * 5.0f, -1.0f, 0.0f);
  } else if (error > -1.0f) {
    // Cao hơn (-0.5 đến -1°C) → giảm nhẹ -5 đến -1°
    pidBoost = constrain(error * 6.0f, -5.0f, -1.0f);
  } else if (error > -2.0f) {
    // Cao (-1 đến -2°C) → giảm vừa phải -10 đến -5°
    pidBoost = constrain(error * 4.0f, -10.0f, -5.0f);
  } else if (error > -3.0f) {
    // Cao hơn (-2 đến -3°C) → giảm -15 đến -10°
    pidBoost = constrain(error * 4.5f, -15.0f, -10.0f);
  } else {
    // Cao rất nhiều (<-3°C) → giảm mạnh -25 đến -15°
    pidBoost = constrain(error * 6.0f, -25.0f, -15.0f);
  }
  
  float adaptiveAngle = targetAngle + pidBoost;
  int newAngle = constrain((int)adaptiveAngle, 0, 180);  // ← MỞ RỘNG TỪ minAngle/maxAngle
  
  // ===== CẬP NHẬT SERVO - LUÔN GHI NẾU setTemp THAY ĐỔI HOẶC ERROR LỚNTHAY ĐỔI =====
  static float lastSetTemp = -999.0f;
  static int lastWrittenAngle = -1;
  
  if (newAngle != lastWrittenAngle || fabs(setTemp - lastSetTemp) > 0.1f) {
    servoAngle = newAngle;
    servoWater.write(servoAngle);
    lastWrittenAngle = newAngle;
    lastSetTemp = setTemp;
    
    Serial.printf("[SERVO-WRITE] setTemp=%.1f°C + PID(%.1f) → Góc=%d° | Error=%+.1f°C\n", 
                  setTemp, pidBoost, servoAngle, error);
  }
  
  // ===== SMART MONITORING - PID FEEDBACK =====
  // Tích phân sai số (để giám sát xu hướng)
  servoState.errorIntegral += error * dt;
  servoState.errorIntegral = constrain(servoState.errorIntegral, -10.0f, 10.0f);
  
  // Đạo hàm sai số (tốc độ thay đổi)
  float errorDerivative = (error - servoState.lastError) / dt;
  servoState.lastError = error;
  
  // ===== PHÂN LOẠI TRẠNG THÁI =====
  String status = "";
  String action = "";
  int statusCode = 0;  // 0=ổn, 1=tăng, -1=giảm
  
  if (fabs(error) < 0.3f) {
    status = "✓ ON DINH";
    statusCode = 0;
    action = "Đã đạt nhiệt độ mong muốn";
  } else if (error > 0.3f) {
    status = "🔥 TANG";
    statusCode = 1;
    action = (error > 3.0f) ? "Cần tăng nhanh" : 
             (error > 1.0f) ? "Cần tăng vừa phải" : "Cần tăng chậm";
  } else {
    status = "❄️ GIAM";
    statusCode = -1;
    action = (error < -3.0f) ? "Cần giảm nhanh" : 
             (error < -1.0f) ? "Cần giảm vừa phải" : "Cần giảm chậm";
  }
  
  // ===== DEBUG CHI TIẾT =====
  static unsigned long lastDetailedDebugTime = 0;
  if (currentTime - lastDetailedDebugTime > 2000) {  // Debug mỗi 2 giây
    Serial.printf("[SERVO-DETAIL] ┌────────────────────────────────┐\n");
    Serial.printf("[SERVO-DETAIL] │ Set: %.1f°C | Current: %.1f°C  │\n", setTemp, currentTemp);
    Serial.printf("[SERVO-DETAIL] │ Error: %+.2f°C | Integral: %.2f │\n", error, servoState.errorIntegral);
    Serial.printf("[SERVO-DETAIL] │ Rate: %+.2f°C/s | Angle: %d°    │\n", errorDerivative, servoAngle);
    Serial.printf("[SERVO-DETAIL] │ Status: %s              │\n", status.c_str());
    Serial.printf("[SERVO-DETAIL] │ Action: %s│\n", action.c_str());
    Serial.printf("[SERVO-DETAIL] └────────────────────────────────┘\n");
    
    // Dự đoán thời gian đạt nhiệt độ
    if (fabs(errorDerivative) > 0.01f) {
      float estimatedTime = fabs(error) / fabs(errorDerivative);
      Serial.printf("[SERVO-ETA] Dự kiến: ~%.0f giây nữa đạt %.1f°C\n", estimatedTime, setTemp);
    }
    
    lastDetailedDebugTime = currentTime;
  }
  
  // ===== SIMPLE STATUS EVERY 5 SECONDS =====
  static unsigned long lastSimpleDebugTime = 0;
  if (currentTime - lastSimpleDebugTime > 5000) {
    Serial.printf("[SERVO] Set=%.1f | Current=%.1f | Error=%+.1f | Angle=%d° | %s | %s\n",
                  setTemp, currentTemp, error, servoAngle, status.c_str(), action.c_str());
    lastSimpleDebugTime = currentTime;
  }
}

// ================== FEEDBACK NHIỆT ĐỘ ==================
// ================== PHẢN HỒI NHIỆT ĐỘ ===================
/*
 * Hiển thị trạng thái nhiệt độ mỗi 2 giây
 * Chỉ khi relay đã bật (waterOn == true)
 */
void printTemperatureFeedback()
{
  static unsigned long lastFeedbackTime = 0;
  
  // Cập nhật mỗi 2 giây
  if (millis() - lastFeedbackTime < 2000) {
    return;
  }
  lastFeedbackTime = millis();
  
  // Chỉ hiển thị khi relay đã bật
  if (!waterOn) {
    return;
  }
  
  // Tính sai số nhiệt độ
  float tempDiff = setTemp - currentTemp;
  String status = "";
  
  // Phân loại trạng thái dựa trên sai số
  if (fabs(tempDiff) < 0.5) {
    status = "✓ ON DINH (ổn định)";
  } else if (tempDiff > 0) {
    status = "🔥 TANG (tăng nhiệt)";
  } else {
    status = "❄️ GIAM (giảm nhiệt)";
  }
  
  // Hiển thị chi tiết
 // Serial.printf("[TEMP-FEEDBACK] Đặt:%.1f°C | Hiện:%.1f°C | Sai:%.1f°C | Servo:%d° | %s\n",
              //  setTemp, currentTemp, tempDiff, servoAngle, status.c_str());
}

// ================== HÀM HỖ TRỢ KHÁC ==================
void drawHomeScreen()
{
  tft.fillScreen(COLOR_BLACK);
  drawCircleBorder(COLOR_GOLD);
  drawCircleBorder(COLOR_GOLD_DARK);
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  char timeBuf[10];
  sprintf(timeBuf, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(4);
  tft.drawString(timeBuf, CENTER_X, CENTER_Y - 40);
  
  lastHour = timeinfo.tm_hour;
  lastMinute = timeinfo.tm_min;
  
  char dateBuf[20];
  sprintf(dateBuf, "%02d/%02d/%04d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  tft.setTextColor(COLOR_GOLD);
  tft.setTextSize(2);
  tft.drawString(dateBuf, CENTER_X, CENTER_Y + 10);
  
  lastDay = timeinfo.tm_mday;
  
  tft.drawRoundRect(HOME_STATUS_BOX_X, HOME_STATUS_BOX_Y, HOME_STATUS_BOX_W, HOME_STATUS_BOX_H, 6, COLOR_GOLD_DARK);
  tft.drawRoundRect(HOME_STATUS_BOX_X + 1, HOME_STATUS_BOX_Y + 1, HOME_STATUS_BOX_W - 2, HOME_STATUS_BOX_H - 2, 6, COLOR_GOLD);
  String mergedStatus = buildHomeStatusLine();
  drawStatusTextMarquee(mergedStatus,
                        HOME_STATUS_BOX_X,
                        HOME_STATUS_BOX_Y,
                        HOME_STATUS_BOX_W,
                        HOME_STATUS_BOX_H,
                        COLOR_GOLD,
                        homeStatusMarquee,
                        true);
  lastHomeDisplayedStatus = mergedStatus;
  
  lastPersonDetected = personDetected;
  lastLightOn = lightOn;
  lastToiletOccupied = toiletOccupied;
  lastWaterOn = waterOn;
  
  if (waterOn) {
    tft.setTextColor(COLOR_CYAN);
    char waterStatusBuf[30];
    sprintf(waterStatusBuf, "NUOC: %.1f C", currentTemp);
    tft.drawString(waterStatusBuf, CENTER_X, CENTER_Y + 60);
    lastDisplayedCurrentTemp = currentTemp;
  }
  
  Serial.println("[DISPLAY] HOME SCREEN vẽ xong");
}

void drawWaterControlScreen()
{
  tft.fillScreen(COLOR_BLACK);
  drawCircleBorder(COLOR_ORANGE);
  
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_ORANGE);
  tft.setTextSize(2);
  tft.drawString("WATER", CENTER_X, 30);
  tft.drawFastHLine(45, 44, 150, COLOR_GOLD_DARK);
  
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);
  tft.drawString("CURRENT", CENTER_X - 40, 70);
  
  char tempBuf[10];
  dtostrf(currentTemp, 4, 1, tempBuf);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.drawString(tempBuf, CENTER_X - 45, 95);
  tft.setTextSize(1);
  tft.drawString("C", CENTER_X - 10, 105);
  
  tft.setTextColor(COLOR_GOLD);
  tft.setTextSize(1);
  tft.drawString("SET", CENTER_X + 40, 70);
  
  dtostrf(setTemp, 4, 1, tempBuf);
  tft.setTextColor(COLOR_GOLD);
  tft.setTextSize(2);
  tft.drawString(tempBuf, CENTER_X + 35, 95);
  tft.setTextSize(1);
  tft.drawString("C", CENTER_X + 70, 105);
  
  tft.setTextColor(COLOR_CYAN);
  tft.setTextSize(1);
  tft.drawString("PHIM +/- DE TANG/GIAM", CENTER_X, 150);
  
  float diff = setTemp - currentTemp;
  String tempStatus = (diff > 1.0) ? "TANG" : (diff < -1.0) ? "GIAM" : "ON";
  uint16_t statusColor = (diff > 1.0) ? COLOR_ORANGE : (diff < -1.0) ? COLOR_CYAN : COLOR_WHITE;
  
  tft.setTextColor(statusColor);
  tft.drawString(tempStatus, CENTER_X, 170);
  
  lastDisplayedCurrentTemp = currentTemp;
  lastDisplayedSetTemp = setTemp;
  
  // Hiển thị servo angle
  char angleStr[10];
  sprintf(angleStr, "Servo:%d°", servoAngle);
  tft.setTextColor(COLOR_CYAN);
  tft.setTextSize(1);
  tft.drawString(angleStr, CENTER_X, 178);

  tft.drawRoundRect(WATER_STATUS_BOX_X, WATER_STATUS_BOX_Y, WATER_STATUS_BOX_W, WATER_STATUS_BOX_H, 5, COLOR_GOLD_DARK);
  tft.drawRoundRect(WATER_STATUS_BOX_X + 1, WATER_STATUS_BOX_Y + 1, WATER_STATUS_BOX_W - 2, WATER_STATUS_BOX_H - 2, 5, COLOR_GOLD);
  String waterStatus = buildHomeStatusLine();
  drawStatusTextMarquee(waterStatus,
                        WATER_STATUS_BOX_X,
                        WATER_STATUS_BOX_Y,
                        WATER_STATUS_BOX_W,
                        WATER_STATUS_BOX_H,
                        COLOR_GOLD,
                        waterStatusMarquee,
                        true);
  lastWaterDisplayedStatus = waterStatus;
  
  Serial.println("[DISPLAY] WATER SCREEN vẽ xong");
}

// ================== UPDATE DISPLAY REAL-TIME ==================
void updateHomeScreenElements()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  // Cập nhật GIỜ (mỗi phút)
  if (timeinfo.tm_hour != lastHour || timeinfo.tm_min != lastMinute) {
    tft.fillRect(45, CENTER_Y - 55, 150, 30, COLOR_BLACK);
    delay(15);
    char timeBuf[10];
    sprintf(timeBuf, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(4);
    tft.drawString(timeBuf, CENTER_X, CENTER_Y - 40);
    lastHour = timeinfo.tm_hour;
    lastMinute = timeinfo.tm_min;
    Serial.printf("[DISPLAY] Cập nhật giờ: %02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min);
  }
  
  // Cập nhật NGÀY (mỗi ngày)
  if (timeinfo.tm_mday != lastDay) {
    tft.fillRect(45, CENTER_Y - 5, 150, 30, COLOR_BLACK);
    delay(15);
    char dateBuf[20];
    sprintf(dateBuf, "%02d/%02d/%04d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    tft.setTextColor(COLOR_GOLD);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(dateBuf, CENTER_X, CENTER_Y + 10);
    lastDay = timeinfo.tm_mday;
    Serial.printf("[DISPLAY] Cập nhật ngày: %02d/%02d/%04d\n", 
                  timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  }
  
  // Cập nhật TRẠNG THÁI (mỗi khi thay đổi)
  String mergedStatus = buildHomeStatusLine();
  bool statusChanged = (personDetected != lastPersonDetected || 
                       lightOn != lastLightOn || 
                       toiletOccupied != lastToiletOccupied || 
                       waterOn != lastWaterOn ||
                       mergedStatus != lastHomeDisplayedStatus);
  
  if (statusChanged) {
    drawStatusTextMarquee(mergedStatus,
                          HOME_STATUS_BOX_X,
                          HOME_STATUS_BOX_Y,
                          HOME_STATUS_BOX_W,
                          HOME_STATUS_BOX_H,
                          COLOR_GOLD,
                          homeStatusMarquee,
                          true);
    Serial.printf("[DISPLAY] Status: %s\n", mergedStatus.c_str());
    
    lastPersonDetected = personDetected;
    lastLightOn = lightOn;
    lastToiletOccupied = toiletOccupied;
    lastWaterOn = waterOn;
    lastHomeDisplayedStatus = mergedStatus;
  } else {
    drawStatusTextMarquee(mergedStatus,
                          HOME_STATUS_BOX_X,
                          HOME_STATUS_BOX_Y,
                          HOME_STATUS_BOX_W,
                          HOME_STATUS_BOX_H,
                          COLOR_GOLD,
                          homeStatusMarquee);
  }
  
  // Cập nhật NHIỆT ĐỘ NƯỚC (mỗi 0.5°C thay đổi)
  if (waterOn && fabs(currentTemp - lastDisplayedCurrentTemp) > 0.5) {
    tft.fillRect(45, CENTER_Y + 53, 150, 16, COLOR_BLACK);
    delay(10);
    
    tft.setTextColor(COLOR_CYAN);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    char waterStatusBuf[30];
    sprintf(waterStatusBuf, "NUOC: %.1f C", currentTemp);
    tft.drawString(waterStatusBuf, CENTER_X, CENTER_Y + 60);
    
    lastDisplayedCurrentTemp = currentTemp;
    //Serial.printf("[DISPLAY] Cập nhật nhiệt độ: %.1f°C\n", currentTemp);
  }
  drawCircleBorder(COLOR_GOLD_DARK);
  drawCircleBorder(COLOR_GOLD);
}

void updateWaterScreenElements()
{
  // Cập nhật CURRENT TEMP (mỗi 0.5°C thay đổi)
  if (fabs(currentTemp - lastDisplayedCurrentTemp) > 0.5) {
    tft.fillRect(35, 80, 80, 35, COLOR_BLACK);
    delay(20);
    
    char tempBuf[10];
    dtostrf(currentTemp, 4, 1, tempBuf);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(tempBuf, CENTER_X - 45, 95);
    delay(10);
    tft.setTextSize(1);
    tft.drawString("C", CENTER_X - 10, 105);
    
    lastDisplayedCurrentTemp = currentTemp;
    Serial.printf("[DISPLAY] Current: %.1f°C\n", currentTemp);
  }
  
  // Cập nhật SET TEMP (mỗi 0.5°C thay đổi)
  if (fabs(setTemp - lastDisplayedSetTemp) > 0.5) {
    tft.fillRect(125, 80, 80, 35, COLOR_BLACK);
    delay(20);
    
    char tempBuf[10];
    dtostrf(setTemp, 4, 1, tempBuf);
    tft.setTextColor(COLOR_GOLD);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(tempBuf, CENTER_X + 35, 95);
    delay(10);
    tft.setTextSize(1);
    tft.drawString("C", CENTER_X + 70, 105);
    
    lastDisplayedSetTemp = setTemp;
    Serial.printf("[DISPLAY] Set: %.1f°C\n", setTemp);
  }
  
  // Cập nhật TRẠNG THÁI NHIỆT (mỗi khi thay đổi)
  float diff = setTemp - currentTemp;
  static String lastTempStatus = "";
  String tempStatus = "";
  uint16_t statusColor = COLOR_WHITE;
  
  if (diff > 1.0) {
    tempStatus = "TANG";
    statusColor = COLOR_ORANGE;
  } else if (diff < -1.0) {
    tempStatus = "GIAM";
    statusColor = COLOR_CYAN;
  } else {
    tempStatus = "ON";
    statusColor = COLOR_WHITE;
  }
  
  if (tempStatus != lastTempStatus) {
    tft.fillRect(35, 162, 170, 14, COLOR_BLACK);
    delay(15);
    
    tft.setTextColor(statusColor);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(tempStatus, CENTER_X, 169);
    
    lastTempStatus = tempStatus;
    Serial.printf("[DISPLAY] Status: %s (diff: %+.1f°C)\n", tempStatus.c_str(), diff);
  }
  
  // Cập nhật SERVO ANGLE
  static int lastDisplayedAngle = -1;
  if (servoAngle != lastDisplayedAngle) {
    tft.fillRect(45, 172, 150, 14, COLOR_BLACK);
    delay(10);
    
    char angleStr[15];
    sprintf(angleStr, "Servo:%d°", servoAngle);
    tft.setTextColor(COLOR_CYAN);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(angleStr, CENTER_X, 178);
    
    lastDisplayedAngle = servoAngle;
    Serial.printf("[DISPLAY] Servo angle: %d°\n", servoAngle);
  }

  String waterStatus = buildHomeStatusLine();
  bool waterStatusChanged = (waterStatus != lastWaterDisplayedStatus);
  if (waterStatusChanged) {
    lastWaterDisplayedStatus = waterStatus;
    Serial.printf("[DISPLAY] Water status: %s\n", waterStatus.c_str());
  }
  drawStatusTextMarquee(waterStatus,
                        WATER_STATUS_BOX_X,
                        WATER_STATUS_BOX_Y,
                        WATER_STATUS_BOX_W,
                        WATER_STATUS_BOX_H,
                        COLOR_GOLD,
                        waterStatusMarquee,
                        waterStatusChanged);

  drawCircleBorder(COLOR_ORANGE);
}

// ================== TEMPERATURE SENSOR CALIBRATION ==================
/*
 * Tinh chỉnh cảm biến nhiệt độ LM35
 * 
 * LM35 đặc tính:
 * - Output: 10mV = 1°C
 * - Range: -55°C đến +150°C
 * - Độ chính xác: ±0.5°C
 * 
 * Tinh chỉnh:
 * 1. OFFSET: Sự sai lệch cố định
 * 2. SCALE: Độ nhạy (gain)
 * 3. CALIBRATION: So sánh với nhiệt kế chuẩn
 */

struct TemperatureSensorCalib {
  float offset;      // Độ lệch cộng thêm (°C)
  float scale;       // Hệ số nhân (gain)
  float minReading;  // Nhiệt độ đo được thấp nhất
  float maxReading;  // Nhiệt độ đo được cao nhất
};

TemperatureSensorCalib tempCalib = {
  0.0f,              // offset = 0 (chưa tinh chỉnh)
  1.0f,              // scale = 1.0 (chưa tinh chỉnh)
  15.0f,             // minReading
  60.0f              // maxReading
};

// ================== ĐỌC VÀ TINH CHỈNH NHIỆT ĐỘ ==================
/*
 * 1. Đọc ADC
 * 2. Convert thành điện áp (3.3V / 4095)
 * 3. Convert thành nhiệt độ (voltage * 100)
 * 4. Áp dụng SCALE: temp = temp * scale
 * 5. Áp dụng OFFSET: temp = temp + offset
 * 6. Giới hạn range
 * 
 * Formula: finalTemp = (adc / 4095 * 3.3 * 100) * scale + offset
 */
float readTemperatureWithCalib()
{
  float sum = 0;
  
  // Lấy 5 lần đọc rồi lấy trung bình
  for (int i = 0; i < 5; i++) {
    int adcValue = analogRead(TEMP_SENSOR_PIN);
    float voltage = (adcValue / 27095.0) * 3.3;  // ← SỬA TỪÂY
    float tempRaw = voltage * 100.0;  // LM35: 10mV = 1°C
    
    // Áp dụng scale & offset
    float tempCalibrated = (tempRaw * tempCalib.scale) + tempCalib.offset;
    
    sum += tempCalibrated;
    delay(2);
  }
  
  float finalTemp = sum / 5.0;
  
  // Giới hạn range
  return constrain(finalTemp, tempCalib.minReading, tempCalib.maxReading);
}

// ================== TEMPERATURE CALIBRATION MODE ==================
/*
 * Hiệu chỉnh cảm biến so sánh với nhiệt kế chuẩn
 * 
 * Phương pháp 2-điểm (2-point calibration):
 * 1. Chuẩn bị 2 nhiệt độ chuẩn (ví dụ: 25°C và 45°C)
 * 2. Đo giá trị cảm biến ở mỗi điểm
 * 3. Tính scale và offset
 */
void startTemperatureCalibration()
{
  Serial.println("\n╔═════════════════════════════════════════╗");
  Serial.println("║   TEMPERATURE SENSOR CALIBRATION       ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
  
  Serial.println("📌 Chuẩn bị:");
  Serial.println("   - Cảm biến LM35");
  Serial.println("   - Nhiệt kế chuẩn (độ chính xác ±0.5°C)");
  Serial.println("   - Nước ở 2 nhiệt độ khác nhau\n");
  
  Serial.println("💡 QUAN TRỌNG: Để servo tối ưu, chọn 2 nhiệt độ ở hai đầu:");
  Serial.println("   - BƯỚC 1: Chọn nhiệt độ THẤP nhất có thể (15-20°C)");
  Serial.println("   - BƯỚC 2: Chọn nhiệt độ CAO nhất có thể (40-45°C)");
  Serial.println("   → Điều này sử dụng toàn bộ phạm vi servo (0-180°)\n");
  
  Serial.println("📌 BƯỚC 1: Nhiệt độ THẤP (~15-20°C)");
  Serial.println("   - Đặt cảm biến vào nước ở nhiệt độ thấp nhất");
  Serial.println("   - Kiểm tra với nhiệt kế chuẩn");
  Serial.println("   - Gõ nhiệt độ chuẩn rồi nhấn ENTER");
  Serial.println("   Format: temp 15.0\n");
}

float readingAtPoint1 = 0.0f;
float standardAtPoint1 = 0.0f;
float readingAtPoint2 = 0.0f;
float standardAtPoint2 = 0.0f;

void processTemperatureCalibrationCommand(String command)
{
  if (command.startsWith("temp ")) {
    // Format: "temp 25.0"
    String valueStr = command.substring(5);
    float standardTemp = valueStr.toFloat();
    
    if (calibStep == 0) {
      // Điểm 1 (nhiệt độ thấp)
      readingAtPoint1 = readTemperatureWithCalib();
      standardAtPoint1 = standardTemp;
      

      
      calibStep = 1;
    }
    else if (calibStep == 1) {
      // Điểm 2 (nhiệt độ cao)
      readingAtPoint2 = readTemperatureWithCalib();
      standardAtPoint2 = standardTemp;
      
    
      // Tính toán scale và offset
      // Scale = ΔStandard / ΔReading
      float readingDiff = readingAtPoint2 - readingAtPoint1;
      float standardDiff = standardAtPoint2 - standardAtPoint1;
      
      if (fabs(readingDiff) < 0.1) {
        Serial.println("✗ Lỗi: Hai điểm quá gần nhau!");
        Serial.println("   Vui lòng chọn 2 nhiệt độ cách xa hơn (ít nhất 10°C)\n");
        calibStep = 0;
        return;
      }
      
     // tempCalib.scale = standardDiff / readingDiff;
    //  tempCalib.offset = standardAtPoint1 - (readingAtPoint1 * tempCalib.scale);
      
    //  Serial.println("╔═════════════════════════════════════════╗");
    //  Serial.println("║   ✓ CALIBRATION HOÀN TẤT               ║");
   //   Serial.println("╚═════════════════════════════════════════╝\n");
      
    //  Serial.printf("📊 Kết quả Calibration:\n");
    //  Serial.printf("   Scale (gain): %.4f\n", tempCalib.scale);
     // Serial.printf("   Offset: %.2f°C\n\n", tempCalib.offset);
      
      // Kiểm chứng
    //  Serial.println("📋 Kiểm chứng:");
   //   Serial.printf("   Point 1: %.2f°C (cảm biến) → %.2f°C (sau calib) [chuẩn: %.2f°C]\n",
   //           readingAtPoint1, 
     //               readingAtPoint1 * tempCalib.scale + tempCalib.offset,
    //                standardAtPoint1);
     // Serial.printf("   Point 2: %.2f°C (cảm biến) → %.2f°C (sau calib) [chuẩn: %.2f°C]\n\n",
        //            readingAtPoint2,
      //              readingAtPoint2 * tempCalib.scale + tempCalib.offset,
      //              standardAtPoint2);
      
   //   Serial.println("✓ Calibration được lưu!");
   //   Serial.println("   Công thức: finalTemp = (reading * scale) + offset\n");
      
      calibStep = 0;
    }
  }
}

bool detectToiletOccupancy()
{
  static bool lastRawState = LOW;
  static bool stableState = LOW;
  static unsigned long rawStateChangeTime = 0;

  bool rawState = (digitalRead(IR_SENSOR_PIN) == LOW);
  if (rawState != lastRawState) {
    lastRawState = rawState;
    rawStateChangeTime = millis();
  }

  if (millis() - rawStateChangeTime >= TOILET_SENSOR_STABLE_TIME) {
    stableState = rawState;
  }
  toiletSignalStable = stableState;

  if (!toiletOccupied) {
    if (!stableState) {
      // Mất tín hiệu trong lúc đếm xác nhận ngồi => reset bộ đếm về 0.
      if (toiletDetectTime != 0) {
        Serial.printf("[TOILET] Reset dem ngoi (mat tin hieu)");
      }
      toiletDetectTime = 0;
      return toiletOccupied;
    }

    // Chỉ xác nhận khi tín hiệu HIGH liên tục đủ 15 giây.
    if (toiletDetectTime == 0) {
      toiletDetectTime = millis();
      Serial.println("[TOILET] Bat dau dem 15s xac nhan ngoi");
    }
    if (millis() - toiletDetectTime >= TOILET_OCCUPY_CONFIRM_TIME) {
      toiletOccupied = true;
      toiletDetectTime = 0;
      autoFlushPending = false;
      Serial.printf("[TOILET] ✓ Xác nhận người ngồi");
      return true;
    }
  } else {
    // Đã có người ngồi:
    // - Mất tín hiệu => bắt đầu chờ 5s để xác nhận rời.
    // - Có tín hiệu lại trong 5s => hủy chờ xả.
    if (!stableState && !autoFlushPending) {
      autoFlushRequestedTime = millis();
      autoFlushPending = true;
      Serial.printf("[TOILET] Mat tin hieu, dem 5s xac nhan roi");
    } else if (stableState && autoFlushPending) {
      autoFlushPending = false;
      Serial.printf("[TOILET] Tin hieu quay lai, huy xa");
    }
  }

  return toiletOccupied;
}

void processAutoFlush()
{
  if (!autoFlushPending) {
    return;
  }

  if (!toiletOccupied) {
    autoFlushPending = false;
    return;
  }

  if (toiletSignalStable) {
    // Có tín hiệu lại trong 5s => coi như chưa rời.
    autoFlushPending = false;
    return;
  }

  if (millis() - autoFlushRequestedTime >= TOILET_LEAVE_FLUSH_DELAY) {
    toiletOccupied = false;
    autoFlushPending = false;
    Serial.printf("[TOILET] 🚽 Xac nhan roi 5s, tu dong xa");
    servoFlushTrigger();
    return;
  }
}

void relayControl(bool state)
{
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  waterOn = state;
  
  if (state) {
    // BAT NUOC - servo về 0° trước
    Serial.println("[WATER] ► BAT NUOC");
    Serial.println("[SERVO] Reset servo về 0° (mở hết)");
    servoAngle = 0;
    servoWater.write(0);
    delay(500);  // Chờ servo về vị trí
    
    // Lưu nhiệt độ ban đầu
    initialWaterTemp = readTemperatureWithCalib();
    currentTemp = initialWaterTemp;
    
    // Giữ nguyên setTemp đã cài trước đó trong WATER mode.
    // Chỉ giới hạn trong biên an toàn min/max.
    if (setTemp < SET_TEMP_MIN) {
      setTemp = SET_TEMP_MIN;
    }
    if (setTemp > SET_TEMP_MAX) {
      setTemp = SET_TEMP_MAX;
    }
    enforceSetTempNotLowerThanCurrent();
    lastDisplayedSetTemp = -999.0;
    lastDisplayedCurrentTemp = -999.0;
    
    waterStartTime = millis();
    currentMode = WATER_CONTROL_MODE;
    
    // Reset PID state
    servoState.lastError = 0.0f;
    servoState.errorIntegral = 0.0f;
    servoState.lastUpdateTime = millis();
    
   // Serial.printf("[WATER] Lưu nhiệt độ ban đầu: %.1f°C\n", initialWaterTemp);
   // Serial.printf("[WATER] Giữ setTemp đã cài: %.1f°C\n", setTemp);
 //   Serial.printf("[WATER] setTemp range: %.1f°C - %.1f°C\n", SET_TEMP_MIN, SET_TEMP_MAX);
   // Serial.printf("[WATER] waterStartTime: %lu ms\n", waterStartTime);
  } else {
    // TAT NUOC - servo về 0° rồi quay về home
    Serial.println("[WATER] ► TAT NUOC");
    Serial.println("[SERVO] Reset servo về 0° (mở hết)");
    servoAngle = 0;
    servoWater.write(0);
    delay(500);
    
    waterStartTime = 0;
    currentMode = HOME_MODE;
    lastMode = 99;
    
  //  Serial.println("[WATER] Quay về HOME mode");
  }
}

// ================== XỬ LÝ SET TEMPERATURE ==================
/*
 * Điều chỉnh setTemp:
 * - MIN: SET_TEMP_MIN
 * - MAX: SET_TEMP_MAX
 * - Bước: 0.5°C
 */
// ================== ĐIỀU CHỈNH NHIỆT ĐỘ ===================
/*
 * Tăng/Giảm setTemp với giới hạn:
 * - MIN: max(SET_TEMP_MIN, currentTemp)
 * - MAX: SET_TEMP_MAX
 * - Bước: 0.5°C
 */

float getSetTempMinBound()
{
  float minBound = SET_TEMP_MIN;
  if (currentTemp > minBound) {
    minBound = currentTemp;
  }
  return minBound;
}

void enforceSetTempNotLowerThanCurrent()
{
  float minBound = getSetTempMinBound();
  if (setTemp < minBound) {
    setTemp = minBound;
    lastDisplayedSetTemp = -999.0f;  // Buoc TFT refresh gia tri setTemp
  //  Serial.printf("[SETTEMP] Tu dong nang setTemp len %.1fC (>= current %.1fC)\n", setTemp, currentTemp);
  }
}

void increaseSetTemp()
{
  // Tăng nhiệt độ, giới hạn tối đa theo SET_TEMP_MAX
  if (setTemp < SET_TEMP_MAX) {
    setTemp += 0.5;
    if (setTemp > SET_TEMP_MAX) {
      setTemp = SET_TEMP_MAX;
    }
 //   Serial.printf("[BUTTON-UP] ✓ Tăng setTemp → %.1f°C (max: %.1f°C)\n", setTemp, SET_TEMP_MAX);
  } else {
 //   Serial.printf("[BUTTON-UP] ⚠ Đã đạt nhiệt độ tối đa: %.1f°C\n", SET_TEMP_MAX);
  }
}

void decreaseSetTemp()
{
  // Giảm nhiệt độ, giới hạn tối thiểu theo nhiệt độ hiện tại.
  float minBound = getSetTempMinBound();
  if (setTemp > minBound) {
    setTemp -= 0.5;
    if (setTemp < minBound) {
      setTemp = minBound;
    }
 //   Serial.printf("[BUTTON-DOWN] ✓ Giam setTemp → %.1fC (min: %.1fC)\n", setTemp, minBound);
  } else {
  //  Serial.printf("[BUTTON-DOWN] Canh bao: setTemp khong duoc thap hon current (%.1fC)\n", minBound);
  }
}

void lightControl(bool state)
{
  digitalWrite(LIGHT_PIN, state ? HIGH : LOW);
  lightOn = state;
}

void servoFlushTrigger()
{
  servoFlush.write(90);
  delay(500);
  servoFlush.write(0);
  delay(3000);
  servoFlush.write(90);
}

// ===== LOGIC NÚT WATER (TTP223 - Detect HIGH Level) =====
/*
 * Logic chính xác:
 * 1. HOME mode: nhấn → WATER (không bật relay)
 * 2. WATER (relay OFF): 
 *    - Giữ 2s → bật relay
 *    - Thả trước 2s → về HOME
 * 3. WATER (relay ON): nhấn → HOME + tắt relay
 */

void processButtons()
{
  bool waterButtonHigh = (digitalRead(BUTTON_WATER_PIN) == HIGH);  // HIGH = nhấn/giữ
  
  if (waterButtonHigh && !buttonWaterPressed) {
    // ===== PHÁT HIỆN CHUYỂN TỪ LOW → HIGH (VỪA NHẤN) =====
    buttonWaterPressed = true;
    buttonWaterPressTime = millis();
    buttonWaterHeld = false;
    
   // Serial.println("\n[BUTTON-WATER] ✓ Phát hiện nhấn (mức HIGH)");
    
    // HOME mode: chuyển sang WATER (đơn giản, toggle ngay)
    if (currentMode == HOME_MODE) {
     // Serial.println("[BUTTON-WATER] → Vào WATER mode");
      currentMode = WATER_CONTROL_MODE;
      lastMode = 99;
      lastActivityTime = millis();
      screenOn = true;
      buttonWaterPressed = false;  // Xử lý xong, reset
      buttonWaterHeld = false;
    }
    // WATER mode + relay ON: chuyển sang HOME + tắt relay (đơn giản, toggle ngay)
    else if (currentMode == WATER_CONTROL_MODE && waterOn) {
      Serial.println("[BUTTON-WATER] → Về HOME + tắt relay");
      relayControl(false);
      delay(300);
      lastActivityTime = millis();
      screenOn = true;
      buttonWaterPressed = false;  // Xử lý xong, reset
      buttonWaterHeld = false;
    }
    // WATER mode + relay OFF: CHỜ XEM NHẤN HAY GIỮ
    // Không toggle ngay, chờ xem thả hay giữ 2s
    else if (currentMode == WATER_CONTROL_MODE && !waterOn) {
     // Serial.println("[BUTTON-WATER] Chờ: Thả→HOME hoặc Giữ 2s→Bật relay");
      // Giữ buttonWaterPressed = true để tiếp tục đếm thời gian
    }
  } 
  else if (waterButtonHigh && buttonWaterPressed && currentMode == WATER_CONTROL_MODE && !waterOn && !buttonWaterHeld) {
    // ===== NÚT ĐANG ĐƯỢC GIỮ (HIGH) - ĐẾM 2S (WATER MODE, RELAY OFF) =====
    unsigned long holdTime = millis() - buttonWaterPressTime;
    
    // Kiểm tra giữ liên tục >= 2 giây
    if (holdTime >= WATER_HOLD_TIME) {
      buttonWaterHeld = true;
      
      Serial.println("\n╔═════════════════════════════════════╗");
      Serial.println("║ [BUTTON-WATER] ✓ BẬT RELAY (2s)    ║");
      Serial.println("╚═════════════════════════════════════╝\n");
      relayControl(true);
      
      lastActivityTime = millis();
      screenOn = true;
    }
    
    // Debug progress bar (cập nhật mỗi 300ms)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 300) {
      float holdTargetSec = WATER_HOLD_TIME / 1000.0f;
      float progress = (holdTime / 1000.0f) / holdTargetSec;
      int barLength = constrain((int)(progress * 20), 0, 20);
      String progressBar = "[";
      for (int i = 0; i < 20; i++) {
        progressBar += (i < barLength) ? "█" : "░";
      }
      progressBar += "]";
      
     // Serial.printf("[BUTTON-WATER] %s %.1f/%.1fs - Giữ để bật relay hoặc thả để HOME\n", 
    //                progressBar.c_str(), holdTime / 1000.0f, holdTargetSec);
      lastDebugTime = millis();
    }
  } 
  else if (!waterButtonHigh && buttonWaterPressed) {
    // ===== CHUYỂN TỪ HIGH → LOW (VỪA THẢ NÚT) =====
    unsigned long holdTime = millis() - buttonWaterPressTime;
    
    // WATER mode + relay OFF + thả trước 2s → về HOME
    if (currentMode == WATER_CONTROL_MODE && !waterOn && !buttonWaterHeld) {
      if (holdTime < WATER_HOLD_TIME) {
     //   Serial.printf("[BUTTON-WATER] Thả - giữ %.1f/2.0s\n", holdTime / 1000.0f);
       // Serial.println("[BUTTON-WATER] → Về HOME\n");
        currentMode = HOME_MODE;
        lastMode = 99;
        lastActivityTime = millis();
        screenOn = true;
      }
    }
    
    // Reset - sẵn sàng cho lần nhấn tiếp theo
    buttonWaterPressed = false;
    buttonWaterHeld = false;
  }
  
  // ===== NÚT UP - TĂNG NHIỆT ĐỊ (WATER MODE: trước/sau relay đều chỉnh được) =====
  if (currentMode == WATER_CONTROL_MODE && digitalRead(BUTTON_UP_PIN) == HIGH) {
    if (millis() - lastUpButtonTime >= REPEAT_DELAY) {
      increaseSetTemp();
    //  Serial.printf("[BUTTON-UP] Tăng setTemp → %.1f°C\n", setTemp);
      lastActivityTime = millis();
      screenOn = true;
      lastUpButtonTime = millis();
    }
  } else {
    lastUpButtonTime = 0;
  }
  
  // ===== NÚT DOWN - GIẢM NHIỆT ĐỘ (WATER MODE: trước/sau relay đều chỉnh được) =====
  if (currentMode == WATER_CONTROL_MODE && digitalRead(BUTTON_DOWN_PIN) == HIGH) {
    if (millis() - lastDownButtonTime >= REPEAT_DELAY) {
      decreaseSetTemp();
    //  Serial.printf("[BUTTON-DOWN] Giảm setTemp → %.1f°C\n", setTemp);
      lastActivityTime = millis();
      screenOn = true;
      lastDownButtonTime = millis();
    }
  } else {
    lastDownButtonTime = 0;
  }
}

void connectWiFi()
{
  Serial.print("[WIFI] Kết nối: ");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " FAIL");
  refreshNetworkStatusLine();
  Serial.print("📶 ");
  Serial.println(statusLine);
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  delay(2000);
  


  





  
 // Serial.println("\n╔═════════════════════════════════════════╗");
 // Serial.println("║  SMART BATHROOM v5.1 INIT              ║");
 // Serial.println("║  Servo Calibration + Feedback          ║");
 // Serial.println("╚═════════════════════════════════════════╝\n");
  
//  Serial.println("📌 Các lệnh Serial:");
 // Serial.println("   - calibration     : Calibrate servo angle");
 // Serial.println("   - tempcalib       : Calibrate temperature sensor");
 // Serial.println("   - tempread        : Đọc nhiệt độ hiện tại\n");
  
//Serial.print("[TFT] Khởi tạo... ");
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BLACK);
//  Serial.println("✓ OK");
  
  //Serial.print("[GPIO] Cấu hình... ");
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(BUTTON_WATER_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_UP_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLDOWN);
  pinMode(LD2410_OUT_PIN, INPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
//Serial.println("✓ OK");
  
 // Serial.print("[SERVO] Khởi tạo... ");
  servoWater.attach(SERVO_WATER_PIN, 500, 2400);
  servoFlush.attach(SERVO_FLUSH_PIN, 500, 2400);
  servoWater.write(90);
  servoFlush.write(90);
 // Serial.println("✓ OK");
  
//Serial.print("[ADC] Cấu hình... ");
  analogSetAttenuation(ADC_11db);
  Serial.println("✓ OK");
  
 // Serial.print("[INT] Interrupt... ");
  attachInterrupt(digitalPinToInterrupt(BUTTON_UP_PIN), handleButtonUp, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DOWN_PIN), handleButtonDown, RISING);
 // Serial.println("✓ OK");
  
 // Serial.println("[LD2410] GPIO2: OUT pin");
  
  connectWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //Serial.println("[TIME] Đồng bộ NTP");
  
  currentTemp = readTemperatureWithCalib();
  //Serial.printf("[TEMP] Khởi tạo: %.1f°C\n", currentTemp);
  publishRuntimeStatusUpdate();
  
  lastActivityTime = millis();
  screenOn = true;
  
  //Serial.println("\n╔═════════════════════════════════════════╗");
 // Serial.println("║  KHỞI TẠO HOÀN TẤT - SẴN SÀNG          ║");
 // Serial.println("╚═════════════════════════════════════════╝\n");


  iot47_wifi_ota_begin(&server);
  server.begin();





}

// ================== LOOP ==================
void loop()
{
 
   iot47_wifi_ota_loop();
  refreshNetworkStatusLine();
  tickTftSerialLine();
 
 
  // Kiểm tra lệnh Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "calibration") {
      startCalibration();
    }
    else if (command == "tempcalib") {
      startTemperatureCalibration();
    }
    else if (command == "tempread") {
      float temp = readTemperatureWithCalib();
      Serial.printf("📊 Nhiệt độ hiện tại: %.2f°C\n", temp);
      Serial.printf("   Scale: %.4f\n", tempCalib.scale);
      Serial.printf("   Offset: %.2f°C\n\n", tempCalib.offset);
    }
    else if (command.startsWith("temp ")) {
      processTemperatureCalibrationCommand(command);
    }
    else {
      Serial.println("❓ Lệnh không hợp lệ");
      Serial.println("   Lệnh hợp lệ: calibration, tempcalib, tempread");
    }
  }
  
  // Trong quá trình calibration
  if (inCalibrationMode) {
    if (calibStep == 1 && digitalRead(BUTTON_UP_PIN) == HIGH) {
      delay(500);
      calibrationStep1_RecordMin();
    }
    if (calibStep == 2 && digitalRead(BUTTON_DOWN_PIN) == HIGH) {
      delay(500);
      calibrationStep2_RecordMax();
    }
    delay(100);
    return;
  }
  
  currentTemp = readTemperatureWithCalib();
  enforceSetTempNotLowerThanCurrent();
  bool personPresent = detectPerson();
  detectToiletOccupancy();
  
  processButtons();
  
  if (personPresent && !personDetected) {
    personDetected = true;
    lightControl(true);
    Serial.println("[PERSON] ► Phát hiện người!");
  }
  else if (!personPresent && personDetected) {
    personDetected = false;
    if (!toiletOccupied && !waterOn) {
      lightControl(false);
    }
  }

  processAutoFlush();
  publishRuntimeStatusUpdate();
  
  if (waterOn) {
    updateServo();
    printTemperatureFeedback();
    
    if (millis() - waterStartTime > WATER_TIMEOUT) {
      relayControl(false);
      Serial.println("[TIMEOUT] Tắt nước (30min)");
    }
  }
  
  // ===== CẬP NHẬT DISPLAY =====
  if (currentMode != lastMode) {
    // Vẽ màn hình mới
    Serial.printf("[DISPLAY] Mode thay đổi: %d → %d\n", lastMode, currentMode);
    if (currentMode == HOME_MODE) {
      Serial.println("[DISPLAY] Vẽ HOME SCREEN");
      drawHomeScreen();
    } else {
      Serial.println("[DISPLAY] Vẽ WATER SCREEN");
      drawWaterControlScreen();
    }
    lastMode = currentMode;
  } else {
    // Cập nhật từng phần tử
    if (currentMode == HOME_MODE) {
      updateHomeScreenElements();
    } else {
      updateWaterScreenElements();
    }
  }
  
  unsigned long currentTime = millis();
  if (!personDetected && screenOn && (currentTime - lastActivityTime > SCREEN_OFF_TIMEOUT)) {
    tft.writecommand(0x28);
    screenOn = false;
  }
  
  if (personDetected && !screenOn) {
    tft.writecommand(0x29);
    screenOn = true;
    lastActivityTime = currentTime;
  }
  
  if (personDetected) {
    lastActivityTime = currentTime;
  }
  
  delay(50);
}

