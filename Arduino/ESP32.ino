#include <Arduino.h>

// --- 硬體定義 (ESP32 SuperMini S3 專用) ---
const int SWITCH_PIN = 0;          // 建議使用 GPIO 2，避開 GPIO 0 的啟動問題
#define IMU_RX 20                  // 連接 IMU 的 TX
#define IMU_TX 21                  // 連接 IMU 的 RX

// --- 狀態變數 ---
int lastSwitchState = -1;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// --- 初始化 IMU 序列埠 ---
// 在 S3 上，Serial1 是較穩定的選擇
HardwareSerial IMUSerial(1);

void setup() {
    // 1. 初始化 USB 序列埠 (電腦端)
    // 務必在工具選單開啟 "USB CDC On Boot: Enabled"
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // 等待連線，最多 3 秒
    
    Serial.println("--- ESP32 SuperMini 系統啟動 ---");

    // 2. 初始化與 IMU 的通訊 (UART1)
    IMUSerial.begin(115200, SERIAL_8N1, IMU_RX, IMU_TX);

    // 3. 配置開關腳位
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    
    // 4. 觸發 IMU (發送空白鍵讓 IMU 開始運作或歸零)
    delay(1000);
}

void loop() {
    // --- 部分 A：處理實體開關 ---
    int reading = digitalRead(SWITCH_PIN);
    if (reading != lastSwitchState) {
        if ((millis() - lastDebounceTime) > debounceDelay) {
            lastSwitchState = reading;
            if (reading == LOW) {
                Serial.println("START");
            } else {
                Serial.println("STOP");
            }
            lastDebounceTime = millis();
        }
    }

    // --- 部分 B：接收並轉發 IMU 資料 ---
    while (IMUSerial.available()) {
        String imuData = IMUSerial.readStringUntil('\n');
        imuData.trim(); // 去除前後空格或換行符號
        
        if (imuData.length() > 0) {
            // 轉發給電腦 (加上標頭方便 Python 辨識)
            Serial.println(imuData);
        }
    }
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == ' ') {
            IMUSerial.write(' '); // 轉發空白鍵給 9DoF Razor IMU
        }
    }
}