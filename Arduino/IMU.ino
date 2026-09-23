/************************************************************
SparkFun 9DoF Razor IMU M0 — 全數值輸出版
每筆 DMP 資料輸出以下幾行 (同一個取樣時刻):
  #YPR=Yaw,Pitch,Roll        (度,相對於歸零姿態)
  #QUAT=w,x,y,z              (相對於歸零姿態)
  #ACC=x,y,z                 (g,感測器座標)
  #GYR=x,y,z                 (deg/s,感測器座標)
  #MAG=x,y,z                 (uT,感測器座標)
  #TMP=t                     (度C,晶片內部溫度)
按空白鍵:以當下姿態為零點。
*************************************************************/

#include <SparkFunMPU9250-DMP.h>

#define SerialPort Serial1
MPU9250_DMP imu;

// 歸零基準四元數,初始為「不旋轉」(1,0,0,0)
float r0 = 1, r1 = 0, r2 = 0, r3 = 0;

// 溫度變化很慢,每 N 筆才讀一次,減少 I2C 負擔
const int TEMP_EVERY = 20;
int tempCounter = 0;
float lastTemp = 0;

void setup() {
  SerialPort.begin(115200);

  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      SerialPort.println("#ERR=MPU9250_NOT_FOUND");
      delay(5000);
    }
  }

  // 量程設定:要在 dmpBegin() 之前
  imu.setAccelFSR(4);        // ±4 g
  imu.setGyroFSR(2000);      // ±2000 deg/s
  imu.setCompassSampleRate(20);  // 磁力計取樣率 (Hz),上限 100

  // DMP:四元數 + 陀螺零偏校正 + 加速度/角速度放進 FIFO
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |
               DMP_FEATURE_GYRO_CAL |
               DMP_FEATURE_SEND_RAW_ACCEL |
               DMP_FEATURE_SEND_CAL_GYRO,
               20);          // FIFO 輸出頻率 (Hz)

  SerialPort.println("#INFO=READY");
}

void loop() {
  // ---------- 1. 空白鍵歸零 ----------
  if (SerialPort.available() > 0) {
    char c = SerialPort.read();
    if (c == ' ') {
      r0 = imu.calcQuat(imu.qw);
      r1 = imu.calcQuat(imu.qx);
      r2 = imu.calcQuat(imu.qy);
      r3 = imu.calcQuat(imu.qz);
      SerialPort.println("#System_Zeroed");
    }
  }

  // ---------- 2. 讀 DMP FIFO (四元數、加速度、角速度) ----------
  if (!imu.fifoAvailable()) return;
  if (imu.dmpUpdateFifo() != INV_SUCCESS) return;

  // 當下四元數
  float n0 = imu.calcQuat(imu.qw);
  float n1 = imu.calcQuat(imu.qx);
  float n2 = imu.calcQuat(imu.qy);
  float n3 = imu.calcQuat(imu.qz);

  // 相對姿態 q_rel = conj(q_ref) ⊗ q_now
  float a0 = r0, a1 = -r1, a2 = -r2, a3 = -r3;
  float q0 = a0*n0 - a1*n1 - a2*n2 - a3*n3;
  float q1 = a0*n1 + a1*n0 + a2*n3 - a3*n2;
  float q2 = a0*n2 - a1*n3 + a2*n0 + a3*n1;
  float q3 = a0*n3 + a1*n2 - a2*n1 + a3*n0;

  // 四元數 -> 歐拉角
  float yaw  = atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
  float sp   = constrain(-2.0 * (q1*q3 - q0*q2), -1.0, 1.0);  // 防 asin NaN
  float pitch = asin(sp);
  float roll = atan2(2.0 * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
  yaw   *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  // 加速度 (g)、角速度 (deg/s)
  float ax = imu.calcAccel(imu.ax), ay = imu.calcAccel(imu.ay), az = imu.calcAccel(imu.az);
  float gx = imu.calcGyro(imu.gx),  gy = imu.calcGyro(imu.gy),  gz = imu.calcGyro(imu.gz);

  // ---------- 3. 磁力計 (不在 FIFO 裡,另外讀) ----------
  imu.updateCompass();
  float mx = imu.calcMag(imu.mx), my = imu.calcMag(imu.my), mz = imu.calcMag(imu.mz);

  // ---------- 4. 溫度 (每 TEMP_EVERY 筆讀一次) ----------
  if (++tempCounter >= TEMP_EVERY) {
    tempCounter = 0;
    if (imu.update(UPDATE_TEMP) == INV_SUCCESS) {
      lastTemp = imu.temperature / 65536.0;   // 函式庫回傳 q16 定點數
    }
  }

  // ---------- 5. 輸出 ----------
  printLine("#YPR=",  yaw, pitch, roll, 2);
  printQuat(q0, q1, q2, q3);
  printLine("#ACC=",  ax, ay, az, 3);
  printLine("#GYR=",  gx, gy, gz, 2);
  printLine("#MAG=",  mx, my, mz, 2);
  SerialPort.print("#TMP=");
  SerialPort.println(lastTemp, 2);
}

void printLine(const char *tag, float a, float b, float c, int digits) {
  SerialPort.print(tag);
  SerialPort.print(a, digits); SerialPort.print(",");
  SerialPort.print(b, digits); SerialPort.print(",");
  SerialPort.println(c, digits);
}

void printQuat(float w, float x, float y, float z) {
  SerialPort.print("#QUAT=");
  SerialPort.print(w, 4); SerialPort.print(",");
  SerialPort.print(x, 4); SerialPort.print(",");
  SerialPort.print(y, 4); SerialPort.print(",");
  SerialPort.println(z, 4);
}
