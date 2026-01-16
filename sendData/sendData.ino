#include <Wire.h>

const int MPU_ADDR = 0x68;
long prevTime;

// === KALMAN FILTER CLASS ===
class Kalman {
  public:
    Kalman() {
      Q_angle = 0.001; 
      Q_bias = 0.003;  
      R_measure = 0.03; 
      angle = 0; 
      bias = 0;
      P[0][0] = 0; P[0][1] = 0; P[1][0] = 0; P[1][1] = 0;
    }

    float getAngle(float newAngle, float newRate, float dt) {
      rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    };

  private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
};

Kalman kalmanPitch;
Kalman kalmanRoll;

float kalAnglePitch = 0, kalAngleRoll = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float yaw = 0;

void setup() {
  Serial.begin(115200); // Pastikan Python juga 115200
  Wire.begin();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Kalibrasi Gyro Cepat (Sekitar 1-2 detik)
  // JANGAN GERAKKAN SENSOR SAAT BOOTING
  long num_readings = 500;
  float sumGX = 0, sumGY = 0, sumGZ = 0;

  for (int i = 0; i < num_readings; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    sumGX += (int16_t)(Wire.read() << 8 | Wire.read());
    sumGY += (int16_t)(Wire.read() << 8 | Wire.read());
    sumGZ += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroXoffset = sumGX / num_readings;
  gyroYoffset = sumGY / num_readings;
  gyroZoffset = sumGZ / num_readings;
  
  prevTime = millis();
}

void loop() {
  // 1. BACA DATA
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() < 14) return;

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  // 2. HITUNG DT
  long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // 3. KONVERSI UNIT
  float gyroXrate = (gx_raw - gyroXoffset) / 131.0;
  float gyroYrate = (gy_raw - gyroYoffset) / 131.0;
  float gyroZrate = (gz_raw - gyroZoffset) / 131.0;

  // Gunakan 16384.0 untuk range default +/- 2g
  float ax_g = ax_raw / 16384.0;
  float ay_g = ay_raw / 16384.0;
  float az_g = az_raw / 16384.0;

  // 4. HITUNG SUDUT (KALMAN)
  float accelPitch = atan2(ay_g, az_g) * RAD_TO_DEG;
  float accelRoll  = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;

  kalAnglePitch = kalmanPitch.getAngle(accelPitch, gyroXrate, dt);
  kalAngleRoll  = kalmanRoll.getAngle(accelRoll, gyroYrate, dt);
  
  // Yaw sederhana (Drift accumulation)
  if (abs(gyroZrate) > 0.3) yaw += gyroZrate * dt;

  // 5. ISOLASI GRAVITASI (DAPATKAN LINEAR ACCEL)
  // Proyeksi gravitasi berdasarkan sudut Kalman yang stabil
  float pitchRad = kalAnglePitch * DEG_TO_RAD;
  float rollRad  = kalAngleRoll * DEG_TO_RAD;

  float gravX_est = -sin(rollRad);
  float gravY_est = sin(pitchRad) * cos(rollRad);
  float gravZ_est = cos(pitchRad) * cos(rollRad);

  // Linear Acceleration (Data Pergeseran)
  float lin_ax = ax_g - gravX_est;
  float lin_ay = ay_g - gravY_est;
  float lin_az = az_g - gravZ_est;

  // Deadzone filter untuk menghilangkan noise saat diam
  if (abs(lin_ax) < 0.04) lin_ax = 0;
  if (abs(lin_ay) < 0.04) lin_ay = 0;
  if (abs(lin_az) < 0.04) lin_az = 0;

  // 6. KIRIM KE PYTHON (Format: P,R,Y,Ax,Ay,Az)
  static long lastPrint = 0;
  if (millis() - lastPrint > 40) { // Kirim setiap 40ms (25Hz)
    Serial.print(kalAnglePitch, 2); Serial.print(",");
    Serial.print(kalAngleRoll, 2);  Serial.print(",");
    Serial.print(yaw, 2);           Serial.print(",");
    Serial.print(lin_ax, 3);        Serial.print(","); // Gunakan 3 desimal untuk presisi akselerasi
    Serial.print(lin_ay, 3);        Serial.print(",");
    Serial.println(lin_az, 3);      // Akhiri dengan newline
    
    lastPrint = millis();
  }
}