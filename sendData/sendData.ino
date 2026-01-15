#include <Wire.h>

const int MPU_ADDR = 0x68; 
float pitch = 0, roll = 0, yaw = 0;
long prevTime;

// Variabel untuk Filter Gravitasi
float gravX = 0, gravY = 0, gravZ = 0;
float alpha = 0.8; // Faktor Filter (Semakin besar = semakin lambat adaptasi gravitasi)

// Offset Gyro tetap perlu
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

void setup() {
  Serial.begin(115200); 
  delay(1000); 
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // === KALIBRASI GYRO SAJA (ACCEL GA PERLU) ===
  Serial.println("KALIBRASI GYRO...");
  long num_readings = 500;
  float sumGX=0, sumGY=0, sumGZ=0;

  for (int i = 0; i < num_readings; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    sumGX += Wire.read() << 8 | Wire.read();
    sumGY += Wire.read() << 8 | Wire.read();
    sumGZ += Wire.read() << 8 | Wire.read();
    delay(2);
  }
  gyroXoffset = sumGX / num_readings;
  gyroYoffset = sumGY / num_readings;
  gyroZoffset = sumGZ / num_readings;
  
  // Inisialisasi nilai awal gravitasi
  // Baca sekali untuk isi variabel grav
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  gravX = Wire.read() << 8 | Wire.read();
  gravY = Wire.read() << 8 | Wire.read();
  gravZ = Wire.read() << 8 | Wire.read();
  
  prevTime = millis();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Baca 14 register sekaligus

  if (Wire.available() < 14) return;

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // --- 1. SUDUT (PITCH/ROLL/YAW) ---
  float gyro_pitch = (gx_raw - gyroXoffset) / 131.0;
  float gyro_roll  = (gy_raw - gyroYoffset) / 131.0;
  float gyro_yaw   = (gz_raw - gyroZoffset) / 131.0;

  if (abs(gyro_pitch) < 0.1) gyro_pitch = 0;
  if (abs(gyro_roll) < 0.1) gyro_roll = 0;
  if (abs(gyro_yaw) < 0.1) gyro_yaw = 0;

  float ax_f = (float)ax_raw;
  float ay_f = (float)ay_raw;
  float az_f = (float)az_raw;

  float accel_pitch = atan2(ay_f, az_f) * 57.296; 
  float accel_roll  = atan2(-ax_f, sqrt(ay_f*ay_f + az_f*az_f)) * 57.296;

  pitch = 0.96 * (pitch + gyro_pitch * dt) + 0.04 * accel_pitch;
  roll  = 0.96 * (roll  + gyro_roll  * dt) + 0.04 * accel_roll;
  yaw   = yaw + gyro_yaw * dt;

  // --- 2. ISOLASI GERAKAN LINEAR (Hapus Gravitasi) ---
  // Algoritma: Low Pass Filter untuk cari Gravitasi, lalu kurangi Raw Data
  
  gravX = alpha * gravX + (1 - alpha) * ax_raw;
  gravY = alpha * gravY + (1 - alpha) * ay_raw;
  gravZ = alpha * gravZ + (1 - alpha) * az_raw;

  // Linear Accel = Raw - Gravity
  float lin_ax = ax_raw - gravX;
  float lin_ay = ay_raw - gravY;
  float lin_az = az_raw - gravZ;

  // Skala & Deadzone
  // Bagi 4096 agar sensitif
  float ax_g = lin_ax / 4096.0;
  float ay_g = lin_ay / 4096.0;
  float az_g = lin_az / 4096.0;

  // Deadzone (Hapus noise kecil)
  if (abs(ax_g) < 0.1) ax_g = 0;
  if (abs(ay_g) < 0.1) ay_g = 0;
  if (abs(az_g) < 0.1) az_g = 0;

  // OUTPUT
  static long lastPrint = 0;
  if (millis() - lastPrint > 40) { 
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(roll, 2);  Serial.print(",");
    Serial.print(yaw, 2);   Serial.print(",");
    Serial.print(ax_g, 2);  Serial.print(",");
    Serial.print(ay_g, 2);  Serial.print(",");
    Serial.println(az_g, 2);
    lastPrint = millis();
  }
}