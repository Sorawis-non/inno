#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/wdt.h>   // Watchdog for Leonardo (AVR)
  #include <avr/io.h>
#endif

// ---------- Leonardo I2C pins ----------
static const uint8_t SDA_PIN = 2;
static const uint8_t SCL_PIN = 3;

// ---------- I2C clock ----------
static uint32_t I2C_CLOCK = 100000UL;  // 100kHz เสถียรสุดเวลาเขย่าแรง

// ---------- MPU registers ----------
static uint8_t MPU_ADDR = 0x68;         // detect 0x68/0x69
static const uint8_t REG_SMPLRT_DIV     = 0x19;
static const uint8_t REG_CONFIG         = 0x1A;
static const uint8_t REG_GYRO_CONFIG    = 0x1B;
static const uint8_t REG_ACCEL_CONFIG   = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2  = 0x1D;
static const uint8_t REG_INT_PIN_CFG    = 0x37;
static const uint8_t REG_USER_CTRL      = 0x6A;
static const uint8_t REG_PWR_MGMT_1     = 0x6B;
static const uint8_t REG_WHO_AM_I       = 0x75;
static const uint8_t REG_ACCEL_XOUT_H   = 0x3B;

// ---------- AK8963 (mag inside MPU9250/55) ----------
static const uint8_t MAG_ADDR   = 0x0C;
static const uint8_t AK_WIA     = 0x00;
static const uint8_t AK_ST1     = 0x02;
static const uint8_t AK_HXL     = 0x03;
static const uint8_t AK_CNTL1   = 0x0A;
static const uint8_t AK_CNTL2   = 0x0B;
static const uint8_t AK_ASAX    = 0x10;

// ---------- math / filters ----------
struct Vector3 { float x, y, z; };

static const float g0 = 9.80665f;
static const float RAD2DEG = 57.29577951308232f;
static const float DEG2RAD = 0.017453292519943295f;

static const float OMEGA_DEADBAND   = 0.03f;
static const float ACCEL_STILL_EPS  = 0.35f;
static const float ACCEL_LP_TAU     = 0.08f;
static const float ACCEL_DEADBAND   = 0.10f;
static const float AVG_TAU          = 1.0f;

static const float OMEGA_DB2 = OMEGA_DEADBAND * OMEGA_DEADBAND;
static const float G_LO2 = (g0 - ACCEL_STILL_EPS) * (g0 - ACCEL_STILL_EPS);
static const float G_HI2 = (g0 + ACCEL_STILL_EPS) * (g0 + ACCEL_STILL_EPS);

// accel/gyro scales (±2000dps, ±4g)
static const float LSB_PER_DPS = 16.4f;
static const float LSB_PER_G4  = 8192.0f;
static const float GYRO_SCALE  = DEG2RAD / LSB_PER_DPS;   // raw * -> rad/s
static const float ACCEL_SCALE = g0 / LSB_PER_G4;         // raw * -> m/s^2

// AK8963 sensitivity (16-bit = 0.15 µT/LSB)
static const float MAG_UT_PER_LSB_16 = 0.15f;

// CM compensation (set {0,0,0} if not needed)
static const Vector3 r_cm = {0.02f, 0.0f, 0.0f};

// update throttling
static const uint16_t IMU_HZ = 200;
static const uint32_t IMU_PERIOD_US = 1000000UL / IMU_HZ;

static const uint16_t MAG_HZ = 100;
static const uint32_t MAG_PERIOD_US = 1000000UL / MAG_HZ;

// ---------- state ----------
static Vector3 omega={0,0,0}, omega_prev={0,0,0};
static Vector3 gyro_bias={0,0,0};
static Vector3 accel_raw={0,0,0};
static Vector3 accel_cm_filt={0,0,0};
static Vector3 mag_uT={0,0,0};

static float ADx=0, ADy=0, ADz=0;      // deg
static float AavgX=0, AavgY=0, AavgZ=0, AavgAl=0;

static uint32_t lastMicrosIMU=0;
static uint32_t lastGoodReadMicros=0;
static uint32_t i2cFailStreak=0;

static bool hasMag=false;
static float magAdj[3] = {1,1,1};

static bool streaming=false;
static uint32_t lastPrintMs=0;
static const uint16_t PRINT_EVERY_MS = 20;

// ---------- helpers ----------
static inline Vector3 cross(const Vector3 &a, const Vector3 &b) {
  return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline float norm2(const Vector3 &v) { return v.x*v.x + v.y*v.y + v.z*v.z; }

static void applyI2C() {
  Wire.setClock(I2C_CLOCK);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
}

static bool writeRegDev(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}
static bool readRegsDev(uint8_t addr, uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  int got = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (got != (int)len) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (uint8_t i=0; i<len; i++) buf[i] = Wire.read();
  return true;
}

static inline bool writeReg(uint8_t reg, uint8_t val) { return writeRegDev(MPU_ADDR, reg, val); }
static inline bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) { return readRegsDev(MPU_ADDR, reg, buf, len); }
static inline uint8_t readReg(uint8_t reg) { uint8_t v=0xFF; (void)readRegs(reg,&v,1); return v; }

static uint8_t detectMpuAddr() {
  for (uint8_t a=0x68; a<=0x69; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission(true) == 0) return a;
  }
  return 0;
}

// bus clear (แก้ SDA คา)
static void i2cBusClear() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);

  for (uint8_t i=0; i<9; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  }

  // STOP
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
}

static bool mpuConfigure() {
  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false; // PLL
  delay(30);

  writeReg(REG_CONFIG, 0x03);         // DLPF ~44Hz
  writeReg(REG_SMPLRT_DIV, 0x00);     // 1kHz internal

  writeReg(REG_GYRO_CONFIG,  0x18);   // ±2000 dps
  writeReg(REG_ACCEL_CONFIG, 0x08);   // ±4g
  writeReg(REG_ACCEL_CONFIG2,0x03);

  delay(10);
  return true;
}

static void mpuEnableBypass() {
  writeReg(REG_USER_CTRL, 0x00);
  writeReg(REG_INT_PIN_CFG, 0x02);
  delay(10);
}

static bool ak8963Init() {
  uint8_t wia=0x00;
  if (!readRegsDev(MAG_ADDR, AK_WIA, &wia, 1)) return false;
  if (wia != 0x48) return false;

  writeRegDev(MAG_ADDR, AK_CNTL2, 0x01);
  delay(50);

  writeRegDev(MAG_ADDR, AK_CNTL1, 0x00);
  delay(10);

  writeRegDev(MAG_ADDR, AK_CNTL1, 0x0F);
  delay(10);

  uint8_t asa[3]={128,128,128};
  if (!readRegsDev(MAG_ADDR, AK_ASAX, asa, 3)) return false;

  for (int i=0;i<3;i++) magAdj[i] = ((asa[i] - 128) / 256.0f) + 1.0f;

  writeRegDev(MAG_ADDR, AK_CNTL1, 0x00);
  delay(10);

  writeRegDev(MAG_ADDR, AK_CNTL1, 0x16); // 100Hz, 16-bit
  delay(10);

  return true;
}

static bool ak8963Read(Vector3 &out_uT) {
  uint8_t st1=0;
  if (!readRegsDev(MAG_ADDR, AK_ST1, &st1, 1)) return false;
  if ((st1 & 0x01) == 0) return false;

  uint8_t b[7];
  if (!readRegsDev(MAG_ADDR, AK_HXL, b, 7)) return false;

  int16_t mx = (int16_t)((b[1]<<8) | b[0]);
  int16_t my = (int16_t)((b[3]<<8) | b[2]);
  int16_t mz = (int16_t)((b[5]<<8) | b[4]);
  uint8_t st2 = b[6];
  if (st2 & 0x08) return false;

  out_uT.x = mx * MAG_UT_PER_LSB_16 * magAdj[0];
  out_uT.y = my * MAG_UT_PER_LSB_16 * magAdj[1];
  out_uT.z = mz * MAG_UT_PER_LSB_16 * magAdj[2];
  return true;
}

// recover ที่ “แน่น” ขึ้น
static void recoverI2CAndMPU() {
  i2cBusClear();

#if defined(ARDUINO_ARCH_AVR)
  // reset TWI hardware (กันบัสค้างบางเคส)
  TWCR = 0;
#endif

  Wire.end();
  delay(5);

  Wire.begin();
  applyI2C();

  (void)writeReg(REG_PWR_MGMT_1, 0x80);
  delay(80);

  (void)mpuConfigure();

  mpuEnableBypass();
  hasMag = ak8963Init();

  lastMicrosIMU = micros();
  lastGoodReadMicros = lastMicrosIMU;
  i2cFailStreak = 0;
}

static void resetImuValues() {
  ADx=ADy=ADz=0;
  AavgX=AavgY=AavgZ=AavgAl=0;
  accel_cm_filt = {0,0,0};
}

static void calibrateGyroBiasBootOnly() {
  const int N = 450;
  int32_t sx=0, sy=0, sz=0;
  int good=0;

  for (int i=0; i<N; i++) {
    uint8_t buf[14];
    if (readRegs(REG_ACCEL_XOUT_H, buf, 14)) {
      int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
      int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
      int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);
      sx += gx; sy += gy; sz += gz;
      good++;
    }
    delay(3);
  }
  if (good < 50) return;

  gyro_bias.x = (sx/(float)good) * GYRO_SCALE;
  gyro_bias.y = (sy/(float)good) * GYRO_SCALE;
  gyro_bias.z = (sz/(float)good) * GYRO_SCALE;
}

static void updateIMU() {
  static uint32_t lastReadUs = 0;
  uint32_t now = micros();
  if ((uint32_t)(now - lastReadUs) < IMU_PERIOD_US) return;
  lastReadUs = now;

  uint32_t dtu = (uint32_t)(now - lastMicrosIMU);
  lastMicrosIMU = now;
  if (dtu == 0) return;

  float dt = dtu * 1e-6f;
  if (dt > 0.2f) dt = 1.0f / (float)IMU_HZ;
  float inv_dt = 1.0f / dt;

  if (lastGoodReadMicros && (uint32_t)(now - lastGoodReadMicros) > 150000UL) {
    recoverI2CAndMPU();
    return;
  }

  uint8_t buf[14];
  bool ok=false;
  for (uint8_t tries=0; tries<2; tries++) {
    if (readRegs(REG_ACCEL_XOUT_H, buf, 14)) { ok=true; break; }
    delayMicroseconds(120);
  }

  if (!ok) {
    // กัน “ดูเหมือนหยุด”: เคลียร์ค่าที่จะพิมพ์ออก
    omega = {0,0,0};
    accel_cm_filt = {0,0,0};

    if (++i2cFailStreak >= 6) recoverI2CAndMPU();
    return;
  }

  i2cFailStreak = 0;
  lastGoodReadMicros = now;

  int16_t ax=(int16_t)((buf[0]<<8)|buf[1]);
  int16_t ay=(int16_t)((buf[2]<<8)|buf[3]);
  int16_t az=(int16_t)((buf[4]<<8)|buf[5]);
  int16_t gx=(int16_t)((buf[8]<<8)|buf[9]);
  int16_t gy=(int16_t)((buf[10]<<8)|buf[11]);
  int16_t gz=(int16_t)((buf[12]<<8)|buf[13]);

  omega_prev = omega;

  omega.x = (gx * GYRO_SCALE) - gyro_bias.x;
  omega.y = (gy * GYRO_SCALE) - gyro_bias.y;
  omega.z = (gz * GYRO_SCALE) - gyro_bias.z;

  accel_raw.x = ax * ACCEL_SCALE;
  accel_raw.y = ay * ACCEL_SCALE;
  accel_raw.z = az * ACCEL_SCALE;

  float w2 = norm2(omega);
  float a2 = norm2(accel_raw);
  bool still = (w2 < OMEGA_DB2) && (a2 > G_LO2) && (a2 < G_HI2);
  if (still) omega = {0,0,0};

  ADx += fabsf(omega.x) * dt * RAD2DEG;
  ADy += fabsf(omega.y) * dt * RAD2DEG;
  ADz += fabsf(omega.z) * dt * RAD2DEG;

  Vector3 alpha = {
    (omega.x - omega_prev.x) * inv_dt,
    (omega.y - omega_prev.y) * inv_dt,
    (omega.z - omega_prev.z) * inv_dt
  };

  Vector3 term1 = cross(alpha, r_cm);
  Vector3 wxr   = cross(omega, r_cm);
  Vector3 term2 = cross(omega, wxr);

  Vector3 accel_cm_raw = {
    accel_raw.x - term1.x - term2.x,
    accel_raw.y - term1.y - term2.y,
    accel_raw.z - term1.z - term2.z
  };

  float aAlpha = dt / (ACCEL_LP_TAU + dt);
  accel_cm_filt.x += aAlpha * (accel_cm_raw.x - accel_cm_filt.x);
  accel_cm_filt.y += aAlpha * (accel_cm_raw.y - accel_cm_filt.y);
  accel_cm_filt.z += aAlpha * (accel_cm_raw.z - accel_cm_filt.z);

  if (fabsf(accel_cm_filt.x) < ACCEL_DEADBAND) accel_cm_filt.x = 0;
  if (fabsf(accel_cm_filt.y) < ACCEL_DEADBAND) accel_cm_filt.y = 0;
  if (fabsf(accel_cm_filt.z) < ACCEL_DEADBAND) accel_cm_filt.z = 0;

  float avgAlpha = dt / (AVG_TAU + dt);
  AavgX += avgAlpha * (accel_cm_filt.x - AavgX);
  AavgY += avgAlpha * (accel_cm_filt.y - AavgY);
  AavgZ += avgAlpha * (accel_cm_filt.z - AavgZ);

  float absAvgNow = (fabsf(accel_cm_filt.x)+fabsf(accel_cm_filt.y)+fabsf(accel_cm_filt.z)) * (1.0f/3.0f);
  AavgAl += avgAlpha * (absAvgNow - AavgAl);
}

static void updateMag() {
  if (!hasMag) return;
  static uint32_t lastMagUs = 0;
  uint32_t now = micros();
  if ((uint32_t)(now - lastMagUs) < MAG_PERIOD_US) return;
  lastMagUs = now;

  Vector3 m;
  if (ak8963Read(m)) mag_uT = m;
}

// ---------- Serial output ----------
static void printHeaderCSV() {
  Serial.println(F("Gx,Gy,Gz,ADx,ADy,ADz,ADavg,Ax,Ay,Az,AavgX,AavgY,AavgZ,AavgAl"));
}

static void printOneCsvLine() {
  float ADavg = (ADx + ADy + ADz) * (1.0f/3.0f);

  Serial.print(omega.x, 3); Serial.print(',');
  Serial.print(omega.y, 3); Serial.print(',');
  Serial.print(omega.z, 3); Serial.print(',');

  Serial.print(ADx, 2); Serial.print(',');
  Serial.print(ADy, 2); Serial.print(',');
  Serial.print(ADz, 2); Serial.print(',');
  Serial.print(ADavg, 2); Serial.print(',');

  Serial.print(accel_cm_filt.x, 2); Serial.print(',');
  Serial.print(accel_cm_filt.y, 2); Serial.print(',');
  Serial.print(accel_cm_filt.z, 2); Serial.print(',');

  Serial.print(AavgX, 2); Serial.print(',');
  Serial.print(AavgY, 2); Serial.print(',');
  Serial.print(AavgZ, 2); Serial.print(',');
  Serial.println(AavgAl, 2);
}

static void printHelp() {
  Serial.println(F("# Commands: IMU START | IMU STOP | IMU RESET VALUE | STATUS | HELP"));
}

static void handleCommand(char *line) {
  while (*line==' '||*line=='\t') line++;
  for (char *p=line; *p; p++) if (*p>='a' && *p<='z') *p = *p - 'a' + 'A';

  if (strcmp(line,"HELP")==0)  { printHelp(); return; }
  if (strcmp(line,"STATUS")==0){ updateIMU(); printOneCsvLine(); return; }

  if (strcmp(line,"IMU START")==0) {
    resetImuValues();
    lastMicrosIMU = micros();
    lastGoodReadMicros = lastMicrosIMU;
    streaming = true;
    Serial.println(F("# IMU START"));
    printHeaderCSV();
    return;
  }
  if (strcmp(line,"IMU STOP")==0) { streaming=false; Serial.println(F("# IMU STOP")); return; }
  if (strcmp(line,"IMU RESET VALUE")==0) { resetImuValues(); Serial.println(F("# IMU RESET VALUE")); return; }

  Serial.println(F("# Unknown. Type HELP"));
}

static void readSerialLines() {
  static char buf[64];
  static uint8_t idx=0;
  while (Serial.available()) {
    char c=(char)Serial.read();
    if (c=='\r' || c=='\n') {
      if (idx>0) { buf[idx]='\0'; handleCommand(buf); idx=0; }
    } else {
      if (idx < sizeof(buf)-1) buf[idx++] = c;
    }
  }
}

// ---------- setup / loop ----------
void setup() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_disable(); // กันรีเซ็ตวนตอนเริ่ม
#endif

  Serial.begin(115200);
  uint32_t t0=millis();
  while (!Serial && (millis()-t0)<1500) {}

  Wire.begin();
  applyI2C();
  delay(100);

  uint8_t found = detectMpuAddr();
  if (!found) {
    Serial.println(F("# MPU not found at 0x68/0x69"));
    while(1) delay(10);
  }
  MPU_ADDR = found;

  Serial.print(F("# MPU addr=0x")); Serial.println(MPU_ADDR, HEX);

  if (!mpuConfigure()) recoverI2CAndMPU();

  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.print(F("# WHO_AM_I=0x")); Serial.println(who, HEX);

  mpuEnableBypass();
  hasMag = ak8963Init();
  Serial.print(F("# MAG=")); Serial.println(hasMag ? F("OK") : F("NO"));

  calibrateGyroBiasBootOnly();

  lastMicrosIMU = micros();
  lastGoodReadMicros = lastMicrosIMU;

  printHelp();
  printHeaderCSV();

#if defined(ARDUINO_ARCH_AVR)
  wdt_enable(WDTO_2S); // ถ้าค้าง >2s จะรีเซ็ตเอง
#endif
}

void loop() {
#if defined(ARDUINO_ARCH_AVR)
  wdt_reset(); // ต้องเรียกทุก loop กัน watchdog รีเซ็ต
#endif

  readSerialLines();

  updateIMU();
  updateMag();

  if (streaming) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastPrintMs) >= PRINT_EVERY_MS) {
      lastPrintMs = nowMs;
      printOneCsvLine();
    }
  }
}
