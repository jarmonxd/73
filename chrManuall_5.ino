#include <ATX2.h>
void (*resetFunc)(void) = 0; // ประกาศฟังก์ชัน reset

float pvYaw = 80;
uint8_t rxCnt = 0, rxBuf[8];

int s0, s1, s2, s3;
int ref0 = 300, ref1 = 300, ref2 = 300, ref3 = 300;

void getSensor() {
  s0 = analog(0);
  s1 = analog(1);
  s2 = analog(2);
  s3 = analog(3);
}

void setup() {
  XIO(); // คำสั่งเริ่มต้นระบบของ ATX2 (จำเป็นต้องมี)
  glcdClear();
  Serial.begin(9600);
  Serial1.begin(115200); // เริ่มต้น Serial1 สำหรับคุยกับ IMU
  glcd(0, 0, "Ready Go!");
  servo(2, 55);
  OK();
  beep();     // ส่งเสียงบี๊บยืนยันการกด
  delay(500); // พักแป๊บนึง

  zeroYaw();
  // --- ตัวอย่างการเรียกใช้ ---

  // ช่วงแรก: วิ่งช้าๆ หน่อย (ความเร็ว 30) เพื่อความชัวร์
  // FdTime(30, 30, 2000, 1);
  FdUntilLineG(30, 30, 1);
  FdTimeG(-30, -30, 200, 0);
  trg(50, 50, -90);
  // FdTime(28, 33, 1000, 0);
  // FdUntilLine(28, 33, 1);
  // delay(200);
  // FdTime(-20, -20, 400, 0);
  // delay(200);
  // tl(50, 50, 550);
  // FdTime(50, 55, 500, 0);
  // FdUntilLine(28, 33, 1);

  //   servoDrop(160, 500, 55);
}

void loop() {
  resetFunc(); // สั่งรีเซ็ตบอร์ดทันที
  // ไม่ได้ทำอะไร
}

void FdTime(int sl, int sr, int time, int sp) {
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)time) {
    // อ่านค่าเซนเซอร์
    getSensor();

    if (s3 < ref3) {
      // ถ้า a3 เจอ เลี่ยงไปด้านขวา
      motor(1, -30);
      motor(2, -30);
      motor(3, 30);
      motor(4, 30);
      delay(150);
    } else if (s0 < ref0) {
      // ถ้า a0 เจอ เลี่ยงไปด้านซ้าย
      motor(1, 30);
      motor(2, 30);
      motor(3, -30);
      motor(4, -30);
      delay(150);
    } else {
      // วิ่งตรงตามปกติ
      motor(1, sr);
      motor(2, sr);
      motor(3, sl);
      motor(4, sl);
    }
  }

  if (sp == 1) {
    motor_stop(ALL);
  }
}

void servoDrop(int openAngle, int delayTime, int closeAngle) {
  servo(2, openAngle);
  delay(delayTime);
  servo(2, closeAngle);
}

void tl(int sl, int sr, int time) {
  motor(1, sr);
  motor(2, sr);
  motor(3, -sl);
  motor(4, -sl);
  delay(time);
  motor_stop(ALL);
}

void tr(int sl, int sr, int time) {
  motor(1, -sr);
  motor(2, -sr);
  motor(3, sl);
  motor(4, sl);
  delay(time);
  motor_stop(ALL);
}

void FdUntilLine(int sl, int sr, int sp) {
  bool alignmentMode = false;

  while (1) {
    getSensor();

    // ถ้าเจอเส้นทั้ง 2 ตัว ให้หยุด (ออกจาก loop)
    if (s1 < ref1 && s2 < ref2) {
      break;
    }

    // ตรวจสอบว่าครุ่นเช็คเจอเส้นบ้างหรือยัง
    if (s1 < ref1 || s2 < ref2) {
      alignmentMode = true;
    }

    if (alignmentMode) {
      // เมื่อเข้าโหมดปรับจูนแล้ว ห้ามเดินหน้าตรงเด็ดขาด
      if (s1 < ref1) {
        // เจอแค่ซ้าย ให้หมุนซ้าย
        motor(1, -30);
        motor(2, -30);
        motor(3, 30);
        motor(4, 30);
      } else if (s2 < ref2) {
        // เจอแค่ขวา ให้หมุนขวา
        motor(1, 30);
        motor(2, 30);
        motor(3, -30);
        motor(4, -30);
      } else {
        // กรณีหลุดเส้นในขณะปรับจูน ให้หยุด (กันพุ่ง)
        motor_stop(ALL);
      }
    } else {
      // ยังไม่เจอเส้นหลัก วิ่งตามไลน์ปกติ
      if (s3 < ref3) {
        // เลี่ยงขวา
        motor(1, -30);
        motor(2, -30);
        motor(3, 30);
        motor(4, 30);
        delay(50);
      } else if (s0 < ref0) {
        // เลี่ยงซ้าย
        motor(1, 30);
        motor(2, 30);
        motor(3, -30);
        motor(4, -30);
        delay(50);
      } else {
        // วิ่งตรง
        motor(1, sr);
        motor(2, sr);
        motor(3, sl);
        motor(4, sl);
      }
    }
  }

  if (sp == 1) {
    motor_stop(ALL);
  }
}
void zeroYaw() {
  Serial1.write(0XA5);
  Serial1.write(0X54);
  delay(60);
  // pitch correction roll angle
  Serial1.write(0XA5);
  Serial1.write(0X55);
  delay(60);
  // zero degree heading
  Serial1.write(0XA5);
  Serial1.write(0X52);
  delay(60);
  // automatic mode
}

// ฟังก์ชันอ่านค่าจาก IMU ผ่าน Serial1
bool getIMU() {
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA)
      return false;
    rxCnt++;
    if (rxCnt == 8) { // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) { // data package is correct
        // แปลงข้อมูลเป็น int16_t ทำให้รองรับค่าติดลบได้ (เช่น -179.99 ถึง 180.00)
        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        return true;
      }
    }
  }
  return false;
}

// ฟังก์ชันเซต 0 แบบอัตโนมัติ พร้อมแสดงผลบนจอสี (GLCD)
void Auto_zero() {
  zeroYaw();
  getIMU();

  glcdClear();    // ล้างหน้าจอ (แทน oled.clear)
  setTextSize(1); // ตั้งขนาดตัวอักษร
  glcd(1, 2, "Setting zero");

  // วนลูปจนกว่าค่าจะเข้าใกล้ 0 (เช็คทั้งบวกและลบด้วย abs)
  // ถ้า pvYaw เริ่มต้นเป็น 0 และอ่านค่าไม่ทัน ลูปนี้จะไม่ทำงานเลย
  while (abs(pvYaw) > 0.02) {
    zeroYaw();
    if (getIMU()) {
      glcd(3, 2, "Yaw: %f  ", pvYaw);
    }
    delay(50); // หน่วงเวลาเล็กน้อยเพื่อให้จอภาพไม่กระพริบเร็วเกินไป
  }

  glcdClear();
  beep(); // ส่งเสียงเตือนเมื่อเซตเสร็จ
}

// ================= GYROSCOPE CONTROL FUNCTIONS =================
// ค่าคงที่สำหรับการปรับค่าความแรงการชดเชย (สามารถปรับได้ตามต้องการ)
float Kp = 2.0; // Proportional gain - ยิ่งสูง ยิ่งตอบสนองเร็วแต่อาจเหว่ง

// ฟังก์ชันคำนวณค่าชดเชยจากไจโร
int gyroCorrection() {
  getIMU(); // อ่านค่าไจโรล่าสุด
  // pvYaw บวก = เอียงขวา -> ต้องชดเชยให้ไปซ้าย (ลดมอเตอร์ขวา)
  // pvYaw ลบ = เอียงซ้าย -> ต้องชดเชยให้ไปขวา (ลดมอเตอร์ซ้าย)
  int correction = (int)(Kp * pvYaw);
  // จำกัดค่าไม่ให้เกิน ±30 เพื่อป้องกันการเหว่ง
  if (correction > 30)
    correction = 30;
  if (correction < -30)
    correction = -30;
  return correction;
}

// FdTimeG: เดินหน้าตามเวลาโดยใช้ไจโรชดเชย
void FdTimeG(int sl, int sr, int time, int sp) {
  zeroYaw(); // รีเซ็ตค่าไจโรก่อนเริ่ม
  delay(50);

  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)time) {
    getSensor();
    int correction = gyroCorrection();

    // คำนวณความเร็วมอเตอร์พร้อมชดเชย
    // correction บวก (เอียงขวา) -> ลด sr, เพิ่ม sl เพื่อหักซ้าย
    // correction ลบ (เอียงซ้าย) -> เพิ่ม sr, ลด sl เพื่อหักขวา
    int adjSl = sl + correction;
    int adjSr = sr - correction;

    if (s3 < ref3) {
      // ถ้า a3 เจอ เลี่ยงไปด้านขวา
      motor(1, -30);
      motor(2, -30);
      motor(3, 30);
      motor(4, 30);
      delay(150);
      zeroYaw(); // รีเซ็ตไจโรหลังเลี้ยว
    } else if (s0 < ref0) {
      // ถ้า a0 เจอ เลี่ยงไปด้านซ้าย
      motor(1, 30);
      motor(2, 30);
      motor(3, -30);
      motor(4, -30);
      delay(150);
      zeroYaw(); // รีเซ็ตไจโรหลังเลี้ยว
    } else {
      // วิ่งตรงพร้อมชดเชยไจโร
      motor(1, adjSr);
      motor(2, adjSr);
      motor(3, adjSl);
      motor(4, adjSl);
    }
  }

  if (sp == 1) {
    motor_stop(ALL);
  }
}

// FdUntilLineG: เดินหน้าจนเจอเส้นโดยใช้ไจโรชดเชย
void FdUntilLineG(int sl, int sr, int sp) {
  zeroYaw(); // รีเซ็ตค่าไจโรก่อนเริ่ม
  delay(50);

  bool alignmentMode = false;

  while (1) {
    getSensor();
    int correction = gyroCorrection();

    // คำนวณความเร็วมอเตอร์พร้อมชดเชย
    int adjSl = sl + correction;
    int adjSr = sr - correction;

    // ถ้าเจอเส้นทั้ง 2 ตัว ให้หยุด
    if (s1 < ref1 && s2 < ref2) {
      break;
    }

    // ตรวจสอบว่าเจอเส้นบ้างหรือยัง
    if (s1 < ref1 || s2 < ref2) {
      alignmentMode = true;
    }

    if (alignmentMode) {
      // เมื่อเข้าโหมดปรับจูนแล้ว ไม่ใช้ไจโร
      if (s1 < ref1) {
        motor(1, -30);
        motor(2, -30);
        motor(3, 30);
        motor(4, 30);
      } else if (s2 < ref2) {
        motor(1, 30);
        motor(2, 30);
        motor(3, -30);
        motor(4, -30);
      } else {
        motor_stop(ALL);
      }
    } else {
      // ยังไม่เจอเส้นหลัก วิ่งตามไลน์ปกติพร้อมไจโรชดเชย
      if (s3 < ref3) {
        motor(1, -30);
        motor(2, -30);
        motor(3, 30);
        motor(4, 30);
        delay(50);
        zeroYaw();
      } else if (s0 < ref0) {
        motor(1, 30);
        motor(2, 30);
        motor(3, -30);
        motor(4, -30);
        delay(50);
        zeroYaw();
      } else {
        motor(1, adjSr);
        motor(2, adjSr);
        motor(3, adjSl);
        motor(4, adjSl);
      }
    }
  }

  if (sp == 1) {
    motor_stop(ALL);
  }
}

// tlg: หมุนซ้ายโดยใช้ไจโรตรวจสอบมุม
// เลี้ยวซ้าย = pvYaw จะเพิ่มขึ้นเป็นบวก
// targetAngle ควรเป็นค่าบวก เช่น tlg(50, 50, 90) = หมุนซ้าย 90 องศา
void tlg(int sl, int sr, float targetAngle) {
  zeroYaw();
  delay(100);

  float absTarget = abs(targetAngle); // ใช้ค่าบวกเสมอ

  while (1) {
    getIMU();

    // ป้องกัน wrap-around: ถ้าค่าเกิน 80 หรือต่ำกว่า -80 ให้หยุด
    if (pvYaw < -100 || pvYaw > 100) {
      // ค่า wrap-around ให้รีเซ็ตและหยุด
      break;
    }

    // เลี้ยวซ้าย = pvYaw เพิ่มขึ้น (บวก)
    if (pvYaw >= absTarget) {
      break;
    }
    motor(1, sr);
    motor(2, sr);
    motor(3, -sl);
    motor(4, -sl);
  }
  motor_stop(ALL);
}

// trg: หมุนขวาโดยใช้ไจโรตรวจสอบมุม
// เลี้ยวขวา = pvYaw จะลดลงเป็นลบ
// targetAngle ควรเป็นค่าบวก เช่น trg(50, 50, 90) = หมุนขวา 90 องศา
void trg(int sl, int sr, float targetAngle) {
  zeroYaw();
  delay(100);

  float absTarget = abs(targetAngle); // ใช้ค่าบวกเสมอ

  while (1) {
    getIMU();

    // ป้องกัน wrap-around: ถ้าค่าเกิน 80 หรือต่ำกว่า -80 ให้หยุด
    if (pvYaw < -100 || pvYaw > 100) {
      // ค่า wrap-around ให้รีเซ็ตและหยุด
      break;
    }

    // เลี้ยวขวา = pvYaw ลดลง (ลบ)
    if (pvYaw <= -absTarget) {
      break;
    }
    motor(1, -sr);
    motor(2, -sr);
    motor(3, sl);
    motor(4, sl);
  }
  motor_stop(ALL);
}