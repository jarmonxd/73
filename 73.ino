#include <ATX2.h>            //ห้ามลบไม่งั้นใช้ไม่ได้
void (*resetFunc)(void) = 0; // ประกาศฟังก์ชัน reset

float pvYaw = 0;           // ค่า Yaw ดิบจากเซนเซอร์ (-180 ถึง +180)
float pvYawContinuous = 0; // ค่า Yaw แบบต่อเนื่อง (สามารถเกิน ±180 ได้)
float lastRawYaw = 0;      // ค่า Yaw ดิบครั้งก่อน (สำหรับตรวจจับ wrap-around)
float yawOffset = 0;       // ค่าชดเชย wrap-around สะสม
uint8_t rxCnt = 0, rxBuf[8];

// --- ส่วนตั้งค่าเซนเซอร์ (ตาของหุ่นยนต์) ---
int s0, s1, s2, s3; // ตัวแปรสำหรับเก็บค่าที่เซนเซอร์อ่านได้
// ค่าอ้างอิงสำหรับแยกแยะสีพื้นกับเส้น (ถ้าน้อยกว่าค่านี้คือเจอเส้นสีดำ)
int ref0 = 300, ref1 = 300, ref2 = 300, ref3 = 300;

// Function Prototypes
void showSensorValues(bool enabled);
void FdTime(int sl, int sr, int time, int sp, bool checkSen0 = true,
            bool checkSen3 = true);
void FdTimeG(int sl, int sr, int time, int sp, bool checkSen0 = true,
             bool checkSen3 = true);
void servoDrop(int openAngle, int delayTime, int closeAngle);
void tl(int sl, int sr, int time);
void tr(int sl, int sr, int time);
void FdUntilLine(int sl, int sr, int sp);
void zeroYaw();
bool getIMU();
void resetContinuousYaw();
void Auto_zero();
int gyroCorrection();
void FdUntilLineG(int sl, int sr, int sp);
void tlg(int sl, int sr, float targetAngle, float slowPct, int slowSpeed,
         int correctSpeed);
void tlg(int sl, int sr, float targetAngle);
void trg(int sl, int sr, float targetAngle, float slowPct, int slowSpeed,
         int correctSpeed);
void trg(int sl, int sr, float targetAngle);

void getSensor() {
  s0 = analog(0); // ตาซ้ายสุด (นอก)
  s1 = analog(1); // ตาซ้าย (ใน - ไว้เช็คเส้น)
  s2 = analog(2); // ตาขวา (ใน - ไว้เช็คเส้น)
  s3 = analog(3); // ตาขวาสุด (นอก)
}

void setup() {
  XIO();       // คำสั่งเริ่มต้นระบบของ ATX2
  glcdClear(); // รีเซ็ตหน้าจอ
  Serial.begin(9600);
  Serial1.begin(115200);   // เริ่มต้น Serial1 สำหรับคุยกับ IMU
  glcd(0, 0, "Ready Go!"); // ขึ้นหน้าจอว่าพร้อมใช้งาน
  servo(2, 55);            // รีเซ้ตค่าที่ปัดลูกบาศเป็นค่าเริ่มต้น
  // showSensorValues(true); // วัดค่าแสง
  OK();
  beep();     // ส่งเสียงบี๊บยืนยันการกด
  delay(500); // รอก่อนวิ่ง
  zeroYaw();
  resetContinuousYaw(); // รีเซ็ต pvYawContinuous เป็น 0

  // --- ใช้งานโค้ดส่วนด้านล่างนี้ ---

  FdTime(100, 100, 100, 0);
  FdUntilLineG(30, 30, 1);
  FdTimeG(-30, -30, 300, 1);
  delay(100);
  trg(50, 50, 90, 0.70, 30, 30); // ความเร็วซ้าย, ความเร็วขวา, องศาที่ต้องการไป,
                                 // ค่าเปอร์เซ็นต์, ความเร็วชะลอ,ความเร็วแก้ไขเลี้ยวเกิน
  FdTime(70, 70, 100, 0);
  FdUntilLineG(30, 30, 1);
  FdTimeG(-30, -30, 500, 1, false, false);
  servoDrop(160, 500, 55);
  FdTimeG(-30, -30, 800, 1, false, false);
  trg(50, 50, 90, 0.70, 30, 30); // ความเร็วซ้าย, ความเร็วขวา, องศาที่ต้องการไป,
                                 // ค่าเปอร์เซ็นต์, ความเร็วชะลอ,ความเร็วแก้ไขเลี้ยวเกิน
  FdTime(70, 70, 100, 0);
  FdUntilLineG(30, 30, 1);
  delay(50);
  FdTimeG(-30, -30, 300, 1, false, false);
  delay(100);
  tlg(70, 70, 90, 0.75, 30, 30); // ความเร็วซ้าย, ความเร็วขวา, องศาที่ต้องการไป,
                                 // ค่าเปอร์เซ็นต์, ความเร็วชะลอ,ความเร็วแก้ไขเลี้ยวเกิน
  FdTimeG(50, 50, 600, 1);
  trg(50, 50, 90, 0.70, 30, 30); // ความเร็วซ้าย, ความเร็วขวา, องศาที่ต้องการไป,
                                 // ค่าเปอร์เซ็นต์, ความเร็วชะลอ,ความเร็วแก้ไขเลี้ยวเกิน
  FdTime(100, 100, 100, 0);
  FdUntilLineG(30, 30, 1);
  FdTimeG(-30, -30, 300, 1, false, false);
  trg(50, 50, 90, 0.70, 30, 30); // ความเร็วซ้าย, ความเร็วขวา, องศาที่ต้องการไป,
  FdUntilLineG(30, 30, 1);

  // FdTime(28, 33, 1000, 0);
  // FdUntilLine(28, 33, 1);
  // delay(200);
  // FdTime(-20, -20, 400, 0);
  // delay(200);
  // tl(50, 50, 550);
  // FdTime(50, 55, 500, 0);
  // FdUntilLine(28, 33, 1);

  //   servoDrop(160, 500, 55);

  /*
  ========================================================================
  [ คู่มือการเขียนคำสั่งควบคุมหุ่นยนต์ ]
  ========================================================================

  คำสั่งพื้นฐาน (ก๊อปปี้ไปวางใช้งานได้เลย):

  1. การเดินหน้า (Fd = Forward)
     - FdTime(50, 50, 1000, 1);
       -> เดินหน้า (ซ้าย 50, ขวา 50) นาน 1 วินาที แล้วหยุดเบรก
     - FdTimeG(50, 50, 1000, 1);
       -> เดินหน้า "แบบตัวตรง" โดยใช้ไจโรช่วย (เวลา 1 วินาที)
     - FdUntilLine(30, 30, 1);
       -> เดินหน้าไปเรื่อยๆ จนกว่า "ตาหน้าจะเจอเส้นดำ" แล้วหยุด
     - FdUntilLineG(30, 30, 1);
       -> เดินหน้าไปหาเส้น "แบบใช้ไจโรประคองตัวให้ตรงตลอดทาง"

  2. การเลี้ยว (tl = Turn Left, tr = Turn Right)
     - tlg(50, 50, 90);
       -> เลี้ยวซ้าย 90 องศา (ใช้ไจโรวัดมุม แม่นยำสูง)
     - trg(50, 50, 90);
       -> เลี้ยวขวา 90 องศา (ใช้ไจโรวัดมุม แม่นยำสูง)
     - tl(50, 50, 500); / tr(50, 50, 500);
       -> เลี้ยวตามเวลา 0.5 วินาที (ไม่ใช้ไจโร)

  3. คำสั่งอื่นๆ
     - servoDrop(160, 500, 55);
       -> สั่งปัดลูกบาศก์ (กางไป 160 องศา, รอ 0.5 วินาที, กลับมา 55 องศา)
     - delay(500);
       -> สั่งให้หุ่นยนต์ "หยุดรอ" 0.5 วินาที
     - showSensorValues(true);
       -> ใช้ "เช็คค่าแสง" ที่หน้าจอ (ดูค่าพื้นขาว/เส้นดำ)

  *หมายเหตุ:
   - "หยุดไหม" -> ใส่ 1 คือหยุดเบรกทันที, ใส่ 0 คือปล่อยให้ไหล
   - "ซ้าย, ขวา" -> คือความเร็วมอเตอร์ (แนะนำที่ 30 ถึง 80)
  ========================================================================
  */
}

void loop() {
  resetFunc(); // สั่งรีเซ็ตบอร์ดทันที
}

// --- ฟังก์ชันเดินหน้าตามเวลา ---
// sl = ความเร็วล้อซ้าย, sr = ความเร็วล้อขวา, time = เวลา(ms), sp = 1:หยุด 0:ไม่หยุด
void FdTime(int sl, int sr, int time, int sp, bool checkSen0, bool checkSen3) {
  unsigned long startTime = millis(); // เริ่มจับเวลา
  while (millis() - startTime < (unsigned long)time) {
    // อ่านค่าเซนเซอร์
    getSensor();

    if (checkSen3 && s3 < ref3) {
      // ถ้า a3 เจอ เลี่ยงไปด้านขวา
      motor(1, -30);
      motor(2, -30);
      motor(3, 30);
      motor(4, 30);
      delay(150);
    } else if (checkSen0 && s0 < ref0) {
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

// --- ฟังก์ชันปัดแขนเซอร์โว ---
// openAngle = องศาตอนปัดออก, delayTime = เวลารอ, closeAngle = องศาตอนหุบกลับ
void servoDrop(int openAngle, int delayTime, int closeAngle) {
  servo(2, openAngle); // สั่งเซอร์โวช่อง 2
  delay(delayTime);
  servo(2, closeAngle);
}

// --- ฟังก์ชันเลี้ยวซ้าย (ตามเวลา) ---
void tl(int sl, int sr, int time) {
  motor(1, sr); // มอเตอร์ขวาเดินหน้า
  motor(2, sr);
  motor(3, -sl);
  motor(4, -sl);
  delay(time);
  motor_stop(ALL);
}

// --- ฟังก์ชันเลี้ยวขวา (ตามเวลา) ---
void tr(int sl, int sr, int time) {
  motor(1, -sr); // มอเตอร์ขวาถอยหลัง
  motor(2, -sr);
  motor(3, sl);
  motor(4, sl);
  delay(time);
  motor_stop(ALL);
}

// --- ฟังก์ชันเดินหน้าจนกว่าจะเจอเส้น (เซนเซอร์ s1, s2) ---
void FdUntilLine(int sl, int sr, int sp) {
  bool alignmentMode = false; // โหมดปรับตัวให้ตรงเส้น

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

// ฟังก์ชันอ่านค่าจาก IMU ผ่าน Serial1 (Continuous Yaw)
// ปรับปรุง: อ่านค่าทั้งหมดที่มีใน Buffer เพื่อให้ได้ค่าล่าสุดเสมอ (ลด Latency)
bool getIMU() {
  bool updated = false;
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) {
      // ถ้า byte แรกไม่ใช่ header ให้ข้ามไป
      continue;
    }
    rxCnt++;
    if (rxCnt == 8) { // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) { // data package is correct
        // แปลงข้อมูลเป็น int16_t (ค่าดิบ -180 ถึง +180)
        float rawYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        pvYaw = rawYaw; // เก็บค่าดิบไว้ด้วย

        // ตรวจจับ wrap-around และชดเชย
        float delta = rawYaw - lastRawYaw;

        // ถ้าค่าเปลี่ยนมากกว่า 300 องศา = เกิด wrap-around
        if (delta > 300) {
          // wrap จาก + ไป - (เช่น 179 -> -179)
          yawOffset -= 360;
        } else if (delta < -300) {
          // wrap จาก - ไป + (เช่น -179 -> 179)
          yawOffset += 360;
        }

        lastRawYaw = rawYaw;
        pvYawContinuous = rawYaw + yawOffset;

        updated = true; // ระบุว่ามีการอัพเดทค่าใหม่
      }
    }
  }
  return updated; // คืนค่า true ถ้ามีการอัพเดทค่าอย่างน้อย 1 ครั้ง
}

// รีเซ็ต Continuous Yaw เป็น 0
void resetContinuousYaw() {
  // อ่านค่าล่าสุดทิ้งก่อน เพื่อให้แน่ใจว่า lastRawYaw เป็นปัจจุบัน
  getIMU();
  lastRawYaw = pvYaw;
  yawOffset = -pvYaw; // ทำให้ค่าปัจจุบันกลายเป็น 0
  pvYawContinuous = 0;
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
  unsigned long lastDisp = 0;
  while (abs(pvYaw) > 0.02) {
    zeroYaw();
    if (getIMU()) {
      // แสดงผลแค่ทุกๆ 150ms เพื่อไม่ให้หน่วงเกินไป
      if (millis() - lastDisp > 150) {
        lastDisp = millis();
        // Arduino ทั่วไป %f อาจไม่ทำงาน ใช้การแปลงเป็น int หรือ String
        // glcd(3, 2, "Yaw: %f  ", pvYaw); -> แก้เป็น:
        glcd(3, 2, "Yaw: %d.%02d  ", (int)pvYaw, abs((int)(pvYaw * 100) % 100));
      }
    }
  }

  glcdClear();
  beep(); // ส่งเสียงเตือนเมื่อเซตเสร็จ
}

// ฟังก์ชันแสดงค่าเซนเซอร์ 0-3 บนจอ GLCD
void showSensorValues(bool enabled) {
  if (!enabled)
    return;

  glcdClear();
  setTextSize(1);
  while (!sw_OK()) { // วนลูปจนกว่าจะกดปุ่ม OK
    getSensor();
    glcd(0, 0, "S0: %d   ", s0);
    glcd(1, 0, "S1: %d   ", s1);
    glcd(2, 0, "S2: %d   ", s2);
    glcd(3, 0, "S3: %d   ", s3);
    glcd(5, 0, "Press OK to Exit");
    delay(100);
  }
  glcdClear();
}

// ================= GYROSCOPE CONTROL FUNCTIONS =================
// ค่าคงที่สำหรับการปรับค่าความแรงการชดเชย
// หมายเหตุ: Kp ตัวนี้ใช้เฉพาะในฟังก์ชันเดินหน้า (FdTimeG, FdUntilLineG) เท่านั้น
// ไม่ได้ถูกใช้ใน tlg หรือ trg (ซึ่งใช้ logic แบบ step speed แทน)
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
// --- ฟังก์ชันเดินหน้าตามเวลา แบบใช้ไจโรช่วย (วิ่งตัวตรง) ---
void FdTimeG(int sl, int sr, int time, int sp, bool checkSen0, bool checkSen3) {
  zeroYaw(); // เซ็ตมุมปัจจุบันเป็น 0 เพื่อเริ่มวิ่งตรง
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

    if (checkSen3 && s3 < ref3) {
      // ถ้า a3 เจอ เลี่ยงไปด้านขวา
      motor(1, -30);
      motor(2, -30);
      motor(3, 30);
      motor(4, 30);
      delay(150);
      zeroYaw(); // รีเซ็ตไจโรหลังเลี้ยว
    } else if (checkSen0 && s0 < ref0) {
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
// --- ฟังก์ชันเดินหน้าจนเจอเส้น แบบใช้ไจโรช่วย (วิ่งตัวตรงไปหาเส้น) ---
void FdUntilLineG(int sl, int sr, int sp) {
  zeroYaw(); // เซ็ตมุมปัจจุบันเป็น 0
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

// tlg: หมุนซ้ายโดยใช้ไจโรตรวจสอบมุม (ใช้ Continuous Yaw - รองรับมุมเกิน 180°)
// เลี้ยวซ้าย = pvYawContinuous จะเพิ่มขึ้นเป็นบวก
// targetAngle = มุมเป้าหมาย (ค่าบวก)
// slowPct = เปอร์เซ็นต์ที่จะเริ่มชะลอ (เช่น 0.7 = 70% ของมุมเป้าหมาย)
// slowSpeed = ความเร็วเมื่อชะลอ
// correctSpeed = ความเร็วเมื่อแก้ไขเลี้ยวเกิน
// --- ฟังก์ชันเลี้ยวซ้าย แบบระบุองศา (ใช้ไจโร) ---
// sl, sr = ความเร็ว, targetAngle = องศาที่อยากให้เลี้ยวไป
void tlg(int sl, int sr, float targetAngle, float slowPct, int slowSpeed,
         int correctSpeed) {

  glcd(0, 0, "Waiting IMU...");

  // รอจนกว่าจะได้ค่าจาก IMU จริงๆ (รอ return true)
  unsigned long waitStart = millis();
  while (!getIMU()) {
    if (millis() - waitStart > 1000) { // timeout 1 วินาที
      glcd(0, 0, "IMU Timeout!  ");
      delay(500);
      break;
    }
    delay(10);
  }
  delay(100);

  // รีเซ็ต Continuous Yaw เป็น 0
  resetContinuousYaw();

  // รออ่านค่าใหม่หลัง reset
  waitStart = millis();
  while (!getIMU()) {
    if (millis() - waitStart > 500)
      break;
    delay(10);
  }

  // แสดงผลเริ่มต้น (ใช้ %d.%d แทน %f)
  glcd(0, 0, "Start: %d.%d   ", (int)pvYawContinuous,
       abs((int)(pvYawContinuous * 10) % 10));
  delay(200);

  float absTarget = abs(targetAngle);
  float slowThreshold = absTarget * slowPct; // มุมที่เริ่มชะลอ

  unsigned long startTime = millis();
  // unsigned long lastDispTime = 0; // ตัวแปรสำหรับจับเวลาแสดงผล (ปิดเพื่อลด Latency)
  unsigned long timeout =
      (unsigned long)(absTarget * 100); // timeout ตามมุม (100ms ต่อ 1 องศา)
  if (timeout < 3000)
    timeout = 3000; // minimum 3 วินาที

  while (1) {
    // พยายามอ่านค่า IMU ให้บ่อยที่สุด (สำคัญมากสำหรับการ integrate ค่า)
    getIMU();

    // แสดงค่าบนจอ (update แค่ทุกๆ 150ms พอ ไม่ให้หน่วง loop)
    // *** COMMENT OUT เพื่อความเร็วสูงสุด ***
    // if (millis() - lastDispTime > 150) {
    //   lastDispTime = millis();
    //   // Debug: แสดงค่า Yaw และ Target
    //   glcd(0, 0, "Yaw:%d.%d T:%d", (int)pvYawContinuous,
    //        abs((int)(pvYawContinuous * 10) % 10), (int)absTarget);
    // }

    // ถ้าถึงเป้าหมายแล้ว ออกจาก loop
    if (pvYawContinuous >= absTarget) {
      motor_stop(ALL); // สั่งหยุดทันทีก่อนแสดงผล
      glcd(1, 0, "REACHED!      ");
      break;
    }

    // timeout ป้องกันหมุนไม่หยุด
    if (millis() - startTime > timeout) {
      motor_stop(ALL);
      glcd(1, 0, "TIMEOUT!      ");
      break;
    }

    // คำนวณความเร็วตามระยะห่างจากเป้าหมาย
    int currentSl, currentSr;
    if (pvYawContinuous >= slowThreshold) {
      // ใกล้เป้าหมายแล้ว ชะลอความเร็ว
      currentSl = slowSpeed;
      currentSr = slowSpeed;
    } else {
      // ยังไกล ใช้ความเร็วปกติ
      currentSl = sl;
      currentSr = sr;
    }

    motor(1, currentSr);
    motor(2, currentSr);
    motor(3, -currentSl);
    motor(4, -currentSl);
  }
  motor_stop(ALL); // ย้ำหยุดอีกครั้ง

  // แก้ไขการเลี้ยวเกิน (overshoot correction)
  delay(100);
  startTime = millis();
  while (millis() - startTime < 2000) { // แก้ไขไม่เกิน 2 วินาที
    if (getIMU()) {
      if (pvYawContinuous <= absTarget + 2) {
        break; // อยู่ในช่วงที่ยอมรับได้
      }

      // แสดงผลตอนแก้ (ไม่ถี่มาก)
      // *** COMMENT OUT เพื่อความเร็วสูงสุด ***
      // if (millis() - lastDispTime > 150) {
      //    lastDispTime = millis();
      //    glcd(0, 0, "FIX: %d.%d     ", (int)pvYawContinuous,
      //    abs((int)(pvYawContinuous*10)%10));
      // }

      // หมุนกลับ (ไปขวา) ช้าๆ
      motor(1, -correctSpeed);
      motor(2, -correctSpeed);
      motor(3, correctSpeed);
      motor(4, correctSpeed);
    }
  }
  motor_stop(ALL);

  // แสดงผลค่าสุดท้าย
  glcd(0, 0, "Done: %d.%d    ", (int)pvYawContinuous,
       abs((int)(pvYawContinuous * 10) % 10));
}

// tlg แบบ default parameters (สำหรับเรียกใช้แบบง่าย)
void tlg(int sl, int sr, float targetAngle) {
  tlg(sl, sr, targetAngle, 0.7, 20,
      15); // slowPct=70%, slowSpeed=20, correctSpeed=15
}

// trg: หมุนขวาโดยใช้ไจโรตรวจสอบมุม (ใช้ Continuous Yaw - รองรับมุมเกิน 180°)
// เลี้ยวขวา = pvYawContinuous จะลดลงเป็นลบ
// targetAngle = มุมเป้าหมาย (ค่าบวก)
// slowPct = เปอร์เซ็นต์ที่จะเริ่มชะลอ (เช่น 0.7 = 70% ของมุมเป้าหมาย)
// slowSpeed = ความเร็วเมื่อชะลอ
// correctSpeed = ความเร็วเมื่อแก้ไขเลี้ยวเกิน
// --- ฟังก์ชันเลี้ยวขวา แบบระบุองศา (ใช้ไจโร) ---
void trg(int sl, int sr, float targetAngle, float slowPct, int slowSpeed,
         int correctSpeed) {

  glcd(0, 0, "Waiting IMU...");

  // รอจนกว่าจะได้ค่าจาก IMU จริงๆ (รอ return true)
  unsigned long waitStart = millis();
  while (!getIMU()) {
    if (millis() - waitStart > 1000) { // timeout 1 วินาที
      glcd(0, 0, "IMU Timeout!  ");
      delay(500);
      break;
    }
    delay(10);
  }
  delay(100);

  // รีเซ็ต Continuous Yaw เป็น 0
  resetContinuousYaw();

  // รออ่านค่าใหม่หลัง reset
  waitStart = millis();
  while (!getIMU()) {
    if (millis() - waitStart > 500)
      break;
    delay(10);
  }

  // ใช้ %d.%d แทน %.1f
  glcd(0, 0, "Start: %d.%d   ", (int)pvYawContinuous,
       abs((int)(pvYawContinuous * 10) % 10));
  delay(200);

  float absTarget = abs(targetAngle);
  float slowThreshold = absTarget * slowPct; // มุมที่เริ่มชะลอ

  unsigned long startTime = millis();
  // unsigned long lastDispTime = 0;
  unsigned long timeout =
      (unsigned long)(absTarget * 100); // timeout ตามมุม (100ms ต่อ 1 องศา)
  if (timeout < 3000)
    timeout = 3000; // minimum 3 วินาที

  while (1) {
    // อ่านค่า IMU บ่อยๆ
    getIMU();

    // แสดงค่าแบบ throttle
    // *** COMMENT OUT เพื่อความเร็วสูงสุด ***
    // if (millis() - lastDispTime > 150) {
    //   lastDispTime = millis();
    //   glcd(0, 0, "CYaw=%d.%d T=%d", (int)pvYawContinuous,
    //   abs((int)(pvYawContinuous*10)%10), -(int)absTarget);
    // }

    // ถ้าถึงเป้าหมายแล้ว ออกจาก loop (ค่าลบ)
    if (pvYawContinuous <= -absTarget) {
      motor_stop(ALL);
      glcd(1, 0, "REACHED!      ");
      break;
    }

    // timeout ป้องกันหมุนไม่หยุด
    if (millis() - startTime > timeout) {
      motor_stop(ALL);
      glcd(1, 0, "TIMEOUT!      ");
      break;
    }

    // คำนวณความเร็วตามระยะห่างจากเป้าหมาย
    int currentSl, currentSr;
    if (abs(pvYawContinuous) >= slowThreshold) {
      // ใกล้เป้าหมายแล้ว ชะลอความเร็ว
      currentSl = slowSpeed;
      currentSr = slowSpeed;
    } else {
      // ยังไกล ใช้ความเร็วปกติ
      currentSl = sl;
      currentSr = sr;
    }

    motor(1, -currentSr);
    motor(2, -currentSr);
    motor(3, currentSl);
    motor(4, currentSl);
  }
  motor_stop(ALL);

  // แก้ไขการเลี้ยวเกิน (overshoot correction)
  delay(100);
  startTime = millis();
  while (millis() - startTime < 2000) { // แก้ไขไม่เกิน 2 วินาที
    if (getIMU()) {
      if (pvYawContinuous >= -absTarget - 2) {
        break; // อยู่ในช่วงที่ยอมรับได้
      }

      // *** COMMENT OUT เพื่อความเร็วสูงสุด ***
      // if (millis() - lastDispTime > 150) {
      //   lastDispTime = millis();
      //   glcd(0, 0, "FIX: %d.%d     ", (int)pvYawContinuous,
      //   abs((int)(pvYawContinuous*10)%10));
      // }

      // หมุนกลับ (ไปซ้าย) ช้าๆ
      motor(1, correctSpeed);
      motor(2, correctSpeed);
      motor(3, -correctSpeed);
      motor(4, -correctSpeed);
    }
  }
  motor_stop(ALL);
  glcd(0, 0, "Done: %d.%d    ", (int)pvYawContinuous,
       abs((int)(pvYawContinuous * 10) % 10));
}

// trg แบบ default parameters (สำหรับเรียกใช้แบบง่าย)
void trg(int sl, int sr, float targetAngle) {
  trg(sl, sr, targetAngle, 0.7, 20,
      15); // slowPct=70%, slowSpeed=20, correctSpeed=15
}