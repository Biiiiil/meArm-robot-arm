/*
  ============================================================
  meArm 机械臂 - 内存优化版
  包含：任务一 + 任务二 + 任务三 + 任务五直线绘制
  ============================================================
  内存优化说明：
  - 录制数据用 byte 代替 int（0-180度一个字节够了）
  - 录制点数减少到 150 个（约7.5秒，够用）
  - 任务二动作数组存到 PROGMEM（Flash），不占 RAM
  ============================================================
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <avr/pgmspace.h>

// ====== 舵机引脚 ======
const byte pinBase = 2;
const byte pinRArm = 3;
const byte pinFArm = 4;
const byte pinClaw = 5;

Servo base;
Servo rArm;
Servo fArm;
Servo claw;

// ====== 摇杆引脚 ======
const byte joyL_X = A0;
const byte joyL_Y = A1;
const byte joyR_X = A2;
const byte joyR_Y = A3;

const int joyDeadZone = 50;
const byte joyStep = 2;

// ====== 软串口（按键板） ======
const byte softRX = 8;
const byte softTX = 9;
SoftwareSerial armSerial(softRX, softTX);

// ====== 夹爪角度 ======
const byte clawOpen = 10;
const byte clawClose = 100;

// ====== 速度 ======
byte DSD = 15;

// ====== 任务五：机械臂参数 ======
const float ARM_L1 = 80.0;
const float ARM_L2 = 80.0;
const float ARM_L3 = 35.0;
const float BASE_H = 60.0;

const float baseOffset = 90.0;
const float rArmOffset = 90.0;
const float fArmOffset = 90.0;

const int baseDir = 1;
const int rArmDir = 1;
const int fArmDir = -1;

// ====== 任务五：绘图参数 ======
const float drawZ = 50.0;
const float penUpOffset = 20.0;
const byte lineSteps = 40;
const byte drawDelay = 25;

float currentX = 50.0;
float currentY = 0.0;
float currentZ = 80.0;
bool isPenUp = false;

// ====== 任务二：动作序列（放 PROGMEM 省 RAM）======
// 每个动作8个阶段：{底座, 大臂, 小臂, 夹爪}
const byte actionA[] PROGMEM = {
   90,100, 80,clawOpen,
   90,130, 50,clawOpen,
   90,130, 50,clawClose,
   90,100, 80,clawClose,
   60,100, 80,clawClose,
   60,130, 50,clawClose,
   60,130, 50,clawOpen,
   90, 90, 90,clawOpen
};

const byte actionB[] PROGMEM = {
  120,100, 80,clawOpen,
  120,130, 50,clawOpen,
  120,130, 50,clawClose,
  120,100, 80,clawClose,
  150,100, 80,clawClose,
  150,130, 50,clawClose,
  150,130, 50,clawOpen,
   90, 90, 90,clawOpen
};

const byte actionC[] PROGMEM = {
   45,100, 80,clawOpen,
   45,130, 50,clawOpen,
   45,130, 50,clawClose,
   45,100, 80,clawClose,
   30,100, 80,clawClose,
   30,130, 50,clawClose,
   30,130, 50,clawOpen,
   90, 90, 90,clawOpen
};

// ====== 任务三：循环搬运 ======
byte currentTarget = 0;

// ====== 任务三：录制（byte 省一半内存）======
#define MAX_RECORD 150       // 150点 × 50ms = 7.5秒
#define RECORD_INTERVAL 50
byte recordData[MAX_RECORD][4];
int recordCount = 0;
bool isRecording = false;
unsigned long lastRecordTime = 0;

// ====== 任务三：播放 ======
bool isPlaying = false;
int playIndex = 0;
unsigned long lastPlayTime = 0;


// ====== 数学角度转舵机角度 ======
float math2servo(float mathAng, float offset, int dir) {
  float deg = mathAng * 180.0 / PI;
  float servoAng = offset + dir * deg;
  if (servoAng < 0) servoAng = 0;
  if (servoAng > 180) servoAng = 180;
  return servoAng;
}

// ====== 逆运动学 ======
bool inverseKinematics(float x, float y, float z, float &baseAng, float &rArmAng, float &fArmAng) {
  float theta1 = atan2(y, x);
  float r = sqrt(x * x + y * y) - ARM_L3;
  float h = z - BASE_H;
  if (r < 0) r = 0;

  float D = sqrt(r * r + h * h);
  if (D > ARM_L1 + ARM_L2) return false;
  if (D < abs(ARM_L1 - ARM_L2)) return false;

  float cosA = (ARM_L1 * ARM_L1 + D * D - ARM_L2 * ARM_L2) / (2 * ARM_L1 * D);
  if (cosA > 1) cosA = 1;
  if (cosA < -1) cosA = -1;
  float alpha = acos(cosA);

  float phi = atan2(h, r);
  float theta2 = phi + alpha;

  float cosB = (ARM_L1 * ARM_L1 + ARM_L2 * ARM_L2 - D * D) / (2 * ARM_L1 * ARM_L2);
  if (cosB > 1) cosB = 1;
  if (cosB < -1) cosB = -1;
  float beta = acos(cosB);

  float thetaF = theta2 - (PI - beta);

  baseAng = math2servo(theta1, baseOffset, baseDir);
  rArmAng = math2servo(theta2, rArmOffset, rArmDir);
  fArmAng = math2servo(thetaF, fArmOffset, fArmDir);

  if (baseAng < 0 || baseAng > 180) return false;
  if (rArmAng < 0 || rArmAng > 180) return false;
  if (fArmAng < 0 || fArmAng > 180) return false;

  return true;
}

// ====== setup ======
void setup() {
  base.attach(pinBase);
  rArm.attach(pinRArm);
  fArm.attach(pinFArm);
  claw.attach(pinClaw);

  base.write(90);
  rArm.write(90);
  fArm.write(90);
  claw.write(clawOpen);

  Serial.begin(9600);
  Serial.println("meArm - Memory Optimized");
  Serial.println("Task1+2+3+5 Line");

  armSerial.begin(9600);
  Serial.println("Cmd: b/r/f/c+num, xyz, ABC, L x1y1x2y2, U/D, o, h/l");
}

// ====== 单舵机移动 ======
void servoCmd(byte type, byte target, byte dly) {
  int fromPos;
  switch (type) {
    case 'b': fromPos = base.read(); break;
    case 'r': fromPos = rArm.read(); break;
    case 'f': fromPos = fArm.read(); break;
    case 'c': fromPos = claw.read(); break;
    default: return;
  }
  if (target > 180) target = 180;

  if (fromPos < target) {
    for (int i = fromPos; i <= target; i++) {
      switch (type) {
        case 'b': base.write(i); break;
        case 'r': rArm.write(i); break;
        case 'f': fArm.write(i); break;
        case 'c': claw.write(i); break;
      }
      delay(dly);
    }
  } else {
    for (int i = fromPos; i >= target; i--) {
      switch (type) {
        case 'b': base.write(i); break;
        case 'r': rArm.write(i); break;
        case 'f': fArm.write(i); break;
        case 'c': claw.write(i); break;
      }
      delay(dly);
    }
  }
}

// ====== 多舵机协同 ======
void servoSync(byte xA, byte yA, byte zA, byte dly) {
  int b0 = base.read(), r0 = rArm.read(), f0 = fArm.read();
  int sb = abs(xA - b0), sr = abs(yA - r0), sf = abs(zA - f0);
  int mx = sb; if (sr > mx) mx = sr; if (sf > mx) mx = sf;
  if (mx == 0) return;

  for (int i = 0; i <= mx; i++) {
    byte bp = sb == 0 ? b0 : b0 + (xA - b0) * i / sb;
    byte rp = sr == 0 ? r0 : r0 + (yA - r0) * i / sr;
    byte fp = sf == 0 ? f0 : f0 + (zA - f0) * i / sf;
    base.write(bp);
    rArm.write(rp);
    fArm.write(fp);
    delay(dly);
  }
}

// ====== 任务二：搬运（从 PROGMEM 读）======
void pickAndPlace(const byte *action, const char *name) {
  Serial.print("Pick&Place ");
  Serial.println(name);

  for (byte step = 0; step < 8; step++) {
    byte a = pgm_read_byte_near(action + step * 4 + 0);
    byte b = pgm_read_byte_near(action + step * 4 + 1);
    byte c = pgm_read_byte_near(action + step * 4 + 2);
    byte d = pgm_read_byte_near(action + step * 4 + 3);
    servoSync(a, b, c, DSD);
    servoCmd('c', d, DSD);
    delay(200);
  }
  Serial.println("Done");
}

// ====== 任务五：移动到坐标 ======
bool moveTo(float x, float y, float z) {
  float b, r, f;
  if (!inverseKinematics(x, y, z, b, r, f)) return false;

  byte steps = 20;
  float b0 = base.read(), r0 = rArm.read(), f0 = fArm.read();
  for (byte i = 1; i <= steps; i++) {
    base.write(b0 + (b - b0) * i / steps);
    rArm.write(r0 + (r - r0) * i / steps);
    fArm.write(f0 + (f - f0) * i / steps);
    delay(drawDelay);
  }
  currentX = x;
  currentY = y;
  currentZ = z;
  return true;
}

// ====== 任务五：画直线 ======
void drawLine(float x1, float y1, float x2, float y2) {
  float z = isPenUp ? (drawZ + penUpOffset) : drawZ;
  for (byte i = 0; i <= lineSteps; i++) {
    float t = (float)i / lineSteps;
    float x = x1 + (x2 - x1) * t;
    float y = y1 + (y2 - y1) * t;
    float b, r, f;
    if (inverseKinematics(x, y, z, b, r, f)) {
      base.write(b);
      rArm.write(r);
      fArm.write(f);
      delay(drawDelay);
    } else {
      Serial.print("Out of reach: (");
      Serial.print(x); Serial.print(","); Serial.print(y); Serial.println(")");
      return;
    }
  }
  currentX = x2;
  currentY = y2;
  currentZ = z;
}

// ====== 任务五：抬笔/落笔 ======
void penUp() {
  if (isPenUp) return;
  moveTo(currentX, currentY, drawZ + penUpOffset);
  isPenUp = true;
  Serial.println("Pen Up");
}
void penDown() {
  if (!isPenUp) return;
  moveTo(currentX, currentY, drawZ);
  isPenUp = false;
  Serial.println("Pen Down");
}

// ====== 显示状态 ======
void reportStatus() {
  Serial.print("B:"); Serial.print(base.read());
  Serial.print(" R:"); Serial.print(rArm.read());
  Serial.print(" F:"); Serial.print(fArm.read());
  Serial.print(" C:"); Serial.print(claw.read());
  Serial.print(" DSD:"); Serial.println(DSD);
  Serial.print("Pos:"); Serial.print(currentX);
  Serial.print(","); Serial.print(currentY);
  Serial.print(","); Serial.print(currentZ);
  Serial.print(" Pen:"); Serial.println(isPenUp ? "Up" : "Down");
}

// ====== 串口解析 ======
void armDataCmd(char c) {
  static int val = 0;
  static int sx = 0, sy = 0;
  static byte step = 0;  // 0=单舵机, 1=x, 2=y, 3=z

  // L指令缓存
  static char lbuf[32];
  static byte lbuf_i = 0;
  static bool isL = false;

  // 数字
  if (c >= '0' && c <= '9') {
    val = val * 10 + (c - '0');
    if (isL && lbuf_i < 31) lbuf[lbuf_i++] = c;
    return;
  }
  // L指令的逗号/小数点/负号/空格
  if (isL && (c == ',' || c == '.' || c == '-' || c == ' ')) {
    if (lbuf_i < 31) lbuf[lbuf_i++] = c;
    return;
  }

  switch (c) {
    case 'b': case 'r': case 'f': case 'c':
      val = 0; step = 0; break;
    case 'x':
      sx = 0; val = 0; step = 1; break;
    case 'y':
      if (step == 1) { sx = val; val = 0; step = 2; } break;
    case 'z':
      if (step == 2) { sy = val; val = 0; step = 3; } break;
    case ',':
      if (step == 1) { sx = val; val = 0; }
      else if (step == 2) { sy = val; val = 0; }
      break;
    case 'A': pickAndPlace(actionA, "A"); val = 0; break;
    case 'B': pickAndPlace(actionB, "B"); val = 0; break;
    case 'C': pickAndPlace(actionC, "C"); val = 0; break;
    case 'L': isL = true; lbuf_i = 0; val = 0; break;
    case 'U': penUp(); val = 0; break;
    case 'D': penDown(); val = 0; break;
    case 'o': reportStatus(); val = 0; break;
    case 'h':
      if (DSD > 1) DSD--;
      Serial.print("DSD="); Serial.println(DSD);
      val = 0; break;
    case 'l':
      DSD++;
      Serial.print("DSD="); Serial.println(DSD);
      val = 0; break;
    case '\n': case '\r': case ' ':
      if (isL) {
        lbuf[lbuf_i] = 0;
        float x1, y1, x2, y2;
        char *p = lbuf;
        x1 = atof(p);
        p = strchr(p, ','); if (p) p++; else { isL = false; break; }
        y1 = atof(p);
        p = strchr(p, ','); if (p) p++; else { isL = false; break; }
        x2 = atof(p);
        p = strchr(p, ','); if (p) p++; else { isL = false; break; }
        y2 = atof(p);
        Serial.print("Line:("); Serial.print(x1); Serial.print(","); Serial.print(y1);
        Serial.print(")->("); Serial.print(x2); Serial.print(","); Serial.print(y2); Serial.println(")");
        drawLine(x1, y1, x2, y2);
        Serial.println("Done");
        isL = false; lbuf_i = 0;
      }
      else if (step == 3) {
        int sz = val;
        Serial.print("Sync:"); Serial.print(sx); Serial.print(",");
        Serial.print(sy); Serial.print(","); Serial.println(sz);
        servoSync(sx, sy, sz, DSD);
        step = 0;
      }
      val = 0;
      break;
    default: val = 0; break;
  }

  // 单舵机指令执行
  static char lastCmd = 0;
  if (c == 'b' || c == 'r' || c == 'f' || c == 'c') {
    lastCmd = c;
  } else if (lastCmd != 0 && (c < '0' || c > '9') && val > 0 && val <= 180) {
    switch (lastCmd) {
      case 'b': Serial.print("Base->"); Serial.println(val); servoCmd('b', val, DSD); break;
      case 'r': Serial.print("RArm->"); Serial.println(val); servoCmd('r', val, DSD); break;
      case 'f': Serial.print("FArm->"); Serial.println(val); servoCmd('f', val, DSD); break;
      case 'c': Serial.print("Claw->"); Serial.println(val); servoCmd('c', val, DSD); break;
    }
    lastCmd = 0;
    val = 0;
  }
}

// ====== 摇杆 ======
void joyBase() {
  int v = analogRead(joyL_X);
  int p = base.read();
  if (v > 512 + joyDeadZone) { if (p < 180 - joyStep) base.write(p + joyStep); delay(DSD); }
  else if (v < 512 - joyDeadZone) { if (p > joyStep) base.write(p - joyStep); delay(DSD); }
}
void joyRArm() {
  int v = analogRead(joyL_Y);
  int p = rArm.read();
  if (v > 512 + joyDeadZone) { if (p < 180 - joyStep) rArm.write(p + joyStep); delay(DSD); }
  else if (v < 512 - joyDeadZone) { if (p > joyStep) rArm.write(p - joyStep); delay(DSD); }
}
void joyFArm() {
  int v = analogRead(joyR_X);
  int p = fArm.read();
  if (v > 512 + joyDeadZone) { if (p < 180 - joyStep) fArm.write(p + joyStep); delay(DSD); }
  else if (v < 512 - joyDeadZone) { if (p > joyStep) fArm.write(p - joyStep); delay(DSD); }
}
void joyClaw() {
  int v = analogRead(joyR_Y);
  int p = claw.read();
  if (v > 512 + joyDeadZone) { if (p < 180 - joyStep) claw.write(p + joyStep); delay(DSD); }
  else if (v < 512 - joyDeadZone) { if (p > joyStep) claw.write(p - joyStep); delay(DSD); }
}
void joystickControl() { joyBase(); joyRArm(); joyFArm(); joyClaw(); }

// ====== 任务三：循环搬运 ======
void nextPick() {
  if (isRecording || isPlaying) { Serial.println("Busy"); return; }
  currentTarget = (currentTarget + 1) % 3;
  if (currentTarget == 0) pickAndPlace(actionA, "A");
  else if (currentTarget == 1) pickAndPlace(actionB, "B");
  else pickAndPlace(actionC, "C");
}

// ====== 任务三：录制 ======
void startRec() {
  if (isPlaying) return;
  isRecording = true;
  recordCount = 0;
  lastRecordTime = millis();
  Serial.println("Record Start");
}
void stopRec() {
  isRecording = false;
  Serial.print("Record End: "); Serial.print(recordCount); Serial.println(" pts");
  int sec = (recordCount * RECORD_INTERVAL) / 1000;
  Serial.print("Time: "); Serial.print(sec); Serial.println("s");
  if (sec < 10) Serial.println("Warn: <10s");
}
void recordUpdate() {
  if (!isRecording) return;
  if (recordCount >= MAX_RECORD) { stopRec(); return; }
  if (millis() - lastRecordTime >= RECORD_INTERVAL) {
    recordData[recordCount][0] = base.read();
    recordData[recordCount][1] = rArm.read();
    recordData[recordCount][2] = fArm.read();
    recordData[recordCount][3] = claw.read();
    recordCount++;
    lastRecordTime = millis();
  }
}

// ====== 任务三：播放 ======
void startPlay() {
  if (isRecording) return;
  if (recordCount == 0) { Serial.println("No data"); return; }
  isPlaying = true;
  playIndex = 0;
  lastPlayTime = millis();
  Serial.println("Play Start");
}
void playUpdate() {
  if (!isPlaying) return;
  if (playIndex >= recordCount) { isPlaying = false; Serial.println("Play End"); return; }
  if (millis() - lastPlayTime >= RECORD_INTERVAL) {
    base.write(recordData[playIndex][0]);
    rArm.write(recordData[playIndex][1]);
    fArm.write(recordData[playIndex][2]);
    claw.write(recordData[playIndex][3]);
    playIndex++;
    lastPlayTime = millis();
  }
}

// ====== 任务三：回中 ======
void goHome() {
  if (isRecording || isPlaying) { Serial.println("Busy"); return; }
  Serial.println("Home");
  servoSync(90, 90, 90, DSD);
  servoCmd('c', clawOpen, DSD);
}

// ====== 任务三：按键板指令 ======
void processRemote() {
  if (armSerial.available() > 0) {
    char cmd = armSerial.read();
    Serial.print("Remote: "); Serial.println(cmd);
    switch (cmd) {
      case '1': nextPick(); break;
      case '2': if (!isRecording) startRec(); else stopRec(); break;
      case '3': startPlay(); break;
      case '4': goHome(); break;
    }
  }
}

// ====== 主循环 ======
void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    armDataCmd(c);
  }
  processRemote();
  recordUpdate();
  playUpdate();
  if (!isPlaying) joystickControl();
}
