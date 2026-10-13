#include <Servo.h>     //使用Servo库
Servo base, rArm, fArm, claw;  //创建4个舵机 

//存储极限值
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 0;
const int rArmMax = 180;
const int fArmMin = 0;
const int fArmMax = 180;
const int clawMin = 0;
const int clawMax = 180;

int DSD = 15;   //默认运动速度，调节运行速度（数值越小越快）

// ====== 摇杆引脚定义 ======
const int joyL_X = A0;   // 左摇杆X → 底座
const int joyL_Y = A1;   // 左摇杆Y → 大臂
const int joyR_X = A2;   // 右摇杆X → 小臂
const int joyR_Y = A3;   // 右摇杆Y → 夹爪

const int joyDeadzone = 50;   // 摇杆死区（中间位置不响应的范围）
const int joyStep = 1;        // 每次摇杆移动的角度步长

// ============================================================
// 任务二：A/B/C 物体搬运动作序列
// 每个数组 8 行 × 4 列
// 列顺序：{底座, 大臂, 小臂, 夹爪}
// 行顺序：1.移到物体上方 2.下降 3.夹爪闭合 4.抬起
//         5.移到放置点上方 6.下降 7.夹爪张开 8.回中
// ⚠️ 角度是占位值，需要实际调试后替换！
// 调试方法：用 b/r/f/c 指令摆好位置，再用 o 查看角度，填到这里
// ============================================================

// 夹爪角度（按最大范围，需要实际调试）
const int clawOpen = 10;    // 夹爪张开角度
const int clawClose = 100;  // 夹爪闭合角度

// ====== 物体A搬运动作（硬币） ======
int actionA[8][4] = {
  { 90, 100, 80, clawOpen},     // 1. 移到A上方
  { 90, 130, 50, clawOpen},     // 2. 下降到A
  { 90, 130, 50, clawClose},    // 3. 夹爪闭合
  { 90, 100, 80, clawClose},    // 4. 抬起
  { 60, 100, 80, clawClose},    // 5. 移到A放置点上方
  { 60, 130, 50, clawClose},    // 6. 下降
  { 60, 130, 50, clawOpen},     // 7. 夹爪张开
  { 90, 90,  90, clawOpen},      // 8. 回中
};

// ====== 物体B搬运动作（笔） ======
int actionB[8][4] = {
  { 70, 100, 80, clawOpen},     // 1. 移到B上方
  { 70, 130, 50, clawOpen},     // 2. 下降到B
  { 70, 130, 50, clawClose},    // 3. 夹爪闭合
  { 70, 100, 80, clawClose},    // 4. 抬起
  { 40, 100, 80, clawClose},    // 5. 移到B放置点上方
  { 40, 130, 50, clawClose},    // 6. 下降
  { 40, 130, 50, clawOpen},     // 7. 夹爪张开
  { 90, 90,  90, clawOpen},      // 8. 回中
};

// ====== 物体C搬运动作（自定义物体） ======
int actionC[8][4] = {
  { 110, 100, 80, clawOpen},    // 1. 移到C上方
  { 110, 130, 50, clawOpen},    // 2. 下降到C
  { 110, 130, 50, clawClose},   // 3. 夹爪闭合
  { 110, 100, 80, clawClose},   // 4. 抬起
  { 120, 100, 80, clawClose},   // 5. 移到C放置点上方
  { 120, 130, 50, clawClose},   // 6. 下降
  { 120, 130, 50, clawOpen},    // 7. 夹爪张开
  { 90, 90,  90, clawOpen},      // 8. 回中
};

void setup() {
  base.attach(2);  //填入你实际接的引脚
  delay(200);
  rArm.attach(3);
  delay(200);
  fArm.attach(4);
  delay(200);
  claw.attach(5);
  delay(200);

  base.write(90);  //初始化
  delay(10);
  fArm.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  claw.write(90);
  delay(10);

  Serial.begin(9600);
  Serial.println("Robot Arm Start - Task2 Ready");
  Serial.println("Commands: b/r/f/c+angle | x,y,z | A/B/C | o | h/l");
}

// ============================================================
// 舵机指令函数
// servoName: 'b'底座 'r'大臂 'f'小臂 'c'夹爪
// toPos: 目标角度
// servoDelay: 每步延时(ms)
// ============================================================
void servoCmd(char servoName, int toPos, int servoDelay) {
  int fromPos;  // 当前角度
  int i;

  Serial.println("");
  Serial.print("+Command:Servo");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");

  switch (servoName) {
    case 'b':
      if (toPos < baseMin || toPos > baseMax) {
        Serial.println("+Warning:Base Servo Value Out Of Limit!");
        return;
      }
      fromPos = base.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) { base.write(i); delay(servoDelay); }
      } else {
        for (i = fromPos; i >= toPos; i--) { base.write(i); delay(servoDelay); }
      }
      break;

    case 'c':
      if (toPos < clawMin || toPos > clawMax) {
        Serial.println("+Warning:Claw Servo Value Out Of Limit!");
        return;
      }
      fromPos = claw.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) { claw.write(i); delay(servoDelay); }
      } else {
        for (i = fromPos; i >= toPos; i--) { claw.write(i); delay(servoDelay); }
      }
      break;

    case 'f':
      if (toPos < fArmMin || toPos > fArmMax) {
        Serial.println("+Warning:fArm Servo Value Out Of Limit!");
        return;
      }
      fromPos = fArm.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) { fArm.write(i); delay(servoDelay); }
      } else {
        for (i = fromPos; i >= toPos; i--) { fArm.write(i); delay(servoDelay); }
      }
      break;

    case 'r':
      if (toPos < rArmMin || toPos > rArmMax) {
        Serial.println("+Warning:rArm Servo Value Out Of Limit!");
        return;
      }
      fromPos = rArm.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) { rArm.write(i); delay(servoDelay); }
      } else {
        for (i = fromPos; i >= toPos; i--) { rArm.write(i); delay(servoDelay); }
      }
      break;

    default:
      Serial.println("+Warning:Unknown servo name!");
      break;
  }
}

// ============================================================
// 多舵机协同控制（x,y,z 同步移动）
// x = 底座, y = 大臂, z = 小臂
// ============================================================
void servoSyncCmd(int xTarget, int yTarget, int zTarget, int servoDelay) {
  int fromX = base.read();
  int fromY = rArm.read();
  int fromZ = fArm.read();

  if (xTarget < baseMin || xTarget > baseMax) { Serial.println("+Warning:X Out Of Limit!"); return; }
  if (yTarget < rArmMin || yTarget > rArmMax) { Serial.println("+Warning:Y Out Of Limit!"); return; }
  if (zTarget < fArmMin || zTarget > fArmMax) { Serial.println("+Warning:Z Out Of Limit!"); return; }

  int stepsX = abs(xTarget - fromX);
  int stepsY = abs(yTarget - fromY);
  int stepsZ = abs(zTarget - fromZ);

  int totalSteps = stepsX;
  if (stepsY > totalSteps) totalSteps = stepsY;
  if (stepsZ > totalSteps) totalSteps = stepsZ;

  if (totalSteps == 0) { Serial.println("+Sync: Already at target."); return; }

  for (int i = 1; i <= totalSteps; i++) {
    int posX = fromX + (long)(xTarget - fromX) * i / totalSteps;
    int posY = fromY + (long)(yTarget - fromY) * i / totalSteps;
    int posZ = fromZ + (long)(zTarget - fromZ) * i / totalSteps;
    base.write(posX);
    rArm.write(posY);
    fArm.write(posZ);
    delay(servoDelay);
  }
}

// ============================================================
// 任务二：物体搬运函数
// action: 8行4列的二维数组，每行是一个阶段的4个舵机角度
// 顺序：{底座, 大臂, 小臂, 夹爪}
// ============================================================
void pickAndPlace(int action[8][4], const char* name) {
  Serial.println("");
  Serial.print("===== Pick & Place: ");
  Serial.print(name);
  Serial.println(" =====");

  for (int step = 0; step < 8; step++) {
    Serial.print("Step ");
    Serial.print(step + 1);
    Serial.print("/8  b=");
    Serial.print(action[step][0]);
    Serial.print(" r=");
    Serial.print(action[step][1]);
    Serial.print(" f=");
    Serial.print(action[step][2]);
    Serial.print(" c=");
    Serial.println(action[step][3]);

    // 用 servoSyncCmd 同步移动底座、大臂、小臂
    servoSyncCmd(action[step][0], action[step][1], action[step][2], DSD);

    // 夹爪单独控制（夹爪动作要和三个舵机分开，避免同时动影响抓取）
    servoCmd('c', action[step][3], DSD);

    delay(300);  // 每个阶段之间停300ms
  }

  Serial.println("===== Pick & Place Done =====");
  Serial.println("");
}

// ============================================================
// 解析串口指令
// ============================================================
void armDataCmd(char serialCmd) {
  // 多舵机协同指令：x10,y30,z20
  if (serialCmd == 'x') {
    int xVal = Serial.parseInt();
    Serial.read(); Serial.read();
    int yVal = Serial.parseInt();
    Serial.read(); Serial.read();
    int zVal = Serial.parseInt();
    servoSyncCmd(xVal, yVal, zVal, DSD);
  }
  // 单舵机指令：b/c/f/r + 数字
  else if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r') {
    int servoData = Serial.parseInt();
    servoCmd(serialCmd, servoData, DSD);
  }
  // 任务二：物体搬运指令
  else if (serialCmd == 'A') {
    pickAndPlace(actionA, "A");
  }
  else if (serialCmd == 'B') {
    pickAndPlace(actionB, "B");
  }
  else if (serialCmd == 'C') {
    pickAndPlace(actionC, "C");
  }
  // 其他单字符指令
  else {
    switch (serialCmd) {
      case 'o':
        Serial.println("--- Status ---");
        Serial.print("Base: ");  Serial.println(base.read());
        Serial.print("rArm: ");  Serial.println(rArm.read());
        Serial.print("fArm: ");  Serial.println(fArm.read());
        Serial.print("Claw: ");  Serial.println(claw.read());
        Serial.println("--------------");
        break;

      case 'h':
        if (DSD > 1) DSD--;
        Serial.print("Speed up, DSD = ");
        Serial.println(DSD);
        break;

      case 'l':
        if (DSD < 50) DSD++;
        Serial.print("Speed down, DSD = ");
        Serial.println(DSD);
        break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

// ============================================================
// 摇杆控制函数
// ============================================================
void joyBaseControl() {
  int joyVal = analogRead(joyL_X);
  int currentPos = base.read();
  int targetPos = currentPos;
  if (joyVal > 512 + joyDeadzone) { targetPos = currentPos + joyStep; if (targetPos > baseMax) targetPos = baseMax; }
  else if (joyVal < 512 - joyDeadzone) { targetPos = currentPos - joyStep; if (targetPos < baseMin) targetPos = baseMin; }
  if (targetPos != currentPos) servoCmd('b', targetPos, DSD);
}

void joyRArmControl() {
  int joyVal = analogRead(joyL_Y);
  int currentPos = rArm.read();
  int targetPos = currentPos;
  if (joyVal > 512 + joyDeadzone) { targetPos = currentPos - joyStep; if (targetPos < rArmMin) targetPos = rArmMin; }
  else if (joyVal < 512 - joyDeadzone) { targetPos = currentPos + joyStep; if (targetPos > rArmMax) targetPos = rArmMax; }
  if (targetPos != currentPos) servoCmd('r', targetPos, DSD);
}

void joyFArmControl() {
  int joyVal = analogRead(joyR_X);
  int currentPos = fArm.read();
  int targetPos = currentPos;
  if (joyVal > 512 + joyDeadzone) { targetPos = currentPos + joyStep; if (targetPos > fArmMax) targetPos = fArmMax; }
  else if (joyVal < 512 - joyDeadzone) { targetPos = currentPos - joyStep; if (targetPos < fArmMin) targetPos = fArmMin; }
  if (targetPos != currentPos) servoCmd('f', targetPos, DSD);
}

void joyClawControl() {
  int joyVal = analogRead(joyR_Y);
  int currentPos = claw.read();
  int targetPos = currentPos;
  if (joyVal > 512 + joyDeadzone) { targetPos = currentPos + joyStep; if (targetPos > clawMax) targetPos = clawMax; }
  else if (joyVal < 512 - joyDeadzone) { targetPos = currentPos - joyStep; if (targetPos < clawMin) targetPos = clawMin; }
  if (targetPos != currentPos) servoCmd('c', targetPos, DSD);
}

void joystickControl() {
  joyBaseControl();
  joyRArmControl();
  joyFArmControl();
  joyClawControl();
}

// ============================================================
// 主循环
// ============================================================
void loop() {
  if (Serial.available() > 0) {
    char serialCmd = Serial.read();
    armDataCmd(serialCmd);
  }
  joystickControl();
}
