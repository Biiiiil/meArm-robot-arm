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
  Serial.println("Robot Arm Start");
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

  // 串口输出确认信息
  Serial.println("");
  Serial.print("+Command:Servo");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");

  switch (servoName) {
    // ====== 底座 ======
    case 'b':
      if (toPos < baseMin || toPos > baseMax) {
        Serial.println("+Warning:Base Servo Value Out Of Limit!");
        return;
      }
      fromPos = base.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) {
          base.write(i);
          delay(servoDelay);
        }
      } else {
        for (i = fromPos; i >= toPos; i--) {
          base.write(i);
          delay(servoDelay);
        }
      }
      break;

    // ====== 夹爪 ======
    case 'c':
      if (toPos < clawMin || toPos > clawMax) {
        Serial.println("+Warning:Claw Servo Value Out Of Limit!");
        return;
      }
      fromPos = claw.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) {
          claw.write(i);
          delay(servoDelay);
        }
      } else {
        for (i = fromPos; i >= toPos; i--) {
          claw.write(i);
          delay(servoDelay);
        }
      }
      break;

    // ====== 小臂 ======
    case 'f':
      if (toPos < fArmMin || toPos > fArmMax) {
        Serial.println("+Warning:fArm Servo Value Out Of Limit!");
        return;
      }
      fromPos = fArm.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) {
          fArm.write(i);
          delay(servoDelay);
        }
      } else {
        for (i = fromPos; i >= toPos; i--) {
          fArm.write(i);
          delay(servoDelay);
        }
      }
      break;

    // ====== 大臂 ======
    case 'r':
      if (toPos < rArmMin || toPos > rArmMax) {
        Serial.println("+Warning:rArm Servo Value Out Of Limit!");
        return;
      }
      fromPos = rArm.read();
      if (fromPos <= toPos) {
        for (i = fromPos; i <= toPos; i++) {
          rArm.write(i);
          delay(servoDelay);
        }
      } else {
        for (i = fromPos; i >= toPos; i--) {
          rArm.write(i);
          delay(servoDelay);
        }
      }
      break;

    default:
      Serial.println("+Warning:Unknown servo name!");
      break;
  }
}

// ============================================================
// 解析串口指令
// ============================================================
void armDataCmd(char serialCmd) {
  // 如果是 b/c/f/r，说明后面跟着角度数字
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r') {
    int servoData = Serial.parseInt();       // 读取后面的数字
    servoCmd(serialCmd, servoData, DSD);     // 调用舵机移动
  }
  // 其他单字符指令
  else {
    switch (serialCmd) {
      case 'o':  // 输出所有舵机当前角度
        Serial.println("--- Status ---");
        Serial.print("Base: ");  Serial.println(base.read());
        Serial.print("rArm: ");  Serial.println(rArm.read());
        Serial.print("fArm: ");  Serial.println(fArm.read());
        Serial.print("Claw: ");  Serial.println(claw.read());
        Serial.println("--------------");
        break;

      case 'h':  // 加速（减小延时）
        if (DSD > 1) DSD--;
        Serial.print("Speed up, DSD = ");
        Serial.println(DSD);
        break;

      case 'l':  // 减速（增大延时）
        if (DSD < 50) DSD++;
        Serial.print("Speed down, DSD = ");
        Serial.println(DSD);
        break;

      default:  // 未知指令
        Serial.println("Unknown command");
        break;
    }
  }
}

// ============================================================
// 摇杆控制 - 底座（左摇杆左右）
// ============================================================
void joyBaseControl() {
  int joyVal = analogRead(joyL_X);
  int currentPos = base.read();
  int targetPos = currentPos;

  if (joyVal > 512 + joyDeadzone) {         // 向右扳
    targetPos = currentPos + joyStep;
    if (targetPos > baseMax) targetPos = baseMax;
  } else if (joyVal < 512 - joyDeadzone) {  // 向左扳
    targetPos = currentPos - joyStep;
    if (targetPos < baseMin) targetPos = baseMin;
  }

  if (targetPos != currentPos) {
    servoCmd('b', targetPos, DSD);
  }
}

// ============================================================
// 摇杆控制 - 大臂（左摇杆前后）
// ============================================================
void joyRArmControl() {
  int joyVal = analogRead(joyL_Y);
  int currentPos = rArm.read();
  int targetPos = currentPos;

  if (joyVal > 512 + joyDeadzone) {         // 向前推 → 大臂落下
    targetPos = currentPos - joyStep;
    if (targetPos < rArmMin) targetPos = rArmMin;
  } else if (joyVal < 512 - joyDeadzone) {  // 向后拉 → 大臂抬起
    targetPos = currentPos + joyStep;
    if (targetPos > rArmMax) targetPos = rArmMax;
  }

  if (targetPos != currentPos) {
    servoCmd('r', targetPos, DSD);
  }
}

// ============================================================
// 摇杆控制 - 小臂（右摇杆左右）
// ============================================================
void joyFArmControl() {
  int joyVal = analogRead(joyR_X);
  int currentPos = fArm.read();
  int targetPos = currentPos;

  if (joyVal > 512 + joyDeadzone) {         // 向右扳 → 小臂伸出
    targetPos = currentPos + joyStep;
    if (targetPos > fArmMax) targetPos = fArmMax;
  } else if (joyVal < 512 - joyDeadzone) {  // 向左扳 → 小臂收回
    targetPos = currentPos - joyStep;
    if (targetPos < fArmMin) targetPos = fArmMin;
  }

  if (targetPos != currentPos) {
    servoCmd('f', targetPos, DSD);
  }
}

// ============================================================
// 摇杆控制 - 夹爪（右摇杆前后）
// ============================================================
void joyClawControl() {
  int joyVal = analogRead(joyR_Y);
  int currentPos = claw.read();
  int targetPos = currentPos;

  if (joyVal > 512 + joyDeadzone) {         // 向前推 → 张开
    targetPos = currentPos + joyStep;
    if (targetPos > clawMax) targetPos = clawMax;
  } else if (joyVal < 512 - joyDeadzone) {  // 向后拉 → 闭合
    targetPos = currentPos - joyStep;
    if (targetPos < clawMin) targetPos = clawMin;
  }

  if (targetPos != currentPos) {
    servoCmd('c', targetPos, DSD);
  }
}

// ============================================================
// 摇杆控制总函数
// ============================================================
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

  joystickControl();   // 摇杆控制
}
