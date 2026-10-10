/*
  任务一：机械臂基础控制 + 串口通信 + 摇杆操控
  机器人学院十二期二面考核 - 算法方向
  
  基于太极创客 MeArm 教程风格
  
  指令列表：
    b + 数字  → 底座转到指定角度   例如: b90
    r + 数字  → 大臂转到指定角度   例如: r45
    f + 数字  → 小臂转到指定角度   例如: f120
    c + 数字  → 夹爪转到指定角度   例如: c30
    o         → 输出所有舵机当前角度
    h         → 加速（减小DSD）
    l         → 减速（增大DSD）
    
  双摇杆控制：
    左摇杆左右 → 底座旋转
    左摇杆前后 → 大臂抬落
    右摇杆左右 → 小臂伸缩
    右摇杆前后 → 夹爪开合
*/

#include <Servo.h>  // 使用Servo库

Servo base, rArm, fArm, claw;  // 创建4个舵机对象

// 存储电机极限值（const指定该数值为常量，程序运行中不能改变）
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 45;
const int rArmMax = 180;
const int fArmMin = 35;
const int fArmMax = 120;
const int clawMin = 25;
const int clawMax = 100;

// 当前各舵机角度
int basePos = 90;
int rArmPos = 90;
int fArmPos = 90;
int clawPos = 90;

int DSD = 15;  // Default Servo Delay（默认电机运动延迟时间）
               // 增大此数值降低电机运行速度

// 左摇杆引脚
const int leftJoyXPin  = A0;  // 左摇杆X轴（左右 → 底座旋转）
const int leftJoyYPin  = A1;  // 左摇杆Y轴（前后 → 大臂抬落）

// 右摇杆引脚
const int rightJoyXPin = A2;  // 右摇杆X轴（左右 → 小臂伸缩）
const int rightJoyYPin = A3;  // 右摇杆Y轴（前后 → 夹爪开合）

// 摇杆死区
const int joyDeadZone = 60;

void setup() {
  base.attach(11);   // base 舵机连接引脚11，舵机代号 'b'
  delay(200);        // 稳定性等待
  rArm.attach(10);   // rArm 舵机连接引脚10，舵机代号 'r'
  delay(200);
  fArm.attach(9);    // fArm 舵机连接引脚9，舵机代号 'f'
  delay(200);
  claw.attach(6);    // claw 舵机连接引脚6，舵机代号 'c'
  delay(200);

  // 初始化角度
  base.write(90);
  delay(10);
  fArm.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  claw.write(90);
  delay(10);

  Serial.begin(9600);
  Serial.println("============================");
  Serial.println("   MeArm 任务一 - 基础控制");
  Serial.println("============================");
  Serial.println("指令：");
  Serial.println("  b + 数字 → 底座角度");
  Serial.println("  r + 数字 → 大臂角度");
  Serial.println("  f + 数字 → 小臂角度");
  Serial.println("  c + 数字 → 夹爪角度");
  Serial.println("  o        → 查看状态");
  Serial.println("  h        → 加速");
  Serial.println("  l        → 减速");
  Serial.println("============================");
}

void loop() {
  // 处理串口指令
  if (Serial.available() > 0) {
    char serialCmd = Serial.read();
    armDataCmd(serialCmd);
  }
  
  // 摇杆控制
  joystickControl();
}

// ============================================================
// 解析串口指令
// ============================================================
void armDataCmd(char serialCmd) {
  // 如果是 b/c/f/r，说明后面跟着角度数字
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r') {
    int servoData = Serial.parseInt();
    servoCmd(serialCmd, servoData, DSD);  // 调用舵机移动函数
  }
  // 其他单字符指令
  else {
    switch (serialCmd) {
      case 'o':  // 输出舵机状态信息
        reportStatus();
        break;
        
      case 'h':  // 加速（减小延时）
        if (DSD > 1) {
          DSD--;
        }
        Serial.print("Speed up. DSD = ");
        Serial.println(DSD);
        break;
        
      case 'l':  // 减速（增大延时）
        if (DSD < 50) {
          DSD++;
        }
        Serial.print("Speed down. DSD = ");
        Serial.println(DSD);
        break;
        
      default:  // 未知指令
        Serial.println("Unknown Command.");
        break;
    }
  }
}

// ============================================================
// 单个舵机平滑移动
// 参数：舵机名称，目标角度，每步延时(ms)
// ============================================================
void servoCmd(char servoName, int toPos, int servoDelay) {
  int fromPos;  // 当前角度
  
  // 串口输出确认信息
  Serial.println("");
  Serial.print("+Command: Servo ");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");

  switch (servoName) {
    // ====== 底座 base ======
    case 'b':
      if (toPos >= baseMin && toPos <= baseMax) {
        fromPos = base.read();
        
        if (fromPos <= toPos) {
          for (int i = fromPos; i <= toPos; i++) {
            base.write(i);
            delay(servoDelay);
          }
        } else {
          for (int i = fromPos; i >= toPos; i--) {
            base.write(i);
            delay(servoDelay);
          }
        }
        basePos = toPos;  // 更新当前角度记录
      } else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
      }
      break;

    // ====== 大臂 rArm ======
    case 'r':
      if (toPos >= rArmMin && toPos <= rArmMax) {
        fromPos = rArm.read();
        
        if (fromPos <= toPos) {
          for (int i = fromPos; i <= toPos; i++) {
            rArm.write(i);
            delay(servoDelay);
          }
        } else {
          for (int i = fromPos; i >= toPos; i--) {
            rArm.write(i);
            delay(servoDelay);
          }
        }
        rArmPos = toPos;
      } else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
      }
      break;

    // ====== 小臂 fArm ======
    case 'f':
      if (toPos >= fArmMin && toPos <= fArmMax) {
        fromPos = fArm.read();
        
        if (fromPos <= toPos) {
          for (int i = fromPos; i <= toPos; i++) {
            fArm.write(i);
            delay(servoDelay);
          }
        } else {
          for (int i = fromPos; i >= toPos; i--) {
            fArm.write(i);
            delay(servoDelay);
          }
        }
        fArmPos = toPos;
      } else {
        Serial.println("+Warning: fArm Servo Value Out Of Limit!");
      }
      break;

    // ====== 夹爪 claw ======
    case 'c':
      if (toPos >= clawMin && toPos <= clawMax) {
        fromPos = claw.read();
        
        if (fromPos <= toPos) {
          for (int i = fromPos; i <= toPos; i++) {
            claw.write(i);
            delay(servoDelay);
          }
        } else {
          for (int i = fromPos; i >= toPos; i--) {
            claw.write(i);
            delay(servoDelay);
          }
        }
        clawPos = toPos;
      } else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
      }
      break;

    default:
      Serial.println("+Warning: Unknown servo name!");
      break;
  }
}

// ============================================================
// 输出舵机状态信息
// ============================================================
void reportStatus() {
  Serial.println("");
  Serial.println("--- Robot Arm Status ---");
  Serial.print("Base: ");  Serial.print(base.read());  Serial.print("°  (");
  Serial.print(baseMin);  Serial.print("-");  Serial.print(baseMax);  Serial.println(")");
  
  Serial.print("rArm: ");  Serial.print(rArm.read());  Serial.print("°  (");
  Serial.print(rArmMin);  Serial.print("-");  Serial.print(rArmMax);  Serial.println(")");
  
  Serial.print("fArm: ");  Serial.print(fArm.read());  Serial.print("°  (");
  Serial.print(fArmMin);  Serial.print("-");  Serial.print(fArmMax);  Serial.println(")");
  
  Serial.print("Claw: ");  Serial.print(claw.read());  Serial.print("°  (");
  Serial.print(clawMin);  Serial.print("-");  Serial.print(clawMax);  Serial.println(")");
  
  Serial.print("Speed (DSD): ");  Serial.println(DSD);
  Serial.println("-------------------------");
  Serial.println("");
}

// ============================================================
// 双摇杆控制（太极创客风格 - 自定义函数 + 阻塞式平滑移动）
//   左摇杆左右 → 底座旋转
//   左摇杆前后 → 大臂抬落
//   右摇杆左右 → 小臂伸缩
//   右摇杆前后 → 夹爪开合
// ============================================================
void joystickControl() {
  int leftJoyX  = analogRead(leftJoyXPin);   // 左摇杆X值
  int leftJoyY  = analogRead(leftJoyYPin);   // 左摇杆Y值
  int rightJoyX = analogRead(rightJoyXPin);  // 右摇杆X值
  int rightJoyY = analogRead(rightJoyYPin);  // 右摇杆Y值
  
  // 左摇杆左右 → 底座
  joyBaseControl(leftJoyX);
  
  // 左摇杆前后 → 大臂
  joyRArmControl(leftJoyY);
  
  // 右摇杆左右 → 小臂
  joyFArmControl(rightJoyX);
  
  // 右摇杆前后 → 夹爪
  joyClawControl(rightJoyY);
}

// ============================================================
// 左摇杆左右控制底座旋转
// ============================================================
void joyBaseControl(int joyVal) {
  int center = 512;
  int step = 5;  // 每次移动的角度
  
  if (joyVal - center > joyDeadZone) {
    // 摇杆向右 → 底座角度增加
    int target = basePos + step;
    if (target > baseMax) target = baseMax;
    if (target != basePos) {
      servoCmd('b', target, DSD);
    }
  }
  else if (center - joyVal > joyDeadZone) {
    // 摇杆向左 → 底座角度减小
    int target = basePos - step;
    if (target < baseMin) target = baseMin;
    if (target != basePos) {
      servoCmd('b', target, DSD);
    }
  }
}

// ============================================================
// 左摇杆前后控制大臂抬落
// ============================================================
void joyRArmControl(int joyVal) {
  int center = 512;
  int step = 5;
  
  if (joyVal - center > joyDeadZone) {
    // 摇杆向后 → 大臂抬起（角度增大）
    int target = rArmPos + step;
    if (target > rArmMax) target = rArmMax;
    if (target != rArmPos) {
      servoCmd('r', target, DSD);
    }
  }
  else if (center - joyVal > joyDeadZone) {
    // 摇杆向前 → 大臂落下（角度减小）
    int target = rArmPos - step;
    if (target < rArmMin) target = rArmMin;
    if (target != rArmPos) {
      servoCmd('r', target, DSD);
    }
  }
}

// ============================================================
// 右摇杆左右控制小臂伸缩
// ============================================================
void joyFArmControl(int joyVal) {
  int center = 512;
  int step = 5;
  
  if (joyVal - center > joyDeadZone) {
    // 摇杆向右 → 小臂伸出（角度增大）
    int target = fArmPos + step;
    if (target > fArmMax) target = fArmMax;
    if (target != fArmPos) {
      servoCmd('f', target, DSD);
    }
  }
  else if (center - joyVal > joyDeadZone) {
    // 摇杆向左 → 小臂收回（角度减小）
    int target = fArmPos - step;
    if (target < fArmMin) target = fArmMin;
    if (target != fArmPos) {
      servoCmd('f', target, DSD);
    }
  }
}

// ============================================================
// 右摇杆前后控制夹爪开合
// ============================================================
void joyClawControl(int joyVal) {
  int center = 512;
  int step = 8;  // 夹爪动快一点
  
  if (joyVal - center > joyDeadZone) {
    // 摇杆向后 → 夹爪闭合（角度增大）
    int target = clawPos + step;
    if (target > clawMax) target = clawMax;
    if (target != clawPos) {
      servoCmd('c', target, DSD);
    }
  }
  else if (center - joyVal > joyDeadZone) {
    // 摇杆向前 → 夹爪张开（角度减小）
    int target = clawPos - step;
    if (target < clawMin) target = clawMin;
    if (target != clawPos) {
      servoCmd('c', target, DSD);
    }
  }
}
