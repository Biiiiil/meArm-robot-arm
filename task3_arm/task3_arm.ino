/*
  ============================================================
  meArm 机械臂算法考核 - 机械臂端完整代码
  ============================================================
  包含：
  - 任务一：基础控制 + 串口通信 + 双摇杆操控 + 多舵机协同
  - 任务二：A/B/C 物体搬运（8阶段预定义动作序列）
  - 任务三：按键遥控 + 循环搬运 + 录制/播放 + 回中
  ============================================================
  引脚分配：
  舵机：base=2, rArm=3, fArm=4, claw=5
  软串口：RX=8, TX=9（和按键板通信）
  摇杆：joyL_X=A0, joyL_Y=A1, joyR_X=A2, joyR_Y=A3
  ============================================================
*/

#include <Servo.h>
#include <SoftwareSerial.h>

// ============================================================
// 舵机引脚定义
// ============================================================
const int pinBase = 2;   // 底座
const int pinRArm = 3;   // 大臂
const int pinFArm = 4;   // 小臂
const int pinClaw = 5;   // 夹爪

Servo base;   // 底座舵机
Servo rArm;   // 大臂舵机
Servo fArm;   // 小臂舵机
Servo claw;   // 夹爪舵机

// ============================================================
// 摇杆引脚定义
// ============================================================
const int joyL_X = A0;   // 左摇杆X → 底座
const int joyL_Y = A1;   // 左摇杆Y → 大臂
const int joyR_X = A2;   // 右摇杆X → 小臂
const int joyR_Y = A3;   // 右摇杆Y → 夹爪

// 摇杆死区（中间位置的范围，防止抖动）
const int joyDeadZone = 50;
// 摇杆步进量（每次改变多少度）
const int joyStep = 2;

// ============================================================
// 软串口引脚（和按键遥控板通信）
// ============================================================
const int softRX = 8;    // 接收 ← 接按键板的 TX(10)
const int softTX = 9;    // 发送 → 接按键板的 RX(11)
SoftwareSerial armSerial(softRX, softTX);

// ============================================================
// 夹爪开合角度（调试时根据实际修改）
// ============================================================
const int clawOpen = 10;    // 张开角度
const int clawClose = 100;  // 闭合角度

// ============================================================
// 舵机移动速度（DSD越大越慢）
// ============================================================
int DSD = 15;

// ============================================================
// 任务二：A/B/C 物体搬运动作序列
// 每个动作8个阶段，每阶段4个舵机角度：{底座, 大臂, 小臂, 夹爪}
// 阶段1：移到物体上方
// 阶段2：下降到物体
// 阶段3：夹爪闭合（抓取）
// 阶段4：抬起
// 阶段5：移到放置点上方
// 阶段6：下降到放置点
// 阶段7：夹爪张开（释放）
// 阶段8：回中
// 注意：以下角度是占位值，需要实际调试后修改！
// ============================================================

// 物体A动作序列（比如硬币）
int actionA[8][4] = {
  { 90, 100,  80, clawOpen},   // 阶段1：移到A上方
  { 90, 130,  50, clawOpen},   // 阶段2：下降到A
  { 90, 130,  50, clawClose},  // 阶段3：夹爪闭合
  { 90, 100,  80, clawClose},  // 阶段4：抬起
  { 60, 100,  80, clawClose},  // 阶段5：移到A放置点上方
  { 60, 130,  50, clawClose},  // 阶段6：下降到放置点
  { 60, 130,  50, clawOpen},   // 阶段7：夹爪张开
  { 90,  90,  90, clawOpen}    // 阶段8：回中
};

// 物体B动作序列（比如笔）
int actionB[8][4] = {
  {120, 100,  80, clawOpen},   // 阶段1：移到B上方
  {120, 130,  50, clawOpen},   // 阶段2：下降到B
  {120, 130,  50, clawClose},  // 阶段3：夹爪闭合
  {120, 100,  80, clawClose},  // 阶段4：抬起
  {150, 100,  80, clawClose},  // 阶段5：移到B放置点上方
  {150, 130,  50, clawClose},  // 阶段6：下降到放置点
  {150, 130,  50, clawOpen},   // 阶段7：夹爪张开
  { 90,  90,  90, clawOpen}    // 阶段8：回中
};

// 物体C动作序列（自定义物体）
int actionC[8][4] = {
  { 45, 100,  80, clawOpen},   // 阶段1：移到C上方
  { 45, 130,  50, clawOpen},   // 阶段2：下降到C
  { 45, 130,  50, clawClose},  // 阶段3：夹爪闭合
  { 45, 100,  80, clawClose},  // 阶段4：抬起
  { 30, 100,  80, clawClose},  // 阶段5：移到C放置点上方
  { 30, 130,  50, clawClose},  // 阶段6：下降到放置点
  { 30, 130,  50, clawOpen},   // 阶段7：夹爪张开
  { 90,  90,  90, clawOpen}    // 阶段8：回中
};

// ============================================================
// 任务三：循环搬运
// ============================================================
int currentTarget = 0;   // 当前搬运目标：0=A, 1=B, 2=C

// ============================================================
// 任务三：录制功能
// ============================================================
const int MAX_RECORD = 500;       // 最多录制500个采样点
const int RECORD_INTERVAL = 50;   // 录制间隔(ms)，每50ms记录一次
int recordData[MAX_RECORD][4];    // 录制数据：{底座, 大臂, 小臂, 夹爪}
int recordCount = 0;              // 已录制的点数
bool isRecording = false;         // 是否正在录制
unsigned long lastRecordTime = 0; // 上一次记录的时间

// ============================================================
// 任务三：播放功能
// ============================================================
bool isPlaying = false;           // 是否正在播放
int playIndex = 0;                // 当前播放到第几个点
unsigned long lastPlayTime = 0;   // 上一次播放的时间


// ============================================================
// 初始化
// ============================================================
void setup() {
  // 初始化舵机
  base.attach(pinBase);
  rArm.attach(pinRArm);
  fArm.attach(pinFArm);
  claw.attach(pinClaw);

  // 初始化到中间位置
  base.write(90);
  rArm.write(90);
  fArm.write(90);
  claw.write(clawOpen);

  // 硬件串口（连电脑调试）
  Serial.begin(9600);
  Serial.println("===============================");
  Serial.println("  meArm Robot Arm - Ready");
  Serial.println("===============================");

  // 软串口（和按键板通信）
  armSerial.begin(9600);
  Serial.println("Soft Serial: RX=8, TX=9");

  Serial.println("Commands:");
  Serial.println("  b+num = base angle");
  Serial.println("  r+num = rArm angle");
  Serial.println("  f+num = fArm angle");
  Serial.println("  c+num = claw angle");
  Serial.println("  x,y,z = sync move");
  Serial.println("  A/B/C = pick and place");
  Serial.println("  o = report status");
  Serial.println("  h = speed up");
  Serial.println("  l = speed down");
  Serial.println("Remote: 1=NextPick 2=Record 3=Play 4=Home");
  Serial.println("-------------------------------");
}


// ============================================================
// 单个舵机移动（带平滑过渡）
//   servoType: 'b'底座, 'r'大臂, 'f'小臂, 'c'夹爪
//   target: 目标角度
//   delayMS: 每步延时(ms)
// ============================================================
void servoCmd(char servoType, int target, int delayMS) {
  int fromPos;
  int minAngle, maxAngle;

  // 根据舵机类型获取当前角度和角度范围
  switch (servoType) {
    case 'b':
      fromPos = base.read();
      minAngle = 0;
      maxAngle = 180;
      break;
    case 'r':
      fromPos = rArm.read();
      minAngle = 0;
      maxAngle = 180;
      break;
    case 'f':
      fromPos = fArm.read();
      minAngle = 0;
      maxAngle = 180;
      break;
    case 'c':
      fromPos = claw.read();
      minAngle = 0;
      maxAngle = 180;
      break;
    default:
      Serial.println("Unknown servo");
      return;
  }

  // 范围检查
  if (target < minAngle) target = minAngle;
  if (target > maxAngle) target = maxAngle;

  // 平滑移动
  if (fromPos < target) {
    for (int i = fromPos; i <= target; i++) {
      switch (servoType) {
        case 'b': base.write(i); break;
        case 'r': rArm.write(i); break;
        case 'f': fArm.write(i); break;
        case 'c': claw.write(i); break;
      }
      delay(delayMS);
    }
  } else {
    for (int i = fromPos; i >= target; i--) {
      switch (servoType) {
        case 'b': base.write(i); break;
        case 'r': rArm.write(i); break;
        case 'f': fArm.write(i); break;
        case 'c': claw.write(i); break;
      }
      delay(delayMS);
    }
  }
}


// ============================================================
// 多舵机协同控制（x,y,z 三个舵机同时移动，同时到达）
//   xAngle: 底座目标角度
//   yAngle: 大臂目标角度
//   zAngle: 小臂目标角度
//   delayMS: 每步延时(ms)
// ============================================================
void servoSyncCmd(int xAngle, int yAngle, int zAngle, int delayMS) {
  int baseFrom = base.read();
  int rArmFrom = rArm.read();
  int fArmFrom = fArm.read();

  // 范围检查
  if (xAngle < 0) xAngle = 0;
  if (xAngle > 180) xAngle = 180;
  if (yAngle < 0) yAngle = 0;
  if (yAngle > 180) yAngle = 180;
  if (zAngle < 0) zAngle = 0;
  if (zAngle > 180) zAngle = 180;

  // 计算每个舵机要走多少度
  int baseSteps = abs(xAngle - baseFrom);
  int rArmSteps = abs(yAngle - rArmFrom);
  int fArmSteps = abs(zAngle - fArmFrom);

  // 取最大步数作为总步数（保证同时到达）
  int maxSteps = baseSteps;
  if (rArmSteps > maxSteps) maxSteps = rArmSteps;
  if (fArmSteps > maxSteps) maxSteps = fArmSteps;

  if (maxSteps == 0) {
    Serial.println("Already at target");
    return;
  }

  Serial.print("Sync: base ");
  Serial.print(baseFrom);
  Serial.print("->");
  Serial.print(xAngle);
  Serial.print(", rArm ");
  Serial.print(rArmFrom);
  Serial.print("->");
  Serial.print(yAngle);
  Serial.print(", fArm ");
  Serial.print(fArmFrom);
  Serial.print("->");
  Serial.print(zAngle);
  Serial.print(" (");
  Serial.print(maxSteps);
  Serial.println(" steps)");

  // 逐步移动，三个舵机按比例同时走
  for (int i = 0; i <= maxSteps; i++) {
    int basePos, rArmPos, fArmPos;

    // 用线性插值计算每一步的位置
    if (baseSteps == 0) {
      basePos = baseFrom;
    } else {
      basePos = baseFrom + (xAngle - baseFrom) * i / maxSteps;
    }

    if (rArmSteps == 0) {
      rArmPos = rArmFrom;
    } else {
      rArmPos = rArmFrom + (yAngle - rArmFrom) * i / maxSteps;
    }

    if (fArmSteps == 0) {
      fArmPos = fArmFrom;
    } else {
      fArmPos = fArmFrom + (zAngle - fArmFrom) * i / maxSteps;
    }

    base.write(basePos);
    rArm.write(rArmPos);
    fArm.write(fArmPos);

    delay(delayMS);
  }
}


// ============================================================
// 任务二：执行搬运动作
//   action: 动作序列数组（8个阶段 × 4个舵机角度）
//   name: 物体名称（调试显示用）
// ============================================================
void pickAndPlace(int action[8][4], const char* name) {
  Serial.print("===== Pick & Place ");
  Serial.print(name);
  Serial.println(" =====");

  for (int step = 0; step < 8; step++) {
    Serial.print("Step ");
    Serial.print(step + 1);
    Serial.print("/8: ");
    Serial.print("base=");
    Serial.print(action[step][0]);
    Serial.print(" rArm=");
    Serial.print(action[step][1]);
    Serial.print(" fArm=");
    Serial.print(action[step][2]);
    Serial.print(" claw=");
    Serial.println(action[step][3]);

    // x,y,z 三个舵机同步移动
    servoSyncCmd(action[step][0], action[step][1], action[step][2], DSD);
    // 夹爪单独动
    servoCmd('c', action[step][3], DSD);

    delay(200);  // 每个阶段停一下，稳一稳
  }

  Serial.print("===== ");
  Serial.print(name);
  Serial.println(" done =====");
}


// ============================================================
// 串口指令解析
// ============================================================
void armDataCmd(char serialCmd) {
  static int angleValue = 0;   // 静态变量，保存正在输入的数字
  static int syncX = 0;        // 同步移动的x值
  static int syncY = 0;        // 同步移动的y值
  static int parseStep = 0;    // 解析步骤：0=单舵机, 1=x, 2=y, 3=z

  // 如果是数字，累加角度值
  if (serialCmd >= '0' && serialCmd <= '9') {
    angleValue = angleValue * 10 + (serialCmd - '0');
    return;
  }

  // 如果是字母或符号，处理指令
  switch (serialCmd) {
    case 'b':  // 底座
      angleValue = 0;
      parseStep = 0;
      // 等数字输入完，在后续的非数字字符里执行
      break;

    case 'r':  // 大臂
      angleValue = 0;
      parseStep = 0;
      break;

    case 'f':  // 小臂
      angleValue = 0;
      parseStep = 0;
      break;

    case 'c':  // 夹爪
      angleValue = 0;
      parseStep = 0;
      break;

    case 'x':  // 同步移动 - x
      syncX = 0;
      angleValue = 0;
      parseStep = 1;
      break;

    case 'y':  // 同步移动 - y
      if (parseStep == 1) {
        syncX = angleValue;
        angleValue = 0;
        parseStep = 2;
      }
      break;

    case 'z':  // 同步移动 - z
      if (parseStep == 2) {
        syncY = angleValue;
        angleValue = 0;
        parseStep = 3;
      }
      break;

    case ',':  // 逗号分隔符
      if (parseStep == 1) {
        syncX = angleValue;
        angleValue = 0;
      } else if (parseStep == 2) {
        syncY = angleValue;
        angleValue = 0;
      }
      break;

    case 'A':  // 搬运A物体
      pickAndPlace(actionA, "A");
      angleValue = 0;
      break;

    case 'B':  // 搬运B物体
      pickAndPlace(actionB, "B");
      angleValue = 0;
      break;

    case 'C':  // 搬运C物体
      pickAndPlace(actionC, "C");
      angleValue = 0;
      break;

    case 'o':  // 查看状态
      reportStatus();
      angleValue = 0;
      break;

    case 'h':  // 加速
      if (DSD > 1) DSD--;
      Serial.print("Speed up, DSD = ");
      Serial.println(DSD);
      angleValue = 0;
      break;

    case 'l':  // 减速
      DSD++;
      Serial.print("Speed down, DSD = ");
      Serial.println(DSD);
      angleValue = 0;
      break;

    case '\n':
    case '\r':
    case ' ':
      // 换行、回车、空格：执行之前积累的指令
      if (parseStep == 3) {
        // 同步移动指令 x,y,z
        int syncZ = angleValue;
        Serial.print("Sync move to: ");
        Serial.print(syncX);
        Serial.print(", ");
        Serial.print(syncY);
        Serial.print(", ");
        Serial.println(syncZ);
        servoSyncCmd(syncX, syncY, syncZ, DSD);
        parseStep = 0;
      } else if (parseStep == 0 && angleValue > 0) {
        // 单舵机指令（但还不知道是哪个舵机，这里只是收尾）
        // 实际执行在下面的 default 里通过检测上一条指令
      }
      angleValue = 0;
      break;

    default:
      // 未知字符，忽略
      angleValue = 0;
      break;
  }

  // 处理单舵机指令（b/r/f/c 后面跟数字，遇到非数字时执行）
  // 这里用一个技巧：记录上一条单舵机指令
  static char lastSingleCmd = 0;
  if (serialCmd == 'b' || serialCmd == 'r' || serialCmd == 'f' || serialCmd == 'c') {
    // 如果之前有未执行的单舵机指令，先执行
    if (lastSingleCmd != 0 && angleValue > 0) {
      // 这种情况一般不会发生
    }
    lastSingleCmd = serialCmd;
  } else if (lastSingleCmd != 0 && (serialCmd < '0' || serialCmd > '9') && angleValue > 0) {
    // 数字输入结束，执行单舵机移动
    switch (lastSingleCmd) {
      case 'b':
        Serial.print("Base -> ");
        Serial.println(angleValue);
        servoCmd('b', angleValue, DSD);
        break;
      case 'r':
        Serial.print("RArm -> ");
        Serial.println(angleValue);
        servoCmd('r', angleValue, DSD);
        break;
      case 'f':
        Serial.print("FArm -> ");
        Serial.println(angleValue);
        servoCmd('f', angleValue, DSD);
        break;
      case 'c':
        Serial.print("Claw -> ");
        Serial.println(angleValue);
        servoCmd('c', angleValue, DSD);
        break;
    }
    lastSingleCmd = 0;
    angleValue = 0;
  }
}


// ============================================================
// 报告当前各舵机角度
// ============================================================
void reportStatus() {
  Serial.println("------- Status -------");
  Serial.print("Base: ");
  Serial.print(base.read());
  Serial.print(" deg | ");
  Serial.print("RArm: ");
  Serial.print(rArm.read());
  Serial.print(" deg | ");
  Serial.print("FArm: ");
  Serial.print(fArm.read());
  Serial.print(" deg | ");
  Serial.print("Claw: ");
  Serial.print(claw.read());
  Serial.println(" deg");
  Serial.print("DSD: ");
  Serial.println(DSD);
  Serial.println("----------------------");
}


// ============================================================
// 摇杆控制：底座（左摇杆左右）
// ============================================================
void joyBaseControl() {
  int joyVal = analogRead(joyL_X);
  int currentPos = base.read();

  if (joyVal > 512 + joyDeadZone) {
    // 向右推 → 底座右转
    if (currentPos < 180 - joyStep) {
      base.write(currentPos + joyStep);
      delay(DSD);
    }
  } else if (joyVal < 512 - joyDeadZone) {
    // 向左推 → 底座左转
    if (currentPos > joyStep) {
      base.write(currentPos - joyStep);
      delay(DSD);
    }
  }
}


// ============================================================
// 摇杆控制：大臂（左摇杆前后）
// ============================================================
void joyRArmControl() {
  int joyVal = analogRead(joyL_Y);
  int currentPos = rArm.read();

  if (joyVal > 512 + joyDeadZone) {
    // 向前推 → 大臂落下
    if (currentPos < 180 - joyStep) {
      rArm.write(currentPos + joyStep);
      delay(DSD);
    }
  } else if (joyVal < 512 - joyDeadZone) {
    // 向后拉 → 大臂抬起
    if (currentPos > joyStep) {
      rArm.write(currentPos - joyStep);
      delay(DSD);
    }
  }
}


// ============================================================
// 摇杆控制：小臂（右摇杆左右）
// ============================================================
void joyFArmControl() {
  int joyVal = analogRead(joyR_X);
  int currentPos = fArm.read();

  if (joyVal > 512 + joyDeadZone) {
    // 向右推 → 小臂伸出
    if (currentPos < 180 - joyStep) {
      fArm.write(currentPos + joyStep);
      delay(DSD);
    }
  } else if (joyVal < 512 - joyDeadZone) {
    // 向左推 → 小臂收回
    if (currentPos > joyStep) {
      fArm.write(currentPos - joyStep);
      delay(DSD);
    }
  }
}


// ============================================================
// 摇杆控制：夹爪（右摇杆前后）
// ============================================================
void joyClawControl() {
  int joyVal = analogRead(joyR_Y);
  int currentPos = claw.read();

  if (joyVal > 512 + joyDeadZone) {
    // 向前推 → 夹爪张开
    if (currentPos < 180 - joyStep) {
      claw.write(currentPos + joyStep);
      delay(DSD);
    }
  } else if (joyVal < 512 - joyDeadZone) {
    // 向后拉 → 夹爪闭合
    if (currentPos > joyStep) {
      claw.write(currentPos - joyStep);
      delay(DSD);
    }
  }
}


// ============================================================
// 摇杆总控制
// ============================================================
void joystickControl() {
  joyBaseControl();
  joyRArmControl();
  joyFArmControl();
  joyClawControl();
}


// ============================================================
// 任务三：循环搬运（按键1）
// ============================================================
void nextPickAndPlace() {
  if (isRecording || isPlaying) {
    Serial.println("Busy, skip command");
    return;
  }
  currentTarget = (currentTarget + 1) % 3;  // 0→1→2→0循环
  Serial.print("Next target: ");
  if (currentTarget == 0) {
    Serial.println("A");
    pickAndPlace(actionA, "A");
  } else if (currentTarget == 1) {
    Serial.println("B");
    pickAndPlace(actionB, "B");
  } else {
    Serial.println("C");
    pickAndPlace(actionC, "C");
  }
}


// ============================================================
// 任务三：开始录制（按键2第一次按）
// ============================================================
void startRecord() {
  if (isPlaying) {
    Serial.println("Playing, can't record");
    return;
  }
  isRecording = true;
  recordCount = 0;
  lastRecordTime = millis();
  Serial.println("===== Recording Start =====");
  Serial.println("Use joystick to control the arm");
}


// ============================================================
// 任务三：停止录制（按键2第二次按）
// ============================================================
void stopRecord() {
  isRecording = false;
  Serial.print("===== Recording End, total ");
  Serial.print(recordCount);
  Serial.println(" points =====");
  // 计算时长
  int totalSeconds = (recordCount * RECORD_INTERVAL) / 1000;
  Serial.print("Duration: ");
  Serial.print(totalSeconds);
  Serial.println(" seconds");
  if (totalSeconds < 10) {
    Serial.println("Warning: Less than 10 seconds!");
  }
}


// ============================================================
// 任务三：录制采样（在 loop 里循环调用）
// ============================================================
void recordUpdate() {
  if (!isRecording) return;
  if (recordCount >= MAX_RECORD) {
    stopRecord();  // 录满了自动停止
    return;
  }
  if (millis() - lastRecordTime >= RECORD_INTERVAL) {
    // 记录当前4个舵机的角度
    recordData[recordCount][0] = base.read();
    recordData[recordCount][1] = rArm.read();
    recordData[recordCount][2] = fArm.read();
    recordData[recordCount][3] = claw.read();
    recordCount++;
    lastRecordTime = millis();
  }
}


// ============================================================
// 任务三：开始播放（按键3）
// ============================================================
void startPlay() {
  if (isRecording) {
    Serial.println("Recording, can't play");
    return;
  }
  if (recordCount == 0) {
    Serial.println("No recording data!");
    return;
  }
  isPlaying = true;
  playIndex = 0;
  lastPlayTime = millis();
  Serial.println("===== Play Start =====");
}


// ============================================================
// 任务三：播放更新（在 loop 里循环调用）
// ============================================================
void playUpdate() {
  if (!isPlaying) return;
  if (playIndex >= recordCount) {
    isPlaying = false;
    Serial.println("===== Play End =====");
    return;
  }
  if (millis() - lastPlayTime >= RECORD_INTERVAL) {
    // 从录制数据里读出角度，直接设置舵机
    base.write(recordData[playIndex][0]);
    rArm.write(recordData[playIndex][1]);
    fArm.write(recordData[playIndex][2]);
    claw.write(recordData[playIndex][3]);
    playIndex++;
    lastPlayTime = millis();
  }
}


// ============================================================
// 任务三：回中（按键4）
// ============================================================
void goHome() {
  if (isRecording || isPlaying) {
    Serial.println("Busy, skip command");
    return;
  }
  Serial.println("Going home...");
  // 三个舵机同步回中
  servoSyncCmd(90, 90, 90, DSD);
  // 夹爪张开
  servoCmd('c', clawOpen, DSD);
  Serial.println("Home position reached");
}


// ============================================================
// 任务三：处理按键板发来的指令
// ============================================================
void processRemoteCmd() {
  if (armSerial.available() > 0) {
    char cmd = armSerial.read();
    Serial.print("Remote cmd: ");
    Serial.println(cmd);

    switch (cmd) {
      case '1':  // 按键1：循环搬运
        nextPickAndPlace();
        break;
      case '2':  // 按键2：录制开关
        if (!isRecording) {
          startRecord();
        } else {
          stopRecord();
        }
        break;
      case '3':  // 按键3：播放
        startPlay();
        break;
      case '4':  // 按键4：回中
        goHome();
        break;
      default:
        Serial.println("Unknown remote command");
        break;
    }
  }
}


// ============================================================
// 主循环
// ============================================================
void loop() {
  // 1. 处理电脑串口指令
  if (Serial.available() > 0) {
    char serialCmd = Serial.read();
    armDataCmd(serialCmd);
  }

  // 2. 处理按键板遥控指令
  processRemoteCmd();

  // 3. 录制采样（如果正在录制）
  recordUpdate();

  // 4. 播放更新（如果正在播放）
  playUpdate();

  // 5. 摇杆控制（播放时不响应摇杆，避免干扰）
  if (!isPlaying) {
    joystickControl();
  }
}
