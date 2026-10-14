/*
 * 任务五 - 直线绘制（基础版）
 * 功能：逆运动学 + 直线插补 + 抬笔落笔
 * 串口指令：
 *   L x1,y1,x2,y2   画直线，例如：L 30,20,80,60
 *   U               抬笔
 *   D               落笔
 *   o               查看当前坐标和舵机角度
 */

#include <Servo.h>
#include <math.h>

// ====== 舵机引脚定义 ======
const int pinBase = 2;
const int pinRArm = 3;
const int pinFArm = 4;
const int pinClaw = 5;

Servo base;
Servo rArm;
Servo fArm;
Servo claw;

// ====== 机械臂参数（需要实际测量和校准！）======
const float ARM_L1 = 80.0;     // 大臂长度 mm
const float ARM_L2 = 80.0;     // 小臂长度 mm
const float ARM_L3 = 35.0;     // 夹爪长度 mm
const float BASE_H = 60.0;     // 底座高度 mm

// 舵机角度偏移：舵机多少度对应数学角度的0度
const float baseOffset = 90.0;   // 底座舵机90度=数学0度
const float rArmOffset = 90.0;   // 大臂舵机90度=数学0度
const float fArmOffset = 90.0;   // 小臂舵机90度=数学0度

// 舵机方向：1=舵机增大=数学角度增大，-1=相反
const int baseDir = 1;
const int rArmDir = 1;
const int fArmDir = -1;  // 小臂通常方向相反，需要实际测试

// ====== 绘图参数 ======
const float drawZ = 50.0;       // 落笔高度 mm（z坐标）
const float penUpOffset = 20.0;  // 抬笔时z升高多少 mm
const int lineSteps = 50;        // 直线插补步数（越多越平滑）
const int moveDelay = 20;        // 每步延时 ms

// ====== 全局变量 ======
float currentX = 50.0;   // 当前末端x坐标
float currentY = 0.0;    // 当前末端y坐标
float currentZ = 80.0;   // 当前末端z坐标
bool isPenUp = false;    // 笔是否抬起

// ====== 数学角度转舵机角度 ======
float math2servo(float mathAng, float offset, int dir) {
  float deg = mathAng * 180.0 / PI;
  float servoAng = offset + dir * deg;
  if (servoAng < 0) servoAng = 0;
  if (servoAng > 180) servoAng = 180;
  return servoAng;
}

// ====== 逆运动学 ======
// 输入：目标坐标(x,y,z)
// 输出：三个舵机角度（通过引用返回）
// 返回：true=成功，false=不可达
bool inverseKinematics(float x, float y, float z, float &baseAng, float &rArmAng, float &fArmAng) {
  // 第一步：算底座角度
  float theta1 = atan2(y, x);

  // 第二步：算水平面内距离 r 和高度差 h
  float r = sqrt(x * x + y * y) - ARM_L3;  // 减去夹爪长度
  float h = z - BASE_H;

  if (r < 0) r = 0;

  // 第三步：算肩关节到腕关节的直线距离 D
  float D = sqrt(r * r + h * h);

  // 检查可达性
  if (D > ARM_L1 + ARM_L2) return false;
  if (D < fabs(ARM_L1 - ARM_L2)) return false;

  // 第四步：余弦定理算大臂与直线D的夹角
  float cosAlpha = (ARM_L1 * ARM_L1 + D * D - ARM_L2 * ARM_L2) / (2 * ARM_L1 * D);
  if (cosAlpha > 1) cosAlpha = 1;
  if (cosAlpha < -1) cosAlpha = -1;
  float alpha = acos(cosAlpha);

  // 第五步：算D的仰角
  float phi = atan2(h, r);

  // 第六步：算大臂角度（肘上解）
  float theta2 = phi + alpha;

  // 第七步：算肘关节内角
  float cosBeta = (ARM_L1 * ARM_L1 + ARM_L2 * ARM_L2 - D * D) / (2 * ARM_L1 * ARM_L2);
  if (cosBeta > 1) cosBeta = 1;
  if (cosBeta < -1) cosBeta = -1;
  float beta = acos(cosBeta);

  // 第八步：小臂方向角 = 大臂方向角 - (PI - beta)
  // 小臂相对大臂向下弯，所以方向角比大臂小
  float thetaForearm = theta2 - (PI - beta);

  // 第九步：数学角度转舵机角度
  baseAng = math2servo(theta1, baseOffset, baseDir);
  rArmAng = math2servo(theta2, rArmOffset, rArmDir);
  fArmAng = math2servo(thetaForearm, fArmOffset, fArmDir);

  // 检查舵机角度范围
  if (baseAng < 0 || baseAng > 180) return false;
  if (rArmAng < 0 || rArmAng > 180) return false;
  if (fArmAng < 0 || fArmAng > 180) return false;

  return true;
}

// ====== 移动到指定坐标 ======
bool moveTo(float x, float y, float z) {
  float b, r, f;
  if (!inverseKinematics(x, y, z, b, r, f)) {
    return false;
  }

  // 平滑移动
  int steps = 20;
  float b0 = base.read();
  float r0 = rArm.read();
  float f0 = fArm.read();

  for (int i = 1; i <= steps; i++) {
    float tb = b0 + (b - b0) * i / steps;
    float tr = r0 + (r - r0) * i / steps;
    float tf = f0 + (f - f0) * i / steps;
    base.write(tb);
    rArm.write(tr);
    fArm.write(tf);
    delay(moveDelay);
  }

  currentX = x;
  currentY = y;
  currentZ = z;
  return true;
}

// ====== 画直线 ======
void drawLine(float x1, float y1, float x2, float y2) {
  float z = isPenUp ? (drawZ + penUpOffset) : drawZ;

  for (int i = 0; i <= lineSteps; i++) {
    float t = (float)i / lineSteps;
    float x = x1 + (x2 - x1) * t;
    float y = y1 + (y2 - y1) * t;

    float b, r, f;
    if (inverseKinematics(x, y, z, b, r, f)) {
      base.write(b);
      rArm.write(r);
      fArm.write(f);
      delay(moveDelay);
    } else {
      Serial.print("不可达点：(");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.println(")");
      return;
    }
  }

  currentX = x2;
  currentY = y2;
  currentZ = z;
}

// ====== 抬笔 ======
void penUp() {
  if (isPenUp) return;
  float newZ = drawZ + penUpOffset;
  moveTo(currentX, currentY, newZ);
  isPenUp = true;
  Serial.println("抬笔");
}

// ====== 落笔 ======
void penDown() {
  if (!isPenUp) return;
  moveTo(currentX, currentY, drawZ);
  isPenUp = false;
  Serial.println("落笔");
}

// ====== 显示状态 ======
void reportStatus() {
  Serial.println("====== 当前状态 ======");
  Serial.print("坐标：(");
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.print(", ");
  Serial.print(currentZ);
  Serial.println(")");
  Serial.print("底座："); Serial.print(base.read()); Serial.print("度  ");
  Serial.print("大臂："); Serial.print(rArm.read()); Serial.print("度  ");
  Serial.print("小臂："); Serial.print(fArm.read()); Serial.print("度  ");
  Serial.print("夹爪："); Serial.print(claw.read()); Serial.println("度");
  Serial.print("笔状态："); Serial.println(isPenUp ? "抬起" : "落下");
  Serial.println("======================");
}

// ====== 解析并执行指令 ======
void execCmd(char cmd, char* param) {
  if (cmd == 'L') {
    // 画直线：L x1,y1,x2,y2
    float x1, y1, x2, y2;
    char* p = param;
    x1 = atof(p);
    p = strchr(p, ','); if (p) p++; else return;
    y1 = atof(p);
    p = strchr(p, ','); if (p) p++; else return;
    x2 = atof(p);
    p = strchr(p, ','); if (p) p++; else return;
    y2 = atof(p);

    Serial.print("画直线：(");
    Serial.print(x1); Serial.print(","); Serial.print(y1);
    Serial.print(") -> (");
    Serial.print(x2); Serial.print(","); Serial.print(y2);
    Serial.println(")");

    drawLine(x1, y1, x2, y2);
    Serial.println("完成");
  }
  else if (cmd == 'U') {
    penUp();
  }
  else if (cmd == 'D') {
    penDown();
  }
  else if (cmd == 'o') {
    reportStatus();
  }
}

// ====== 串口读取 ======
void serialRead() {
  static char buf[64];     // 存一行指令的"小本子"
  static int idx = 0;      // 写到第几格了
  static char cmdChar = 0; // 指令字母（L/U/D/o）

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      // 收到回车 = 一句话说完了
      if (idx > 0 || cmdChar != 0) {
        buf[idx] = '\0';
        execCmd(cmdChar, buf);  // 执行指令
        idx = 0;                // 清空小本子
        cmdChar = 0;            // 清空指令
      }
    }
    else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
      // 收到字母 = 新指令的开头
      cmdChar = c;
      idx = 0;  // 小本子翻到第一页
    }
    else {
      // 收到数字、逗号、空格等 = 指令的内容
      if (idx < 63) {
        buf[idx] = c;
        idx++;
      }
    }
  }
}

// ====== setup ======
void setup() {
  base.attach(pinBase);
  rArm.attach(pinRArm);
  fArm.attach(pinFArm);
  claw.attach(pinClaw);

  // 初始位置
  base.write(baseOffset);
  rArm.write(rArmOffset);
  fArm.write(fArmOffset);
  claw.write(90);

  Serial.begin(9600);
  Serial.println("机械臂绘图 - 直线绘制版");
  Serial.println("指令：");
  Serial.println("  L x1,y1,x2,y2   画直线");
  Serial.println("  U               抬笔");
  Serial.println("  D               落笔");
  Serial.println("  o               查看状态");
}

// ====== loop ======
void loop() {
  serialRead();
}
