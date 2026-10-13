/*
  ============================================================
  任务三：按键遥控板
  ============================================================
  功能：检测4个按键，通过软串口向机械臂发送指令
  按键1 → 发 '1' → 循环搬运 A/B/C
  按键2 → 发 '2' → 开始/结束录制
  按键3 → 发 '3' → 播放录制的动作
  按键4 → 发 '4' → 回中
  ============================================================
  接线说明：
  - 按键一端接对应引脚，另一端接GND（因为用了上拉电阻）
  - 软串口 TX(10) → 接机械臂的软串口 RX(8)
  - 软串口 RX(11) → 接机械臂的软串口 TX(9)
  - 两块板共地（GND连在一起）
  ============================================================
*/

#include <SoftwareSerial.h>

// ====== 按键引脚定义（根据实际接线修改） ======
const int key1Pin = 2;   // 按键1：循环搬运
const int key2Pin = 3;   // 按键2：录制开关
const int key3Pin = 4;   // 按键3：播放录制
const int key4Pin = 5;   // 按键4：回中

// ====== 软串口引脚（和机械臂交叉连接） ======
const int softTX = 10;   // 发送 → 接机械臂的 softRX(8)
const int softRX = 11;   // 接收 ← 接机械臂的 softTX(9)

SoftwareSerial mySerial(softRX, softTX);  // RX, TX

// ====== 按键消抖参数 ======
const int debounceTime = 50;   // 消抖延时(ms)

// 按键状态变量
bool key1Last = HIGH;
bool key2Last = HIGH;
bool key3Last = HIGH;
bool key4Last = HIGH;
unsigned long lastDebounce1 = 0;
unsigned long lastDebounce2 = 0;
unsigned long lastDebounce3 = 0;
unsigned long lastDebounce4 = 0;

// ============================================================
// 初始化
// ============================================================
void setup() {
  // 硬件串口（用于调试，连电脑）
  Serial.begin(9600);
  Serial.println("===============================");
  Serial.println("  Key Remote Board - Ready");
  Serial.println("===============================");

  // 软串口（和机械臂通信）
  mySerial.begin(9600);

  // 设置按键引脚为输入上拉模式
  pinMode(key1Pin, INPUT_PULLUP);
  pinMode(key2Pin, INPUT_PULLUP);
  pinMode(key3Pin, INPUT_PULLUP);
  pinMode(key4Pin, INPUT_PULLUP);

  Serial.println("Key pins: 2, 3, 4, 5");
  Serial.println("Soft Serial: TX=10, RX=11");
  Serial.println("Commands: 1=NextPick, 2=Record, 3=Play, 4=Home");
  Serial.println("-------------------------------");
}

// ============================================================
// 按键检测函数（带消抖）
//   pin: 引脚号
//   lastState: 上一次稳定状态（引用变量，会修改原值）
//   lastTime: 上一次状态变化时间（引用变量）
//   cmd: 按下时发送的指令字符
// ============================================================
void checkKey(int pin, bool &lastState, unsigned long &lastTime, char cmd) {
  bool reading = digitalRead(pin);

  // 状态变化时，重置消抖计时
  if (reading != lastState) {
    lastTime = millis();
  }

  // 状态稳定超过消抖时间，才认为是真实的按键动作
  if ((millis() - lastTime) > debounceTime) {
    // 如果按键按下（低电平，因为上拉电阻）
    if (reading == LOW && lastState == HIGH) {
      // 通过软串口发送指令给机械臂
      mySerial.print(cmd);
      // 同时在电脑串口打印调试信息
      Serial.print("Send command: ");
      Serial.println(cmd);
    }
    // 更新上一次状态
    lastState = reading;
  }
}

// ============================================================
// 主循环
// ============================================================
void loop() {
  // 检测四个按键
  checkKey(key1Pin, key1Last, lastDebounce1, '1');
  checkKey(key2Pin, key2Last, lastDebounce2, '2');
  checkKey(key3Pin, key3Last, lastDebounce3, '3');
  checkKey(key4Pin, key4Last, lastDebounce4, '4');
}
