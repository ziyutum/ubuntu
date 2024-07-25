#include <TimerOne.h>

int sensorPin = 2; // 传感器连接到数字引脚 2
int sensorValue = 0; // 传感器读取的值

void setup() {
  pinMode(sensorPin, INPUT); // 设置传感器引脚为输入
  Serial.begin(9600); // 初始化硬件串口

  // 初始化 Timer1
  Timer1.initialize(2000000); // 设置定时器每 2 秒触发一次 (2000000 微秒)
  Timer1.attachInterrupt(clearSerialBuffer); // 附加中断服务程序
}

void loop() {
  sensorValue = digitalRead(sensorPin); // 读取传感器值
  Serial.println(sensorValue); // 打印传感器值到串口
  delay(500); // 等待 500 毫秒
}

// 
void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read(); // 
  }
}