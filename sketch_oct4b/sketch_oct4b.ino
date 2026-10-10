#include <Servo.h>     //使用Servo库
Servo base,rArm,fArm,claw;  //创建4个舵机 

//存储极限值
const int baseMin=0;
const int baseMax=180;
const int rArmMin=0;
const int rArmMax=180;
const int fArmMin=0;
const int fArmMax=180;
const int clawMin=0;
const int clawMax=180

int DSD=15;   //默认运动速度，调节运行速度






void setup() {
 base.attach(2);//加引脚
 delay(200);
 rArm.attach(3);
 delay(200);
 fArm.attach(4);
 delay(200);
 claw.attach(5);
 delay(200);

 base.write(90);//初始化
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

void servoCmd(char servoName,int toPos,int servoDelay){
  Servo servo2go;  //函数内部舵机

  //串口
  Serial.println("");
  Serial.print("+Command:Servo");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".")
  Serial.println("");  //确认指令信息

  int fromPos;

  switch(servoName){
    case'b':
      if(toPos>=baseMin&&toPos<=baseMax){
        servo2go=base;
        fromPos=base.read();
        break;
      }else{
        Serial.println("+Warning:Base Servo Value Out Of Limt!");
        return;
      }

    case'c':
      if(toPos>=clawMin&&toPos<=clawMax){
        servo2go=claw;
        fromPos=claw.read();
        break;
      }else{
        Serial.println("+Warning:Claw Servo Value Out Of Limt!");
        return;
      }

    case'f':
      if(toPos>=fArmMin&&toPos<=fArmMax){
        servo2go=fArm;
        fromPos=fArm.read();
        break;
      }else{
        Serial.println("+Warning:fArm Servo Value Out Of Limt!");
        return;
      }

      case'r':
      if(toPos>=rArmMin&&toPos<=rArmMax){
        servo2go=rArm;
        fromPos=rArm.read();
        break;
      }else{
        Serial.println("+Warning:rArm Servo Value Out Of Limt!");
        return;
      }
  }  
   //舵机运行
   if(fromPos<=toPos){//小了
    for(int i=fromPos;i<=toPos;i++){
      servo2go.write(i);
      delay(servoDelay);
    }
   }else{//大了
    for(int i=fromPos;i>=toPos;i--){
      servo2go.write(i);
      delay(servoDelay);
    }
   }


  



}





void armDataCmd(char serialCmd){      //运动指令
  if(serialCmd=='b'||serialCmd=='c'||serialCmd=='f'||serialCmd=='r'){
    int servoData=Serial.parseInt();
    servoCmd(serialCmd,servoData,DSD);//调用另一个函数
  }else{
    switch(serialCmd){
       case'o':  //输出此时舵机状态
       reportStatus();
       break;
       default:  //未知指令
        Serial.println()
    }

  }
}










void loop() {
  if(Serial.available()>0){
    char serialCmd=Serial.read();
    armDataCmd(serialCmd);
  }

}











