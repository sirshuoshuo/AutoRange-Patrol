#include <MsTimer2.h>         //定时中断头文件库
#include "PinChangeInterrupt.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// 初始I2C地址
#define INITIAL_I2C_ADDRESS1 0x30
// #define INITIAL_I2C_ADDRESS2 0x31

  // 创建两个VL53L0X对象
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

//超声波传感器引脚定义
int L_trigPin = 9; 
int L_echoPin = A3;
long duration_1;
float cm_1;
float measured_distance;
float measure_1;
/***********电机控制板引脚定义************/
/***右轮****/
unsigned int Motor_AIN1=6;        //控制电机的方向引脚  一定改成自己用的
unsigned int Motor_AIN2=3;        //控制电机的方向引脚  一定改成自己用的
/****左轮****/
unsigned int Motor_BIN1=5;
unsigned int Motor_BIN2=11;

/***巡线传感器***/
#define tracker_R1 A2
#define tracker_M A1
#define tracker_L1 A0
// #define tracker_L2 A3
// #define tracker_R2 10
int tracker_M_VAL;
int tracker_R1_VAL;
int tracker_L1_VAL;
// int tracker_R2_VAL;
// int tracker_L2_VAL;


String Target_Value_L;             //串口获取的速度字符串变量
String Target_Value_R;
int value_L;                       //用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 
int value_R;
/***********编码器引脚************/
/***右轮***/
#define ENCODER_A_R 2              //编码器A相引脚——————需要为中断引脚
#define ENCODER_B_R 7              //编码器B相引脚——————如果不4分频，可以不为中断引脚
/***左轮***/
#define ENCODER_A_L 4              //右轮编码器A相引脚——————需要为中断引脚
#define ENCODER_B_L 8              //右轮编码器B相引脚

int Velocity_L,Count_L=0;            //Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
int Velocity_R,Count_R=0;
/***********PID控制器相关参数************/
float Velocity_KP =10, Velocity_KI= 10;
//Velocity_KP,Velocity_KI.PI参数 
volatile float Target_L=0;//目标值
volatile float Target_R=0;
static float Bias_L,PWM_L=0,Last_bias_L=0;   
static float Bias_R,PWM_R=0,Last_bias_R=0;
/*********** 限幅************
*以下两个参数让输出的PWM在一个合理区间
*当输出的PWM小于50时电机不转 所以要设置一个启始PWM
*arduino mega 2560 单片机的PWM不能超过255 所以 PWM_Restrict 起到限制上限的作用
*****************************/
int startPWM=10;                 //初始PWM，暂时不用
int PWM_Restrict=245;            //startPW+PWM_Restric=255<256
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
/***********初始化************/
void setup() 
{
  Serial.begin(9600);            //打开串口
  while (!Serial); // 等待串口连接
  Wire.begin();
  delay(500);
  //  初始化传感器
  if (!lox1.begin(INITIAL_I2C_ADDRESS1)) {
    Serial.println(F("Failed to boot VL53L0X sensor 1"));
    while (1);
  }

// if (!lox2.begin(INITIAL_I2C_ADDRESS2)) {
//   Serial.println(F("Failed to boot VL53L0X sensor 2"));
//   while (1);
// }
 delay(500);
  
  // 初始化成功
  Serial.println("VL53L0X sensors ready");
  Target_L = 0;
  Target_R =0;
  pinMode(L_trigPin, OUTPUT);
  pinMode(L_echoPin, INPUT);
  //Serial.println("/*****START*****/");
  pinMode(ENCODER_A_L,INPUT);     //设置两个相线为输入模式
  pinMode(ENCODER_B_L,INPUT);
  pinMode(ENCODER_A_R,INPUT);
  pinMode(ENCODER_B_R,INPUT);
  //巡线传感器
  pinMode(tracker_R1,INPUT_PULLUP);
  pinMode(tracker_M,INPUT_PULLUP);
  pinMode(tracker_L1,INPUT_PULLUP);
  // pinMode(tracker_L2,INPUT_PULLUP);
  // pinMode(tracker_R2,INPUT_PULLUP);

  pinMode(Motor_AIN1,OUTPUT);   //设置两个驱动引脚为输出模式
  pinMode(Motor_AIN2,OUTPUT); 
  pinMode(Motor_BIN1,OUTPUT);
  pinMode(Motor_BIN2,OUTPUT);
  
MsTimer2::set(10, control); //10毫秒定时中断函数
MsTimer2::start ();        //中断使能 
attachPCINT(digitalPinToPCINT(ENCODER_A_L), READ_ENCODER_A_L,CHANGE);      //开启对应2号引脚的0号外部中断,触发方式为FALLING 即下降沿触发,触发的中断函数为 READ_ENCODER_A 
attachInterrupt(0,READ_ENCODER_A_R,CHANGE);

}
/***********主程序************/
void loop() 
{
  tracker_M_VAL = digitalRead(tracker_M);
  tracker_L1_VAL = digitalRead(tracker_L1);
  tracker_R1_VAL = digitalRead(tracker_R1);
  // tracker_L2_VAL = digitalRead(tracker_L2);
  // tracker_R2_VAL = digitalRead(tracker_R2);
  
  // Serial.print("tracker_L1:");
  // Serial.println(tracker_L1_VAL);
  // Serial.print("tracker_M:");
  // Serial.println(tracker_M_VAL);
  // Serial.print("tracker_R1:");
  // Serial.println(tracker_R1_VAL);
  measure();
 //直行
if(tracker_M_VAL==1 && tracker_L1_VAL==0 && tracker_R1_VAL==0)   //同时触碰到黑线(输出1)，亮灯是0，灭灯是1
{
  Target_L = -5;
  Target_R = -5;
}
//左转
else if(tracker_M_VAL==1 && tracker_L1_VAL==1 && tracker_R1_VAL==0)   //左侧巡线传感器接触到黑线
{
  Target_L = -6;
  Target_R = -4;
}
else if(tracker_M_VAL==0 && tracker_L1_VAL==1 && tracker_R1_VAL==0)
{
  Target_L = -7;
  Target_R = -3;
}
//右转
else if(tracker_M_VAL==1 && tracker_L1_VAL==0 && tracker_R1_VAL==1)   //右侧巡线传感器接触到黑线
{
  Target_L = -4;
  Target_R = -6;
}
else if(tracker_M_VAL==0 && tracker_L1_VAL==0 && tracker_R1_VAL==1)
{
  Target_L = -3;
  Target_R = -7;
}
//停止
else if(tracker_M_VAL==1 && tracker_L1_VAL==1 && tracker_R1_VAL==1)   //左右侧巡线传感器同时接触到黑线
{
  Target_L = 0;
  Target_R = 0;
  
}

// else if(tracker_L2_VAL==0 && tracker_L1_VAL==0 && tracker_M_VAL==0 && tracker_R1_VAL==0 && tracker_R2_VAL==1)
// {
//   Target_L = 0;
//   Target_R = 0;
//   measure();
// }

// else if(tracker_L2_VAL==1 && tracker_L1_VAL==0 && tracker_M_VAL==0 && tracker_R1_VAL==0 && tracker_R2_VAL==0)
// {
//   Target_L = 0;
//   Target_R = 0;
//   measure();
// }

// else if(tracker_L2_VAL==0 && tracker_L1_VAL==0 && tracker_M_VAL==0 && tracker_R1_VAL==0 && tracker_R2_VAL==0)
// {
//   Target_L = 0;
//   Target_R = 0;
//   measure();
// }

else if(tracker_M_VAL==0 && tracker_L1_VAL==0 && tracker_R1_VAL==0)   
 {
   Target_L = 0;
   Target_R = 0;
   //measure();
 }
else   //左右侧巡线传感器同时接触到黑线
  {
    while(1)
    {
    Target_L = 0;
    Target_R = 0;
    }
  }
}
/**********外部中断触发计数器函数************
*根据转速的方向不同我们将计数器累计为正值或者负值(计数器累计为正值为负值为计数器方向)
*只有方向累计正确了才可以实现正确的调整,否则会出现逆方向满速旋转
*
*※※※※※※超级重点※※※※※※
*
*所谓累计在正确的方向即
*(1)计数器方向
*(2)电机输出方向(控制电机转速方向的接线是正着接还是反着接)
*(3)PI 控制器 里面的误差(Basi)运算是目标值减当前值(Target-Encoder),还是当前值减目标值(Encoder-Target)
*三个方向只有对应上才会有效果否则你接上就是使劲的朝着一个方向(一般来说是反方向)满速旋转

例子里是已经对应好的,如果其他驱动单片机在自己尝试的时候出现满速旋转就是三个方向没对应上

下列函数中由于在A相上升沿触发时,B相是低电平,和A相下降沿触发时B是高电平是一个方向,在这种触发方式下,我们将count累计为正,另一种情况将count累计为负
********************************************/
void READ_ENCODER_A_L() 
{
  if (digitalRead(ENCODER_A_L) ==0) 
  {     
    if (digitalRead(ENCODER_B_L) == 1)      
      Count_L++;  //根据另外一相电平判定方向
    else      
      Count_L--;
  }
  
  else 
  {    
    if (digitalRead(ENCODER_B_L) == 1)      
    Count_L--; //根据另外一相电平判定方向
    else      
    Count_L++;
  }
  /*
  if (digitalRead(ENCODER_B_L)==0){
    Count--;
  }
  else{
    Count++;
  }
  */
}

void READ_ENCODER_A_R()
{
  if (digitalRead(ENCODER_A_R) ==0) 
  {     
    if (digitalRead(ENCODER_B_R) == LOW)      
      Count_R++;  //根据另外一相电平判定方向
    else      
      Count_R--;
  }
  
  else 
  {    
    if (digitalRead(ENCODER_B_R) == LOW)      
    Count_R--; //根据另外一相电平判定方向
    else      
    Count_R++;
  }
}
/**********定时器中断触发函数*********/
void control()
{     
  Velocity_L=Count_L;    //把采用周期(内部定时中断周期)所累计的脉冲下降沿的个数,赋值给速度
  Velocity_R=Count_R;
  Count_L=0;           //将脉冲计数器清零
  Count_R=0;
  value_L=Incremental_PI_A_L(Velocity_L,Target_L);  //通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
  value_R=Incremental_PI_A_R(Velocity_R,Target_R);
  Set_PWM_L(value_L);    //将算好的值输出给电机
  Set_PWM_R(value_R);
        
}
/***********PI控制器****************/
int Incremental_PI_A_L (int Encoder,float Target1)
{  
  // static float Bias,PWM=0,Last_bias=0;                  //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias_L=Target1-Encoder;                                   //计算偏差,目标值减去当前值
   PWM_L += Velocity_KP*(Bias_L-Last_bias_L)+Velocity_KI*Bias_L;   //增量式PI控制计算
   //Serial.print(PWM_L);
   
   if(PWM_L>PWM_Restrict)
   PWM_L=PWM_Restrict;                                     //限幅
   
   if(PWM_L<-PWM_Restrict)
   PWM_L=-PWM_Restrict;                                    //限幅  
   
   Last_bias_L=Bias_L;                                       //保存上一次偏差 
 
   return PWM_L;                                           //增量输出
}

int Incremental_PI_A_R (int Encoder,float Target1)
{  
  // static float Bias,PWM=0,Last_bias=0;                    //定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias_R=Target1-Encoder;                                  //计算偏差,目标值减去当前值
   PWM_R += Velocity_KP*(Bias_R-Last_bias_R)+Velocity_KI*Bias_R;   //增量式PI控制计算
   
   if(PWM_R>PWM_Restrict)
   PWM_R=PWM_Restrict;                                     //限幅
   
   if(PWM_R<-PWM_Restrict)
   PWM_R=-PWM_Restrict;                                    //限幅  
   
   Last_bias_R=Bias_R;                                       //保存上一次偏差 

   return PWM_R;                                           //增量输出
}

/**********PWM控制函数*********/
void Set_PWM_L(int motora)                        
{ 
  if (motora > 0)  //如果算出的PWM为正
  {
    analogWrite(Motor_BIN1,motora);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_BIN2, 0); //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (motora == 0)  //如果PWM为0停车
  {
    digitalWrite(Motor_BIN1, 0);
    digitalWrite(Motor_BIN2, 0);
  } else if (motora < 0)  //如果算出的PWM为负
  {
    
    analogWrite(Motor_BIN1, motora+255); //让PWM在设定反转方向反向输出调整
    digitalWrite(Motor_BIN2,1);
   
  }
  
}

void Set_PWM_R(int motora)                        
{ 
  if (motora > 0)  //如果算出的PWM为正
  {
    
    analogWrite(Motor_AIN1,motora);  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_AIN2, 0);
                                    //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } else if (motora == 0)  //如果PWM为0停车
  {
    digitalWrite(Motor_AIN1, 0);
    digitalWrite(Motor_AIN2, 0);
  } else if (motora < 0)  //如果算出的PWM为负
  {
    
   
    analogWrite(Motor_AIN1, motora+255); //让PWM在设定反转方向反向输出调整
    digitalWrite(Motor_AIN2,1);
   
  }
  
}

/***测距***/
void measure() {
  // 获取VL53L0X传感器的测量数据
  lox1.rangingTest(&measure1, false);
  // Serial.print("Sensor 1 (VL53L0X): ");

  // 超声波传感器测量距离
  digitalWrite(L_trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(L_trigPin, HIGH);
  delayMicroseconds(10); // 发送10微秒的HIGH脉冲
  digitalWrite(L_trigPin, LOW);

  long duration_1 = pulseIn(L_echoPin, HIGH);
  float mm_1 = (duration_1 / 2.0) * 0.343;  // 声速：343 m/s = 0.343 mm/µs，计算距离
  

  if (measure1.RangeStatus != 0) { // 如果测量有效
    Serial.print("Sensor 1 (VL53L0X): ");
    Serial.print("Distance (mm): ");
    measure_1 = measure1.RangeMilliMeter+30;
    Serial.println(measure_1);  // 输出毫米单位的距离
    Serial.print("Sensor 2 (Ultrasonic): Distance: ");
    Serial.print(mm_1);
    Serial.println(" mm");  // 输出毫米单位的距离
    measured_distance = 0.5*measure_1+0.5*mm_1;
    Serial.print("Processed Distance: ");
    Serial.print(measured_distance);
    Serial.println(" mm");  // 输出毫米单位的距离
    } else {
      // Serial.println("Out of range");  // 超出测量范围
    }



}

// void measure() {
//   // 获取第一个传感器的测量数据
//   lox1.rangingTest(&measure1, false);
//   Serial.print("Sensor 1: ");
//   if (measure1.RangeStatus != 0) { // 如果测量有效
//     Serial.print("Distance (mm): ");
//     Serial.println(measure1.RangeMilliMeter);
//   } else {
//     Serial.println("NaN");
//   }


//   digitalWrite(L_trigPin, LOW);
//   delayMicroseconds(10);
//   digitalWrite(L_trigPin, HIGH);
//   delayMicroseconds(10); // 改为10微秒的HIGH脉冲
//   digitalWrite(L_trigPin, LOW); 

//   duration_1 = pulseIn(L_echoPin, HIGH);
//   cm_1 = (duration_1 /2.0)*0.0343; // 声速：343 m/s = 0.0343 cm/µs
//   Serial.print("Distance: ");
//   Serial.print(cm_1);
//   Serial.println(" cm");
//   // 获取第二个传感器的测量数据
//   //  lox2.rangingTest(&measure2, false);
//   // Serial.print("Sensor 2: ");
//   // if (measure2.RangeStatus != 4) { // 如果测量有效
//   //  Serial.print("Distance (mm): ");
//   // //  Serial.println(measure2.RangeMilliMeter);
//   //  } else {
//   //  Serial.println("Out of range");
//   //  }
//   //  while(1);
//   //  while(1); // 停止程序
// }



