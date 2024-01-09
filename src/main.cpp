#include <Arduino.h>
#include "Header_files/Robot.h"
#include "Header_files/Kinematics.h"
#define null 0
Robot_Control Robot(20000,20000,20000,20000,20000,20000);
Kinematics Kin;
HardwareTimer timer1(TIM1);
HardwareTimer timer2(TIM4);
HardwareSerial serial1(PA10, PA9);
extern AccelStepper stepperX;
extern AccelStepper stepperY;
extern AccelStepper stepperZ;
float arr[5];
char buff[25];
int tProcess = NONE;
float POINT_A[3],POINT_B[3],POINT_C[3]; 
int curr_mp; //current moving point
int m_point;//the number of moving points displayed on screen (point 1, point 2, point 3)

/*Vectors in Robot.cpp flie*/

void HAL_TIM1_CallBack();
void data1();
void displayVector(float type_array);
/**/


void setup() {
  serial1.begin(9600);
  // serial1.setRx(PA_10);
  // serial1.setTx(PA_9);

  timer1.pause();
  timer1.setOverflow(1000,HERTZ_FORMAT);
  timer1.attachInterrupt(HAL_TIM1_CallBack);
  timer1.resume();

  // timer2.pause();
  // timer2.setOverflow(10,HERTZ_FORMAT);
  // timer2.attachInterrupt(data1);
  // timer2.resume();

// Serial.begin(9600);//khởi tạo PA9,PA10
  pinMode(LED_BUILTIN, OUTPUT);
  Robot.Calib_home();
  delay(250);
}
int count1 = 0; 
String var;
void loop() {
      //Robot.Calib_home();

    if(count1 == 1){
      Kin.inverse(100,0,-300);
      Robot.GetPosition(Kin.theta_1, Kin.theta_2, Kin.theta_3);
      delay(1000);
    }

    else if(count1 == 2){
      var = "300 f 100 0 -300 50 3";
      delay(1000);
    }
    else if(count1 == 3){
      Robot.Calib_home();
     delay(1000);
    }
    else if(count1 == 4){
      var = "300 s 100 0 -300 50 3";
      delay(1000);
    }
    else if(count1 == 5){
      //Robot.Calib_home();
    }
    else if(count1 == 6){
      var = "300 c 100 0 -300 50 3 100";
      delay(1000);
    }
    else {
      var = "0 a 0 0 0 0 0";
      delay(1000);
    }
    count1++;
   //String var = serial1.readStringUntil('\n');
    int str_len = var.length() + 1; 
    char char_array[str_len];
    var.toCharArray(char_array, str_len);
    char *s0= strtok(char_array," ");
    char *s1= strtok(NULL," ");
    char *s2= strtok(NULL," ");
    char *s3= strtok(NULL," ");
    char *s4= strtok(NULL," ");
    String s11=String(s1);
    float s22=String(s2).toFloat();
    float s33=String(s3).toFloat();
    float s44=String(s4).toFloat();
    char s11_char = s11.charAt(0); 
    switch(s11_char)
    {
      case 'f':
        {
          // serial4.println("f");
          Robot.GetPosition(s22,s33,s44);
        }
        break;
      case 'h'://home 
        {
          Robot.Calib_home();
        }
        break;
      case 'i':
        {
          //di chuyển đến tọa độ
          Kin.inverse(s22,s33,s44);
          Robot.GetPosition(Kin.theta_1, Kin.theta_2, Kin.theta_3);
        }
        break;
      case 'c': 
        { //path planning circle
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");
          char *s7= strtok(NULL," ");
          
          float s55=String(s5).toFloat();
          float s66=String(s6).toFloat();
          float s77=String(s7).toFloat();      
          
          POINT_A[0]=s22;
          POINT_A[1]=s33;
          POINT_A[2]=s44;

          Robot.mVelMax = s55;
          Robot.mT_Total = s66;
          Robot.mRadius = s77;

          Robot.mPosMax = 2*pi*Robot.mRadius;
          

          if(Robot.mT_Total > (2*Robot.mPosMax)/Robot.mVelMax)
          {
            Robot.mT_Total = (2*Robot.mPosMax)/Robot.mVelMax;
          }
          if(Robot.mT_Total <= (Robot.mPosMax/Robot.mVelMax))
          {
            Robot.mT_Total = (3*Robot.mPosMax)/(2*Robot.mVelMax);
          }
          Robot.mT_a = Robot.mT_Total - Robot.mPosMax/Robot.mVelMax;
          Robot.mAccelMax = 2*Robot.mVelMax/Robot.mT_a;
          Robot.mTime = 0;
          Robot.cPos = 0;
          tProcess = CIRCLE;
        }
        break;
      case 's'://motion s curve
        {
          char *s5= strtok(NULL," ");
          char *s6= strtok(NULL," ");     
          float s55=String(s5).toFloat(); //VEL
          float s66=String(s6).toFloat(); //TOTAL TIME
          float theta_x=stepperX.currentPosition()*0.028125;
          float theta_y=stepperY.currentPosition()*0.028125;
          float theta_z=stepperZ.currentPosition()*0.028125;
          Kin.forward(theta_x,theta_y,theta_z,&Kin.x, &Kin.y, &Kin.z);      
          POINT_A[0]=round(Kin.x);
          POINT_A[1]=round(Kin.y);
          POINT_A[2]=round(Kin.z);
          POINT_B[0]=s22;
          POINT_B[1]=s33;
          POINT_B[2]=s44;
          Robot.mVelMax = s55;  // vmax
          Robot.mT_Total = s66; //Total travelling time
          Robot.mPosMax = sqrt(pow(POINT_B[0] - POINT_A[0],2)+pow(POINT_B[1] - POINT_A[1],2)+pow(POINT_B[2] - POINT_A[2],2));
          Robot.mKx = (POINT_B[0]-POINT_A[0])/Robot.mPosMax;
          Robot.mKy = (POINT_B[1]-POINT_A[1])/Robot.mPosMax;
          Robot.mKz = (POINT_B[2]-POINT_A[2])/Robot.mPosMax;

          if(Robot.mT_Total > (2*Robot.mPosMax)/Robot.mVelMax)
          {
            Robot.mT_Total = (2*Robot.mPosMax)/Robot.mVelMax;
          }
          if(Robot.mT_Total <= (Robot.mPosMax/Robot.mVelMax))
          {
            Robot.mT_Total = (3*Robot.mPosMax)/(2*Robot.mVelMax);
          }
          Robot.mT_a = Robot.mT_Total - Robot.mPosMax/Robot.mVelMax;
          Robot.mAccelMax = 2*Robot.mVelMax/Robot.mT_a;
          Robot.mTime = 0;
          Robot.cPos = 0;


          if(Robot.mPosMax == 0)
            {tProcess = NONE;}
          else 
            // startTime = micros();
            tProcess = SCURVE;
        }
        break;
      
    }  

}
 
void HAL_TIM1_CallBack()
{
  switch(tProcess)
  {
    case NONE:
      break;
    case CIRCLE:
      Robot.EofGetCircle();
      break;
    case SCURVE:
      Robot.EofGetPos_sCurve();
      break;
    case WARNING:
      // Robot.OutOfRange();
      break;   
      }

}

// void data1(){
//     float alpha1=stepperX.currentPosition()*0.0140625;
//     float alpha2=stepperY.currentPosition()*0.0140625;
//     float alpha3=stepperZ.currentPosition()*0.0140625;
//     Kin.forward(alpha1,alpha2,alpha3,&Kin.x,&Kin.y,&Kin.z);
//     serial1.print("s");
//     serial1.print(" ");
//     serial1.print("k");
//     serial1.print(" ");
//     serial1.print(round(Kin.x));
//     serial1.print(" ");
//     serial1.print(round(Kin.y));
//     serial1.print(" ");
//     serial1.println(round(Kin.z));
// }
