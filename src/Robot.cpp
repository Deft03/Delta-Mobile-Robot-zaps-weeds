#include "Header_files/Robot.h"
#include "Header_files/Kinematics.h"
#include "HardwareSerial.h"
#include <Arduino.h>

Kinematics Kine;
AccelStepper stepperX(1, PA0, PA3, PA6);  //STEP DIR ENA
AccelStepper stepperY(1, PA1, PA4, PA6);
AccelStepper stepperZ(1, PA2, PA5, PA6);
MultiStepper steppersControl;
vector <vector<float>> MovingPoints;
vector <vector<float>> TeachingPoints;
vector <vector<float>> ObjPoints;
Robot_Control :: Robot_Control(int vx, int ax, int vy, int ay, int vz, int az) 
{
    // pinMode(en, OUTPUT);
    digitalWrite(enX, LOW);

    digitalWrite(enY, LOW);

    digitalWrite(enZ, LOW);

    // digitalWrite(en, LOW);
    pinMode(stepX, OUTPUT);
    pinMode(dirX, OUTPUT);

    pinMode(stepY, OUTPUT);
    pinMode(dirY, OUTPUT);

    pinMode(stepZ, OUTPUT);
    pinMode(dirZ, OUTPUT);
    
    pinMode(endX, INPUT_PULLUP);
    pinMode(endY, INPUT_PULLUP);
    pinMode(endZ, INPUT_PULLUP);

    stepperX.setMaxSpeed(vx);
    stepperX.setSpeed(vx);
    stepperX.setAcceleration(ax);

    stepperY.setMaxSpeed(vy);
    stepperY.setSpeed(vy);
    stepperY.setAcceleration(ay);

    stepperZ.setMaxSpeed(vz);
    stepperZ.setSpeed(vz);
    stepperZ.setAcceleration(az);

    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperZ.setCurrentPosition(0);

    steppersControl.addStepper(stepperX);
    steppersControl.addStepper(stepperY);
    steppersControl.addStepper(stepperZ);
}
void Robot_Control :: Config(int v, int a) 
{
    stepperX.setMaxSpeed(v);
    stepperX.setSpeed(v);
    stepperX.setAcceleration(a);

    stepperY.setMaxSpeed(v);
    stepperY.setSpeed(v);
    stepperY.setAcceleration(a);

    stepperZ.setMaxSpeed(v);
    stepperZ.setSpeed(v);
    stepperZ.setAcceleration(a);
}
void Robot_Control :: GetPosition(float theta1, float theta2, float theta3) 
{   
    stepperX.setSpeed(2000);
    stepperY.setSpeed(2000);
    stepperZ.setSpeed(2000);

    int pulse_1 = 12800 * theta1 / 360;
    int pulse_2 = 12800 * theta2 / 360;
    int pulse_3 = 12800 * theta3 / 360;
    stepperX.moveTo(pulse_1);
    stepperY.moveTo(pulse_2);
    stepperZ.moveTo(pulse_3);
    // digitalWrite(dirY,HIGH);
    while ((stepperX.distanceToGo() != 0) || (stepperY.distanceToGo() != 0) || (stepperZ.distanceToGo() != 0)) {
        stepperX.run();
        stepperY.run();
        stepperZ.run();

    }
}
void Robot_Control :: Calib_home()
{
    stepperX.setSpeed(-2000);
    stepperY.setSpeed(-2000);
    stepperZ.setSpeed(-2000);
    while(digitalRead(endX)||digitalRead(endY)||digitalRead(endZ))
    {
      if(digitalRead(endX) == 0)
      {stepperX.setSpeed(0);}
      if(digitalRead(endY) == 0)
      {stepperY.setSpeed(0);}
      if(digitalRead(endZ) == 0)
      {stepperZ.setSpeed(0);}
      stepperX.runSpeed();
      stepperY.runSpeed();
      stepperZ.runSpeed();
    }
    stepperX.setSpeed(2000);
    stepperY.setSpeed(2000);
    stepperZ.setSpeed(2000);

    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperZ.setCurrentPosition(0);
    GetPosition(30, 30, 30); // xuong 34 do cho ve 0 độ
    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperZ.setCurrentPosition(0);
}
void Robot_Control :: GetPositionConst(float theta1, float theta2, float theta3, float v1, float v2, float v3) 
{
// startTime = micros();
    long gotoposition[3];
    gotoposition[0] = 12800 * theta1 / 360;
    gotoposition[1] = 12800 * theta2 / 360;
    gotoposition[2] = 12800 * theta3 / 360;
    stepperX.setSpeed(v1);
    stepperY.setSpeed(v2);
    stepperZ.setSpeed(v3);
    steppersControl.moveTo(gotoposition);
    steppersControl.runSpeedToPosition();

}
void Robot_Control :: EofGetPos_sCurve()  //S curve motion profile
{  
    // startTime = micros();
    if (mTime <= mT_a / 2) {
        // cVel = mAccelMax*pow(mTime, 2)/mT_a;
        cPos = mAccelMax * pow(mTime, 3) / (3 * mT_a);
    } else if (mT_a / 2 < mTime && mTime <= mT_a) {
        // cVel = -(mAccelMax*pow(mTime, 2)/mT_a) + 2*mAccelMax*mTime - mAccelMax*mT_a/2;
        cPos = -(mAccelMax * pow(mTime, 3)) / (3 * mT_a) + mAccelMax * pow(mTime, 2) - mAccelMax * mT_a * mTime / 2 + mAccelMax * pow(mT_a, 2) / 12;
    } else if (mT_a < mTime && mTime <= (mT_Total - mT_a)) {
        // cVel = mAccelMax*mT_a/2;
        cPos = mAccelMax * mT_a * mTime / 2 - mAccelMax * pow(mT_a, 2) / 4;
    } else if (mT_Total - mT_a < mTime && mTime <= mT_Total - mT_a / 2) {
        // cVel = -mAccelMax*pow(mTime, 2)/mT_a + 2*mAccelMax*(mT_Total - mT_a)*mTime/mT_a + mAccelMax*mT_a/2 - mAccelMax*pow((mT_Total - mT_a),2)/mT_a;
        cPos = mAccelMax * (-pow(mTime, 3) / (3 * mT_a) + (mT_Total - mT_a) * pow(mTime, 2) / mT_a + mT_a * mTime / 2 - pow((mT_Total - mT_a), 2) * mTime / mT_a + pow((mT_Total - mT_a), 3) / (3 * mT_a) - pow(mT_a, 2) / 4);
    } else if (mT_Total - mT_a / 2 < mTime && mTime <= mT_Total) {
        // cVel = mAccelMax*pow(mTime, 2)/mT_a - 2*mAccelMax*mT_Total*mTime/mT_a + mAccelMax*pow(mT_Total, 2)/mT_a;
        cPos = mAccelMax * (pow(mTime, 3) / (3 * mT_a) - mT_Total * pow(mTime, 2) / mT_a + pow(mT_Total, 2) * mTime / mT_a - pow(mT_Total, 3) / (3 * mT_a) + pow(mT_a, 2) / 24 + mT_Total * mT_a / 2 - 13 * pow(mT_a, 2) / 24);
    }
    // else {
    //   cPos = mPosMax;
    // }

    float toa_do_x = POINT_A[0] + mKx * cPos;
    float toa_do_y = POINT_A[1] + mKy * cPos;
    float toa_do_z = POINT_A[2] + mKz * cPos;
    Kine.inverse(toa_do_x, toa_do_y, toa_do_z);
    
    // if ((Kine.theta_1 <= -28 || Kine.theta_1 >= 67) || (Kine.theta_2 <= -28 || Kine.theta_2 >= 67) || (Kine.theta_3 <= -28 || Kine.theta_3 >= 67)) {

    //     cPos = 0;
    //     mTime = 0;
    //     tProcess = WARNING;
    // }

    float step1 = 12800 * Kine.theta_1 / 360 - stepperX.currentPosition();  //xung stepper cần đi
    float step2 = 12800 * Kine.theta_2 / 360 - stepperY.currentPosition();
    float step3 = 12800 * Kine.theta_3 / 360 - stepperZ.currentPosition();
    // float v1 = abs(step1 * 500);  //[pul/s]
    // if (v1 > 2000) { v1 = 2000; }
    // float v2 = abs(step2 * 500);
    // if (v2 > 2000) { v2 = 2000; }
    // float v3 = abs(step3 * 500);
    //if (v3 > 2000) { v3 = 2000; }

    float v1 = abs(step1 * 1000);  //[pul/s]
    if (v1 > 20000) { v1 = 20000; }
    float v2 = abs(step2 * 1000);
    if (v2 > 20000) { v2 = 20000; }
    float v3 = abs(step3 * 1000);
    if (v3 > 20000) { v3 = 20000; }

    GetPositionConst(Kine.theta_1, Kine.theta_2, Kine.theta_3, v1, v2, v3);


    if (mTime > mT_Total) {
        cPos = 0;
        // cVel = 0;
        mTime = 0;
        tProcess = NONE;
    }

    mTime += 0.05;
    // serial.print(mTime);
    // unsigned long elapsedTime = micros() - startTime;
    //serial4.print("Thời gian thực hiện (us): ");
    // serial4.println(elapsedTime);
}
void Robot_Control :: EofGetCircle()  //circle path planning
{

    if (mTime <= mT_a / 2) {
        cVel = mAccelMax * pow(mTime, 2) / mT_a;
        cPos = mAccelMax * pow(mTime, 3) / (3 * mT_a);
    } else if (mT_a / 2 < mTime && mTime <= mT_a) {
        cVel = -(mAccelMax * pow(mTime, 2) / mT_a) + 2 * mAccelMax * mTime - mAccelMax * mT_a / 2;
        cPos = -(mAccelMax * pow(mTime, 3)) / (3 * mT_a) + mAccelMax * pow(mTime, 2) - mAccelMax * mT_a * mTime / 2 + mAccelMax * pow(mT_a, 2) / 12;
    } else if (mT_a < mTime && mTime <= (mT_Total - mT_a)) {
        cVel = mAccelMax * mT_a / 2;
        cPos = mAccelMax * mT_a * mTime / 2 - mAccelMax * pow(mT_a, 2) / 4;
    } else if (mT_Total - mT_a < mTime && mTime <= mT_Total - mT_a / 2) {
        cVel = -mAccelMax * pow(mTime, 2) / mT_a + 2 * mAccelMax * (mT_Total - mT_a) * mTime / mT_a + mAccelMax * mT_a / 2 - mAccelMax * pow((mT_Total - mT_a), 2) / mT_a;
        cPos = mAccelMax * (-pow(mTime, 3) / (3 * mT_a) + (mT_Total - mT_a) * pow(mTime, 2) / mT_a + mT_a * mTime / 2 - pow((mT_Total - mT_a), 2) * mTime / mT_a + pow((mT_Total - mT_a), 3) / (3 * mT_a) - pow(mT_a, 2) / 4);
    } else if (mT_Total - mT_a / 2 < mTime && mTime <= mT_Total) {
        cVel = mAccelMax * pow(mTime, 2) / mT_a - 2 * mAccelMax * mT_Total * mTime / mT_a + mAccelMax * pow(mT_Total, 2) / mT_a;
        cPos = mAccelMax * (pow(mTime, 3) / (3 * mT_a) - mT_Total * pow(mTime, 2) / mT_a + pow(mT_Total, 2) * mTime / mT_a - pow(mT_Total, 3) / (3 * mT_a) + pow(mT_a, 2) / 24 + mT_Total * mT_a / 2 - 13 * pow(mT_a, 2) / 24);
    }
    float currAngle = cPos * 2 * pi / mPosMax;

    // float toa_do_x = (POINT_A[0] - x_offset * mRadius) * cos(currAngle) + (POINT_A[1] - y_offset * mRadius) * sin(currAngle) + x_offset * mRadius;
    // float toa_do_y = (POINT_A[1] - y_offset * mRadius) * cos(currAngle) - (POINT_A[0] - x_offset * mRadius) * sin(currAngle) + y_offset * mRadius;
    float toa_do_x = POINT_A[0] - mRadius + mRadius * cos(currAngle);
    float toa_do_y = POINT_A[1] + mRadius * sin(currAngle);
    float toa_do_z = POINT_A[2];

    Kine.inverse(toa_do_x, toa_do_y, toa_do_z);

    // if ((Kine.theta_1 <= -28 || Kine.theta_1 >= 67) || (Kine.theta_2 <= -28 || Kine.theta_2 >= 67) || (Kine.theta_3 <= -28 || Kine.theta_3 >= 67)) {

    //     cPos = 0;
    //     mTime = 0;
    //     tProcess = WARNING;
    // }
    float step1 = 320 * Kine.theta_1 / 9 - stepperX.currentPosition();  //xung stepper cần đi
    float step2 = 320 * Kine.theta_2 / 9 - stepperY.currentPosition();
    float step3 = 320 * Kine.theta_3 / 9 - stepperZ.currentPosition();
    float v1 = abs(step1 * 500);  //[pul/s]
    if (v1 > 20000) { v1 = 20000; }
    float v2 = abs(step2 * 500);
    if (v2 > 20000) { v2 = 20000; }
    float v3 = abs(step3 * 500);
    if (v3 > 20000) { v3 = 20000; }
    GetPositionConst(Kine.theta_1, Kine.theta_2, Kine.theta_3, v1, v2, v3);

    if (mTime > mT_Total) {
        cPos = 0;
        cVel = 0;
        mTime = 0;
        tProcess = NONE;
    }
    mTime += 0.02;
}
// void Robot_Control :: SaveTeachingPos(int point_num, float t_x, float t_y, float t_z, float t_vel, float t_time) //save current position to wanted "cell"
// {
//     // float alpha1=stepperX.currentPosition()*0.0140625;
//     // float alpha2=stepperY.currentPosition()*0.0140625;
//     // float alpha3=stepperZ.currentPosition()*0.0140625;
//     // forward(alpha1,alpha2,alpha3,&x,&y,&z);
//     // TeachingArr[point_num][0] = t_x;
//     // TeachingArr[point_num][1] = t_y;
//     // TeachingArr[point_num][2] = t_z;  
//     // TeachingArr[point_num][3] = t_vel; 
//     // TeachingArr[point_num][4] = t_time;  
//     vector<float> temp;
//     temp.push_back(t_x);
//     temp.push_back(t_y);
//     temp.push_back(t_z);
//     temp.push_back(t_vel);
//     temp.push_back(t_time);
//     TeachingPoints.at(point_num) = temp;
// }
// void Robot_Control :: SaveMovingPos(int point_num, float m_x, float m_y, float m_z, float m_vel, float m_time)
// {
//     // float alpha1 = stepperX.currentPosition()*0.0140625;
//     // float alpha2 = stepperY.currentPosition()*0.0140625;
//     // float alpha3 = stepperZ.currentPosition()*0.0140625;
//     // forward(alpha1,alpha2,alpha3,&x,&y,&z);
//     // MovingArr[point_num][0] = m_x;
//     // MovingArr[point_num][1] = m_y;
//     // MovingArr[point_num][2] = m_z;  
//     // MovingArr[point_num][3] = m_vel;
//     // MovingArr[point_num][4] = m_time;
//     vector<float> temp;
//     temp.push_back(m_x);
//     temp.push_back(m_y);
//     temp.push_back(m_z);
//     temp.push_back(m_vel);
//     temp.push_back(m_time);
//     MovingPoints.at(point_num) = temp;
// }
// void Robot_Control :: SaveObjPos(int b_i, float b_x, float b_y, float b_z, float b_time) 
// {
//     // BoxPos[0][0] = b_i; // index: 0:red | 1:blue | 2:yellow |3:product 1|4:product 2|5:product 3
//     // BoxPos[0][1] = b_x;
//     // BoxPos[0][2] = b_y;
//     // BoxPos[0][3] = b_z;
//     // BoxPos[0][4] = b_t; //predicted time for robot catching an object
//     vector<float> temp;
//     temp.push_back(b_i);
//     temp.push_back(b_x);
//     temp.push_back(b_x);
//     temp.push_back(b_x);
//     temp.push_back(b_time);
//     ObjPoints.at(0) = temp;

// }
// void Robot_Control :: ClearTeachingPos(int point_num)
// {
//     vector<float> temp;
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     TeachingPoints.at(point_num) = temp;
// }
// void Robot_Control :: ClearMovingPos(int point_num)
// {
//     vector<float> temp;
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     temp.push_back(0);
//     MovingPoints.at(point_num) = temp;
// }
// void Robot_Control :: ClrAllPos()
// {
//     MovingPoints.clear();
//     TeachingPoints.clear();

// }
// void Robot_Control :: ClrObjPos()
// {
//     ObjPoints.clear();
// // }
// void Robot_Control :: MovingProcess()
// {

//     if (mTime <= mT_a / 2) {
//         // cVel = mAccelMax*pow(mTime, 2)/mT_a;
//         cPos = mAccelMax * pow(mTime, 3) / (3 * mT_a);
//     } else if (mT_a / 2 < mTime && mTime <= mT_a) {
//         // cVel = -(mAccelMax*pow(mTime, 2)/mT_a) + 2*mAccelMax*mTime - mAccelMax*mT_a/2;
//         cPos = -(mAccelMax * pow(mTime, 3)) / (3 * mT_a) + mAccelMax * pow(mTime, 2) - mAccelMax * mT_a * mTime / 2 + mAccelMax * pow(mT_a, 2) / 12;
//     } else if (mT_a < mTime && mTime <= (mT_Total - mT_a)) {
//         // cVel = mAccelMax*mT_a/2;
//         cPos = mAccelMax * mT_a * mTime / 2 - mAccelMax * pow(mT_a, 2) / 4;
//     } else if (mT_Total - mT_a < mTime && mTime <= mT_Total - mT_a / 2) {
//         // cVel = -mAccelMax*pow(mTime, 2)/mT_a + 2*mAccelMax*(mT_Total - mT_a)*mTime/mT_a + mAccelMax*mT_a/2 - mAccelMax*pow((mT_Total - mT_a),2)/mT_a;
//         cPos = mAccelMax * (-pow(mTime, 3) / (3 * mT_a) + (mT_Total - mT_a) * pow(mTime, 2) / mT_a + mT_a * mTime / 2 - pow((mT_Total - mT_a), 2) * mTime / mT_a + pow((mT_Total - mT_a), 3) / (3 * mT_a) - pow(mT_a, 2) / 4);
//     } else if (mT_Total - mT_a / 2 < mTime && mTime <= mT_Total) {
//         // cVel = mAccelMax*pow(mTime, 2)/mT_a - 2*mAccelMax*mT_Total*mTime/mT_a + mAccelMax*pow(mT_Total, 2)/mT_a;
//         cPos = mAccelMax * (pow(mTime, 3) / (3 * mT_a) - mT_Total * pow(mTime, 2) / mT_a + pow(mT_Total, 2) * mTime / mT_a - pow(mT_Total, 3) / (3 * mT_a) + pow(mT_a, 2) / 24 + mT_Total * mT_a / 2 - 13 * pow(mT_a, 2) / 24);
//     }
//     // else {
//     //   cPos = mPosMax;
//     // }

//     float toa_do_x = POINT_A[0] + mKx * cPos;
//     float toa_do_y = POINT_A[1] + mKy * cPos;
//     float toa_do_z = POINT_A[2] + mKz * cPos;
//     Kine.inverse(toa_do_x, toa_do_y, toa_do_z);
//     if ((Kine.theta_1 <= -28 || Kine.theta_1 >= 67) || (Kine.theta_2 <= -28 || Kine.theta_2 >= 67) || (Kine.theta_3 <= -28 || Kine.theta_3 >= 67)) {

//         cPos = 0;
//         mTime = 0;
//         tProcess = WARNING;
//     }

//     float step1 = 640 * Kine.theta_1 / 9 - stepperX.currentPosition();  //xung stepper cần đi
//     float step2 = 640 * Kine.theta_2 / 9 - stepperY.currentPosition();
//     float step3 = 640 * Kine.theta_3 / 9 - stepperZ.currentPosition();
//     float v1 = abs(step1 * 1000);  //[pul/s]
//     if (v1 > 20000) { v1 = 20000; }
//     float v2 = abs(step2 * 1000);
//     if (v2 > 20000) { v2 = 20000; }
//     float v3 = abs(step3 * 1000);
//     if (v3 > 20000) { v3 = 20000; }

//     GetPositionConst(Kine.theta_1, Kine.theta_2, Kine.theta_3, v1, v2, v3);
//     // float alpha1 = stepperX.currentPosition() * 0.0140625;
//     // float alpha2 = stepperY.currentPosition() * 0.0140625;
//     // float alpha3 = stepperZ.currentPosition() * 0.0140625;
//     // forward(alpha1, alpha2, alpha3, &x, &y, &z);
//     // serial4.print(round(x));
//     // serial4.print(" ");
//     // serial4.print(round(y));
//     // serial4.print(" ");
//     // serial4.println(round(z));
//     if (mTime > mT_Total) 
//     {
//         if(running_point == 5)
//         {
//         digitalWrite(valve, HIGH);
//         }
//         cPos = 0;
//         mTime = 0;
//         tProcess = CAL_MOVING;
//     }
//     mTime += 0.001;

// }