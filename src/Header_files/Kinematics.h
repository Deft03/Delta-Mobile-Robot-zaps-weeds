#include <math.h>

#define sqrt3 1.7320508075688772935274463415059
const float pi = 3.1415926535897932384626433832795;  // PI
#define sin120 sqrt3 / 2.0
#define cos120 -0.5
#define tan60 sqrt3
#define sin30 0.5
#define tan30 1.0 / sqrt3
#define non_existing_povar_error -2
#define no_error 1
class Kinematics
{
    private:
        float Gear_ratio1 = 4;
        float Gear_ratio2 = 4;
        float Gear_ratio3 = 4;
        float ref_theta1 = -30;
        float ref_theta2 = -30;
        float ref_theta3 = -30;
        float theta1 = 0, theta2 = 0, theta3 = 0;
        float ArmLength =175;//170; //lower arm L
        float RodLength = 450;//420; //upper arm l
        float BassTri =  520 ;//490; //top base triangle
        float PlatformTri = 34.64;//30; 
    public:
        float theta_1;
        float theta_2;
        float theta_3;
        float x, y, z;
        int forward(float thetaA, float thetaB, float thetaC, float *x11, float *y11, float *z11);
        int delta_calcAngleYZ(float *Angle, float x0, float y0, float z0);
        int inverse(float x0, float y0, float z0);
        
};