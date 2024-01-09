 #include "Header_files/Kinematics.h"

int Kinematics::forward(float thetaA, float thetaB, float thetaC, float *x11, float *y11, float *z11) 
  {
    *x11 = 0.0;
    *y11 = 0.0;
    *z11 = 0.0;

    float t = (BassTri - PlatformTri) * tan30 / 2.0;
    float dtr = pi / 180.0;

    thetaA *= dtr;
    thetaB *= dtr;
    thetaC *= dtr;

    float y1 = -(t + ArmLength * cos(thetaA));
    float z1 = -ArmLength * sin(thetaA);

    float y2 = (t + ArmLength * cos(thetaB)) * sin30;
    float x2 = y2 * tan60;
    float z2 = -ArmLength * sin(thetaB);

    float y3 = (t + ArmLength * cos(thetaC)) * sin30;
    float x3 = -y3 * tan60;
    float z3 = -ArmLength * sin(thetaC);

    float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    float w1 = y1 * y1 + z1 * z1;
    float w2 = x2 * x2 + y2 * y2 + z2 * z2;
    float w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    // y = (a2*z + b2)/dnm;
    float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    // a*z^2 + b*z + c = 0
    float aV = a1 * a1 + a2 * a2 + dnm * dnm;
    float bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    float cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - RodLength * RodLength);

    // discriminant
    float dV = bV * bV - 4.0 * aV * cV;
    if (dV < 0.0) {
        return non_existing_povar_error;  // non-existing povar. return error,x,y,z
    }

    *z11 = -0.5 * (bV + sqrt(dV)) / aV;
    *x11 = (a1 * (*z11) + b1) / dnm;
    *y11 = (a2 * (*z11) + b2) / dnm;


    return no_error;
}
int Kinematics::delta_calcAngleYZ(float *Angle, float x0, float y0, float z0) 
{
    float y1 = -0.5 * tan30 * BassTri;  // f/2 * tan(30 deg)
    y0 -= 0.5 * tan30 * PlatformTri;    // shift center to edge

    // z = a + b*y
    float aV = (x0 * x0 + y0 * y0 + z0 * z0 + ArmLength * ArmLength - RodLength * RodLength - y1 * y1) / (2.0 * z0);
    float bV = (y1 - y0) / z0;

    // discriminant
    float dV = -(aV + bV * y1) * (aV + bV * y1) + ArmLength * (bV * bV * ArmLength + ArmLength);
    if (dV < 0) {
        return non_existing_povar_error;  // non-existing povar.  return error, theta
    }

    float yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1);  // choosing outer povar
    float zj = aV + bV * yj;
    *Angle = atan2(-zj, (y1 - yj)) * 180.0 / pi;

    return no_error;  // return error, theta
}

// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)
int Kinematics::inverse(float x0, float y0, float z0) 
{
    // theta_1 = 0;
    // theta_2 = 0;
    // theta_3 = 0;
    int error = delta_calcAngleYZ(&theta_1, x0, y0, z0);
    if (error != no_error)
        return error;
    error = delta_calcAngleYZ(&theta_2, x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0);
    if (error != no_error)
        return error;
    error = delta_calcAngleYZ(&theta_3, x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0);

    return error;
}
