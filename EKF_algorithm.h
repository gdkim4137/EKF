#ifndef EKF_ALGORITHM_H
#define EKF_ALGORITHM_H

#include <QGenericMatrix>
#include <QtMath>
#include "Encoder.h"

#define A 100           // distance of vehicle along x axis
#define B 100           // distance of vehicle along y axis
#define C 1/(A+B)
#define DIAMETER    160 // diameter of wheel

#define PULSE       4
#define GEAR_RATIO  20
#define pi 3.141592


//  Coordinates system in this project based on Earth Coordinates system
//  so, we have used for coordinate conversion to adjust different system

//  Local location of vehicle - Earth Coordinates system
//      we used for IMU(myAHRS+) to treat this problem
//      IMU sensor can know the arctic direction anywhere

//  Local location of transponder - Earth Coordinates system
//      we need to measure twist angle between which systems


       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //              must define twist-angle
       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const double twistAngle = 0.0;  //  adust twist-angle between lattice of pattern and
typedef AngularVelocity LinearVelocity;

class EKF{

public:
    //  x(mm), y(mm), w(rad)
    //QGenericMatrix<2,1,double> state;
    double state[2];

protected:
    //QGenericMatrix<2,2,double> coordinate_conversion_mat;   //  local to world coordinates for vehicle
    double coordinate_conversion_mat[4];

    double p[2][2]; //  error co-variance
    double q[2][2]; //  system co-variance
    double r[2][2]; //  observation co-variance
    double k[2][2]; //  kalman gain

public:
    bool firstTime;

public:
    EKF()
    {
       firstTime = true;

       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       //              must define initial co-variance to each matrix(system,observation, error)
       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

       //   kalman filter co-variance
       p[0][0] = 0; p[0][1] = 0;
       p[1][0] = 0; p[1][1] = 0;

       //   system noise
       q[0][0] = 0; q[0][1] = 0;
       q[1][0] = 0; q[1][1] = 0;

       //   observation noise
       r[0][0] = 0; r[0][1] = 0;
       r[1][0] = 0; r[1][1] = 0;

       //   kalman gain
       k[0][0] = 0; k[0][1] = 0;
       k[1][0] = 0; k[1][1] = 0;
    }
public:
    void get_location(int &x, int &y)
    {
        x = state[0]*10;  // up to scale 10^-4
        y = state[1]*10;  // up to scale 10^-4
    }

public:
    void predict(AngularVelocity prev, AngularVelocity curr, double heading)
    {
    //    std::cout << "check debug1"<<std::endl;
        //  Local to global coordinate conversion mat, 2X2
        cacl_rot_mat(heading);


        //  calculate linear velocity about each wheel using previous and current absolute encoder value
        LinearVelocity lv = calc_LinearVelocity(prev,curr);

        //  calculate moving distance about each direction
        double dx = lv.left_top + lv.left_bottom + lv.right_top + lv.right_bottom;
        double dy = -lv.left_top + lv.left_bottom + lv.right_top - lv.right_bottom;

   //     std::cout << "check debug2"<<std::endl;
        //  calculate current location using previous location, moving distance and heading anlge
        state[0] = state[0] + coordinate_conversion_mat[0]*dx + coordinate_conversion_mat[1]*dy + q[0][0];
        state[1] = state[1] + coordinate_conversion_mat[2]*dx + coordinate_conversion_mat[3]*dy + q[1][1];

     //   std::cout << "check debug3"<<std::endl;
        //  calculate error co-variance
        p[0][0] += q[0][0];
        p[0][1] += q[0][1]; //  appoximately zero
        p[1][0] += q[1][0]; //  appoximately zero
        p[1][1] += q[1][1];

        //  calculate kalman gain
        //  k = I + inv(R), where R is diagonal matrix, so inv(r) is inverse of each diagonal elements
        k[0][0] = 1 + (1/r[0][0]);
        k[0][1] = 0;
        k[1][0] = 0;
        k[1][1] = 1 + (1/r[1][1]);
    }

    void update(int x, int y)
    {
        /*
        // Uupdate pose of vehicle when UHF Transponder is detected by reader

        // location of transponder is restored to rectangular form
        // so, we must do that convert real world location( transponder gap is 500mm )
        double location[] = {500*x,500*y};

        //  to consider twist-angle between global coordinates and earth coordinates
        QGenericMatrix<2,2,double> rot_mat = cacl_rot_mat(twistAngle);


        //  observed location from detected transponder
        QGenericMatrix<2,1,double> observed;
        observed(0,0) = rot_mat(0,0)*location[0] + rot_mat(0,1)*location[1];
        observed(1,0) = rot_mat(1,0)*location[0] + rot_mat(1,1)*location[1];

        //  update error covariance
        if(firstTime)
        {
            //  if the first time transponder detected
            //  enable to calculate the odometry by each encoder
            state = observed;
        }
        else
        {
            //  calculate X(prediction) - Z(observation)
            double different_dx = observed(0,0) - state(0,0);
            double different_dy = observed(1,0) - state(1,0);

            // update location
            state(0,0) = state(0,0) + k[0][0]*different_dx;
            state(1,0) = state(1,0) + k[1][0]*different_dy;

            //  update error co-variance
            p[0][0] = p[0][0] - k[0][0]*p[0][0];
            p[1][1] = p[1][1] - k[1][1]*p[1][1];
        }
        */

    }

protected:
    void cacl_rot_mat(double heading)
    {

        //  convert degree to radian
        double rad = qDegreesToRadians(heading);//(2*pi*heading)/360.f;


        coordinate_conversion_mat[0] = qCos(rad);
        coordinate_conversion_mat[1] = -qSin(rad);
        coordinate_conversion_mat[2] = qSin(rad);
        coordinate_conversion_mat[3] = qCos(rad);

    }

    LinearVelocity calc_LinearVelocity(AngularVelocity prev, AngularVelocity curr)
    {
       LinearVelocity result;
       result.left_top      = ((curr.left_top-prev.left_top)        *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.left_bottom   = ((curr.left_bottom-prev.left_bottom)  *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.right_top     = ((curr.right_top-prev.right_top)      *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.right_bottom  = ((curr.right_bottom-prev.right_bottom)*2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       return result;
    }

public:
    LinearVelocity calc_LinearVelocity(AngularVelocity diff)
    {
       LinearVelocity result;
       result.left_top      = ((diff.left_top)        *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.left_bottom   = ((diff.left_bottom)     *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.right_top     = ((diff.right_top)       *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       result.right_bottom  = ((diff.right_bottom)    *2*pi / (PULSE*GEAR_RATIO)) * 0.01 * (DIAMETER/2);
       return result;
    }
};


#endif // EKF_ALGORITHM_H
