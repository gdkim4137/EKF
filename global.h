#ifndef GLOBAL_H
#define GLOBAL_H

#include "ekf_algorithm.h"
#include "yr9010.h"
#include "myahrs_plus.hpp"
#include "encoder.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                 Must consider experimental value
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define _REF_ANGLE_ 60.0f   //  adjust y-axis coordinates
#define _REF_RSSI_  50.0f   //  cut off distance 50mm

YR9010 yr;
WithRobot::MyAhrsPlus imu;
Encoder encoder;
EKF kalman;




void description(void)
{
    std::cout << "This app is for MOBILE INDUSTRY CLUSTER" << std::endl
              << "version : based on console command " << std::endl
              << "latest  :  2016/06/28(Tue)" <<std::endl;
}
double extraction_heading_angle(double yaw)
{
    double real = yaw;

    // -180 ~ 180 => 0 ~ 360 conversion
    if ( real < 0 ) real= real+360;

    // convert left hand to right hand coordinates
    // CCW : positive(+), CW : neagtive(-)
    real = 360 - real;
    double gap = _REF_ANGLE_ -90;
    double heading = real-gap;

    if( gap > 0 )
    {
        //  Case 1 : Leading Phase ( ref > 90 degree )
        if ( 0 < real && real <= gap )
        {
            // heading will be replaced to 360-gap ~ 0
            heading = real - gap + 360;
        }
    }
    else if( gap < 0 )
    {
        // Case 2 : Lagging case ( ref < 90 degree )
        if ( 360 + gap < real && real <= 360 )
        {
            heading = real - gap - 360;
        }
    }
    return heading;
}

bool device_setup()
{

    //  UHF RFID, YR9010
    //  /dev/ttyUSB0
    if(yr.start("COM4",115200) == false)
    {
        std::cout << "Can not access the RFID Reader"<<std::endl;
        return false;
    }
    Sleep(100);
  //  this->msleep(1000);


    //  IMU, myAHRS+
    //  /dev/ttyACM0
    if(imu.start("COM3",115200)==false)
    {
        std::cout <<"Can not access the myAHRS+"<<std::endl;
        return false;
    }
    imu.cmd_ascii_data_format("RPY");
    imu.cmd_divider(1);
    imu.cmd_mode("AC");

    Sleep(100);


    //  Encoder
    /*
    if(encoder.start("/dev/ttyUSB0",115200)==false)
    {
        std::cout << "Can not access the encoder"<<std::endl;
        emit signal_LogMsgOccured("Can not access the encoder");
        return false;
    }
    */
    return true;
}



#endif // GLOBAL_H
