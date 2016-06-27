#include <iostream>
#include <stdio.h>
#include <conio.h>
#include "global.h"

using namespace std;
int main(int argc, char *argv[])
{
    std::cout << "hello" <<std::endl;
    description();
    bool ok = device_setup();

    if (ok == false)
    {
        return false;
    }

    WithRobot::SensorData imu_data;
    yr.request_rssi();
    double heading_angle = 0.0f;

    while(true)
    {
        if ( encoder.bReceived == true && kalman.firstTime == false)
        {
            //  Calculate Wheel odometry by 4-mecanum wheel
            kalman.predict(encoder.prev,encoder.curr,heading_angle);

        }

        if ( yr.bRecognized == true && yr.info.rssi > _REF_RSSI_ )
        {
            //  Calculate Kalman filter using wheel odometry and tr location
            kalman.update(yr.info.x,yr.info.y);
            //  printf("x : %d, y : %d, rssi : %d \n",yr.info.x,yr.info.y,yr.info.rssi);
            yr.bRecognized = false;
        }

        if( imu.wait_data() == true )
        {
            //  Get a heading angle from IMU
            imu.get_data(imu_data);
            WithRobot::EulerAngle& e = imu_data.euler_angle;
            heading_angle = extraction_heading_angle(e.yaw);

            //  std::cout << "heading : " << heading_angle <<std::endl;
        }

        printf("x : %d, y : %d, rssi : %d , heading : %f \n", yr.info.x,yr.info.y,yr.info.rssi,heading_angle);
        encoder.send_pose(1,kalman.state[0],kalman.state[1],(heading_angle*M_PI/2)*10);

        if(kbhit() != 0)
        {
            char key = getch();
            if (key == 'q')
            {
                encoder.stop();
                yr.stop();
                imu.stop();
                return 1;
            }
        }
    }
    return 0;
}
