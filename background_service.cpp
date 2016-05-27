#include "background_service.h"

Background_Service::Background_Service(QObject *parent) :
    QThread(parent)
{
    mStop = false;


}

bool Background_Service::device_setup()
{
    bool check = false;

    /*
    //  UHF RFID, YR9010
    if(yr.start("/dev/ttyUSB0",115200) == false)
    {
        std::cout << "Can not access the RFID Reader"<<std::endl;
        emit signal_LogMsgOccured("Can not access the RFID Reader");
        return false;
    }
    this->msleep(1000);

    //  IMU, myAHRS+
    if(sensor.start("/dev/ttyACM0",115200)==false)
    {
        std::cout <<"Can not access the myAHRS+"<<std::endl;
        emit signal_LogMsgOccured("Can not access the myAHRS+");
        return false;
    }
    sensor.cmd_ascii_data_format("RPY");
    sensor.cmd_divider(1);
    sensor.cmd_mode("AC");

    this->msleep(1000);

    */
    //  Encoder
    if(encoder.start("/dev/ttyUSB0",115200)==false)
    {
        std::cout << "Can not access the encoder"<<std::endl;
        emit signal_LogMsgOccured("Can not access the encoder");
        return false;
    }

    return true;

}

void Background_Service::run()
{
    if(device_setup()==false)
    {
        return;
    }

    emit signal_LogMsgOccured("device setting is complete!");

    WithRobot::SensorData sensor_data;
    std::string command_string;

    while(true)
    {
        this->msleep(10);
        // yr.request_rssi();


      //  std::cout << "while loop"<<std::endl;

//        char command[] = {0x55,0x56,0x57,0x58,0x0A,0x0D};
        if( encoder.bReceived == true)
        {

            //  send pose to vehicle
            char command[] = { 0x55, 0x55,
                               0x51,0x51,0x51,0x51, //  key pose
                               0x51,0x51,0x51,0x51, //  x location
                               0x52,0x52,0x52,0x52, //  y location
                               0x53,0x53,0x53,0x53, //  w (heading angle)
                               0xAA, 0xAA};


            for (int i = 0 ; i < sizeof(command) ; ++i)
                command_string += command[i];

            encoder.send_pose(command_string);
            command_string.clear();



         //   std::cout << curr.left_top <<" "<< curr.right_top <<" "<< curr.left_bottom <<" "<< curr.right_bottom << std::endl;

            emit signal_AngularVelocityReceived(encoder.curr.left_top,encoder.curr.right_top,
                                                encoder.curr.left_bottom,encoder.curr.right_bottom);
//
            // EKF prediction step run

            // send pose
            encoder.bReceived = false;
        }

/*

        if( encoder.bReceived == true)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //              should be changed because don't consider encoder decoded
            //              and
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            //  EKF prediction step
     //     if(kalman.firstTime)    break;
     //     else                    kalman.predict();

            emit signal_AngularVelocityReceived(encoder.info);
            encoder.bReceived = false;
        }


        if(sensor.wait_data()==true)
        {
            //  receive heading angle
            sensor.get_data(sensor_data);
            WithRobot::EulerAngle& e = sensor_data.euler_angle;
            //sensor_count = sensor.get_sample_count();

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //              imu value need to change range of value from [-180, 180] to [0, 360]
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            std::cout << e.yaw << std::endl;
            if (e.yaw > 0 )
            {
                emit signal_HeadingAngleReceived(e.yaw);
            }
            else if (e.yaw < 0)
            {
                emit signal_HeadingAngleReceived(360.0-std::abs(e.yaw));
            }
        }
        if(yr.bRecognized)
        {
            //  EKF update step
//            kalman.update(yr.info.x,yr.info.y);

            yr.bRecognized = false;
            emit signal_UHFTransponderRecognized(QString::number(yr.info.x),QString::number(yr.info.y));
        }
*/
    }
}

void Background_Service::sub_loop()
{


}




