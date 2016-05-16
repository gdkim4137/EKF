#include "background_service.h"

Background_Service::Background_Service(QObject *parent) :
    QThread(parent)
{
    mStop = false;


}

bool Background_Service::device_setup()
{
    bool check = false;

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

    //  Encoder
    if(encoder.start("/dev/ttyAMA0",115200)==false)
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
    while(true)
    {
        this->msleep(10);
        yr.request_rssi();

        /*
        //  send pose to vehicle
        this->msleep(10);
        if( encoder.bReceived == false)
        {
            char command[] = { 0x55, 0x55,
                               0x00,0x00,0x00,0x00, //  key pose
                               0x00,0x00,0x00,0x01, //  x location
                               0x00,0x00,0x00,0x02, //  y location
                               0x00,0x00,0x00,0x03, //  w (heading angle)
                               0x00,0x00,0x00,0x03, //  w (heading angle)
                               0xAA, 0xAA };

            std::string command_string = command;
            encoder.send_pose(command_string);
        }
        */

        if( encoder.bReceived == true)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //              should be changed
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            //  EKF prediction step
            if(kalman.firstTime)    break;
     //       else                    kalman.predict();

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
            /*
            if(e.yaw<180 && e.yaw > -180)
            {
                //  initial angle
                sensor_count++;
                emit signal_HeadingAngleReceived(e.yaw);
            }
            else
            {
                emit signal_HeadingAngleReceived(e.yaw);
            }
            */
        }
        if(yr.bRecognized)
        {
            //  EKF update step
            kalman.update(yr.info.x,yr.info.y);

            yr.bRecognized = false;
            emit signal_UHFTransponderRecognized(QString::number(yr.info.x),QString::number(yr.info.y));
        }
    }
}






