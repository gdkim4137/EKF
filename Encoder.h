#ifndef ENCODER_H
#define ENCODER_H

#include <Sensor.h>
#include <QtCore>

class AngularVelocity
{
public:
    int left_top;
    int left_bottom;
    int right_top;
    int right_bottom;
};

class Encoder : public Sensor
{

protected:
    std::string raw_packet;	//	not yet processed packet when data is not received compltely for meaningful packet

    //	std::string head_info;
    std::string header_packet;	//	which packet lead to location of transponder

public:
    AngularVelocity info;
    bool bReceived;

public:
    Encoder()
    {
        char locate[] = { 0x55, 0x55 };
        for (char c : locate)
        {
            header_packet.push_back(c);
        }

        bReceived = false;
    }

public:
    void send_pose(std::string command)
    {
        send_message(command);
    }

protected:
    virtual void parser(std::string packet)
    {
        //	receiving data from motor encoder
        raw_packet += packet;

        std::cout << "parser loop()" <<std::endl;

        if( raw_packet.length() > 24)
        {
            std::cout << "decoding..." <<std::endl;
            auto pos_encoder = raw_packet.find(header_packet);

            //  decoding

            //  left top wheel encoder

            info.left_top  = (raw_packet[pos_encoder + 7] << 24) |
                             (raw_packet[pos_encoder + 8] << 16) |
                             (raw_packet[pos_encoder + 9] << 8)  |
                             (raw_packet[pos_encoder + 10]);

            //  right top wheel encoder
            info.right_top = (raw_packet[pos_encoder + 11] << 24) |
                             (raw_packet[pos_encoder + 12] << 16) |
                             (raw_packet[pos_encoder + 13] << 8)  |
                             (raw_packet[pos_encoder + 14]);

            //  left bottom wheel encoder
            info.left_bottom  = (raw_packet[pos_encoder + 15] << 24) |
                                (raw_packet[pos_encoder + 16] << 16) |
                                (raw_packet[pos_encoder + 17] << 8)  |
                                (raw_packet[pos_encoder + 18]);

            //  right bottom wheel encoder
            info.right_bottom = (raw_packet[pos_encoder + 19] << 24) |
                                (raw_packet[pos_encoder + 20] << 16) |
                                (raw_packet[pos_encoder + 21] << 8)  |
                                (raw_packet[pos_encoder + 22]);

            //  erase buffer
            raw_packet.erase(raw_packet.begin(), raw_packet.begin()+pos_encoder+20);

            QMutex mutex;
            mutex.lock();
            bReceived = true;
            mutex.unlock();

        }

    }
};

#endif // ENCODER_H
