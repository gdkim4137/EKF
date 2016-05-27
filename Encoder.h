#ifndef ENCODER_H
#define ENCODER_H

#include <Sensor.h>
#include <QtCore>

#define ERR_GAP 10
class AngularVelocity
{
public:
    AngularVelocity()
    {
        insert(0,0,0,0,0);
    }

    AngularVelocity(const AngularVelocity& av)
    {
        key_pose = av.key_pose;
        left_top = av.left_top;
        right_top = av.right_top;
        left_bottom = av.left_bottom;
        right_bottom = av.right_bottom;
    }
public:
    void operator=(AngularVelocity av)
    {
        key_pose = av.key_pose;
        left_top = av.left_top;
        right_top = av.right_top;
        left_bottom = av.left_bottom;
        right_bottom = av.right_bottom;
    }

    AngularVelocity operator-(AngularVelocity& av)
    {
        AngularVelocity diff;
        diff.insert(this->key_pose,this->left_top - av.left_top,
                                    this->right_top - av.right_top,
                                    this->left_bottom -av.left_bottom,
                                    this->right_bottom - av.right_bottom);
        return diff;
    }

    void insert(int kp, int lt, int lb, int rt, int rb)
    {
        key_pose = kp;
        left_top = lt;
        right_top = rt;
        left_bottom = lb;
        right_bottom = rb;
    }

public:
    int key_pose;

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
    AngularVelocity diff;
    AngularVelocity curr;
    AngularVelocity prev;

    bool bReceived;

    bool isFirst;

public:
    Encoder()
    {
        char locate[] = { 0x55, 0x55 };
        for (char c : locate)
        {
            header_packet.push_back(c);
        }
        bReceived = false;
        isFirst = true;
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

        auto pos_encoder = raw_packet.find(header_packet);

        if(  pos_encoder == std::string::npos)
            return;

        if( raw_packet.length() > pos_encoder + 23 )
        {
         //   std::cout << "loop check(start)"<<std::endl;
            //  key pose
            int key_pose  = (raw_packet[pos_encoder + 2] << 24)  |
                             (raw_packet[pos_encoder + 3] << 16) |
                             (raw_packet[pos_encoder + 4] << 8)  |
                             (raw_packet[pos_encoder + 5]);

            //  left top wheel encoder
            int left_top  = (raw_packet[pos_encoder + 6] << 24)  |
                             (raw_packet[pos_encoder + 7] << 16) |
                             (raw_packet[pos_encoder + 8] << 8)  |
                             (raw_packet[pos_encoder + 9]);

            //  right top wheel encoder
            int right_top = (raw_packet[pos_encoder + 10] << 24)  |
                             (raw_packet[pos_encoder + 11] << 16) |
                             (raw_packet[pos_encoder + 12] << 8)  |
                             (raw_packet[pos_encoder + 13]);

            //  left bottom wheel encoder
            int left_bottom  = (raw_packet[pos_encoder + 14] << 24)  |
                                (raw_packet[pos_encoder + 15] << 16) |
                                (raw_packet[pos_encoder + 16] << 8)  |
                                (raw_packet[pos_encoder + 17]);

            //  right bottom wheel encoder
            int right_bottom = (raw_packet[pos_encoder + 18] << 24) |
                                (raw_packet[pos_encoder + 19] << 16) |
                                (raw_packet[pos_encoder + 20] << 8)  |
                                (raw_packet[pos_encoder + 21]);

            if(isFirst == true)
            {
                curr.insert(key_pose,left_top,left_bottom,right_top,right_bottom);
                prev = curr;
                isFirst = false;
            }
            else
            {
                curr.insert(key_pose,left_top,left_bottom,right_top,right_bottom);
                diff = curr - prev;

                if(diff.left_top > ERR_GAP )      diff.left_top = curr.left_top = prev.left_top;
                if(diff.right_top > ERR_GAP )     diff.right_top = curr.right_top = prev.right_top;
                if(diff.left_bottom > ERR_GAP )   diff.left_bottom = curr.left_bottom = prev.left_bottom;
                if(diff.right_bottom > ERR_GAP )  diff.right_bottom = curr.right_bottom = prev.right_bottom;

            }


            //  erase buffer
            raw_packet.erase(raw_packet.begin(), raw_packet.begin()+pos_encoder+24);

            QMutex mutex;
            mutex.lock();
            bReceived = true;
            mutex.unlock();

           //std::cout << info_curr.left_top <<" "<< info_curr.right_top <<" "<< info_curr.left_bottom <<" "<< info_curr.right_bottom << std::endl;

           //std::cout << "loop check(end)"<<std::endl;

        }

    }
};

#endif // ENCODER_H
