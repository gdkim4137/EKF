#include"Sensor.h"
#include"background_service.h"

class Background_Service;

#ifndef YR9010_H
#define YR9010_H

class Transponder
{
public:
    unsigned int x;
    unsigned int y;
public:
    int rssi;
};


class YR9010 : public Sensor
{
protected:
    std::string raw_packet;	//	not yet processed packet when data is not received compltely for meaningful packet

    void(Background_Service::*data_callback)(Transponder& tr);
    void* data_callback_context;

    //	std::string head_info;
    std::string location_tag;	//	which packet lead to location of transponder
    std::string header_packet;

public:
    Transponder info;
    bool bRecognized;

public:
    YR9010()
    {
        char locate[] = { 0x30, 0x00, 0x00 };
        for (char c : locate)
        {
            location_tag.push_back(c);
        }


        bRecognized = false;

    }
    void register_data_callback(void(Background_Service::*callback)(Transponder& tr)) {

     //   data_callback_context = callback_context;
        data_callback = callback;
    }

    void request_rssi()
    {
        //	only one
        char command[] = { 0xA0, 0x04, 0x01, 0x89, 0x01, 0xD1 };
        //	request 255 times
        //	char command[] = { 0xA0, 0x04, 0x01, 0x89, 0xff, 0xD3 };

        std::string command_string = command;
        send_message(command_string);

    }

public:
    void request_without_rssi()
    {
        char command[] = { 0xA0, 0x0F, 0x01, 0x68, 232 };
        std::string command_string = command;
        send_message(command_string);

        //std::cout << "check sum is " << (int)CheckSum((unsigned char*)command, 4) << std::endl;

    }
public:
    unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
    {
        unsigned char i, uSum = 0;
        for (i = 0; i<uBuffLen; i++)
        {
            uSum = uSum + uBuff[i];
        }
        uSum = (~uSum) + 1;
        return uSum;
    }
    void send_pose(std::string command)
  {
        send_message(command);
    }

protected:
    virtual void parser(std::string packet)
    {


        //	transponder의 응답정보가 들어오는 루프.
        //
      //  std::cout << "received" << std::endl;
        //	패킷은 언제 어느순간 어디서 끊겨 들어올지 모르므로 다 받아둠.
        //
        raw_packet += packet;
        std::cout << raw_packet.c_str()<<std::endl;

        //  받은 패킷들로부터 실제로 요청한 정보에대한 내용이 존재하는지 검색.(rssi, EPC)
        auto pos_location = raw_packet.find(location_tag);
        if (pos_location != std::string::npos && raw_packet.size() > pos_location + 14)
        {
            //	태그에 대한 위치정보와 RSSI가 들어온 경우.
            //	+3 ~ +4 : location information of Transponder in World frame
            info.x = static_cast<int>(raw_packet[pos_location + 3]);
            info.y = static_cast<int>(raw_packet[pos_location + 4]);

            //	+14	:	Received Signal Strength Indication
            info.rssi = static_cast<int>(raw_packet[pos_location + 14]);

            //	인식된 태그에 대한 내용 삭제.
            raw_packet.erase(raw_packet.begin(), raw_packet.begin() + pos_location + 14);

            std::cout << info.x <<std::endl;
            QMutex mutex;
            mutex.lock();
            bRecognized = true;
            mutex.unlock();
        }

    }
};

#endif YR9010_H
