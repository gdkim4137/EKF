#include "Serial.h"
#include<thread>
#include<mutex>
#include<chrono>
#include<deque>

#ifndef	SENSOR_H
#define SENSOR_H

class Sensor
{
public:
    Serial::SerialPort serial;

    //	센서로부터 가공되지 않은 패킷을 받는 루틴.
    //  가공되지 않은 데이터를 사용할 수 있는 데이터로 변경
    std::thread thread_receiver;
    std::mutex mutex_communication;

protected:
    //	is operating thread?
    bool activate_thread_receiver;
    bool activate_thread_event;

public:
    //	가공된 데이터를 전달 받을 콜백함수를 등록.
    bool start(std::string port_name, int baudrate = -1)
    {
        std::lock_guard<std::mutex> _l(mutex_communication);

        if (port_name == "" || baudrate < 0)
            return false;
        else
        {
            if (serial.Open(port_name.c_str(), baudrate) == false)
                return false;
        }

        serial.Flush();

        if (activate_thread_receiver == false)
        {
            activate_thread_receiver = true;
            thread_receiver = std::thread(thread_proc_receiver, this);
        }

        return true;
    }
    void stop()
    {
        std::lock_guard<std::mutex> _l(mutex_communication);
        //	receiver thread exit
        activate_thread_receiver = false;
        thread_receiver.join();
        thread_receiver.detach();
    }

protected:
    //	packet 분석
    virtual void parser(std::string packet) = 0;

protected:
    //	이 클래스를 상속받는 자식클래들은 이 함수의 호출을 받는다.
    //  이렇게 구현한 까닭은 각 센서마다 통신하는 패킷이 다르기 때문에
    //  센서별로 정의를 필요로 하기 때문이다.
    //	패킷의 유효성 검사또한 자식 클래스에서 해주어야 할 것이다.
    static void thread_proc_receiver(void* arg)
    {
        //	이 스레드 루프는 sensor class를 상속받는
        //  모든 클래스들의 데이터가 입력될때마다 호출된다.
        //  이 루프에서 다시 자식 혹은 클래스의 함수가 호출되어 분리된다.
        ((Sensor*)arg)->proc_receiver();
    }
    void proc_receiver()
    {
        int len;
        unsigned char buffer[1024];
        while (activate_thread_receiver)
        {
            memset(buffer, 0, sizeof(buffer) - 1);
            len = serial.Read(buffer, sizeof(buffer) - 1);
            if (len == 0)
            {
                sleep(1);
                //Platform::msleep(1);
                continue;
            }
            else if (len > 0)
            {
                std::string received;

                for (int i = 0; i < len; ++i)
                    received.push_back(buffer[i]);

                this->parser(received);
            }
        }
    }
    void send_message(std::string command)
    {
        //	lock을걸고 보냄.
        std::lock_guard<std::mutex> _l(mutex_communication);
        if (serial.Write((unsigned char*)command.c_str(), command.length()) <= 0)
        {
            //	디버그 정보....
        }
    }

};
#endif

