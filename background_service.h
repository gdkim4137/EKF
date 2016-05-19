#ifndef BACKGROUND_SERVICE_H
#define BACKGROUND_SERVICE_H

#include <QtCore>
#include <QThread>      //  for QThread to emit singnals to announce main UI thread
#include <thread>       //  for sensing which is listening for other sensor
#include "YR9010.h"
#include "myahrs_plus.hpp"
#include "Encoder.h"
#include "EKF_algorithm.h"


class pose
{

public:
    double x;
    double y;
    double w;
};


class Background_Service : public QThread
{
    Q_OBJECT

public:
    explicit Background_Service(QObject *parent = 0);

    void run();
    bool mStop;

    void sub_loop();

public:
    YR9010 yr;
    WithRobot::MyAhrsPlus sensor;
    Encoder encoder;
    EKF kalman;


public:
    bool device_setup(void);

public:
 //   void callback_yr9010(Transponder& tr);

signals:
    void signal_AngularVelocityReceived(AngularVelocity);   //  update each of current angular velocity
    void signal_HeadingAngleReceived(double);               //  update current heading angle
    void signal_UHFTransponderRecognized(QString,QString);  //  update latest location of transponder recognized
    void signal_LogMsgOccured(QString);                     //  print log message to 'printLog' UI which is textedit ui

public slots:

};

#endif // BACKGROUND_SERVICE_H
