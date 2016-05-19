#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    service = new Background_Service(this);

    //  connect signals and slots to display information
    connect(service,SIGNAL(signal_AngularVelocityReceived(AngularVelocity)),this,SLOT(slot_AngularVelocityReceived(AngularVelocity)));
    connect(service,SIGNAL(signal_HeadingAngleReceived(double)),this,SLOT(slot_HeadingAngleReceived(double)));
    connect(service,SIGNAL(signal_LogMsgOccured(QString)),this,SLOT(slot_LogMsgOccured(QString)));
    connect(service,SIGNAL(signal_UHFTransponderRecognized(QString,QString)),this,SLOT(slot_UHFTransponderRecognized(QString,QString)));

    service->start();

}

Dialog::~Dialog()
{
    delete ui;
}


void Dialog::slot_AngularVelocityReceived(AngularVelocity av)
{
    //  odometry update and display ( kalman filter prediction step )

    ui->av1->setText(QString::number(av.left_top));
    ui->av2->setText(QString::number(av.right_bottom));
    ui->av3->setText(QString::number(av.left_bottom));
    ui->av4->setText(QString::number(av.right_bottom));
}

void Dialog::slot_HeadingAngleReceived(double val)
{
    //  heading angle update and display
    ui->heading_angle->setText(QString::number(val));
    //ui->printLog->appendPlainText("Heading angle : "+QString::number(val));
}

void Dialog::slot_LogMsgOccured(QString str)
{

    ui->printLog->appendPlainText(str);
}

void Dialog::slot_UHFTransponderRecognized(QString str1, QString str2)
{
    //  pose update and display ( kalman filter prediction step )
    ui->x->setText(str1);
    ui->y->setText(str2);
}
