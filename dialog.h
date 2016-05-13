#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "background_service.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private:
    Ui::Dialog *ui;

protected:
    Background_Service* service;

public slots:

    void slot_AngularVelocityReceived(AngularVelocity);  //  update each of current angular velocity
    void slot_HeadingAngleReceived(double);              //  update current heading angle
    void slot_UHFTransponderRecognized(QString,QString); //  update latest location of transponder recognized
    void slot_LogMsgOccured(QString);                    //  print log message to 'printLog' UI which is textedit ui

};

#endif // DIALOG_H
