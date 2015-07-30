/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia. For licensing terms and
** conditions see http://qt.digia.com/licensing. For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights. These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "console.h"
#include "settingsdialog.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    x(0), y(0), z(0), r(0)
{
    ui->setupUi(this);
    console = new Console;
    console->setEnabled(false);

    serial = new MavSerialPort(this);
    settings = new SettingsDialog(this);

    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionQuit->setEnabled(true);
    ui->actionConfigure->setEnabled(true);
    ui->actionBluetooth->setEnabled(true);
    ui->actionAbout->setEnabled(true);

    initActionsConnections();
    initSerialConnections();

    widget = new QWidget(this);
    QGridLayout* centralLayout = new QGridLayout(widget);

    consoleGroupBox = new QGroupBox(tr("Console"),this);
    QVBoxLayout* consoleLayout = new QVBoxLayout(consoleGroupBox);
    consoleLayout->addWidget(console);
    consoleGroupBox->setLayout(consoleLayout);

    createInfoGroupBox();
    createControlSlidersGroupBox();
    createFlightModeControlGroupBox();

    centralLayout->addWidget(infoGroupBox,0,0);
    centralLayout->addWidget(controlSlidersGroupBox,1,0);
    centralLayout->addWidget(flightModeControlGroupBox,0,1);
    centralLayout->addWidget(consoleGroupBox,1,1);

    initUpdateConnections();
    initCommandConnections();

    widget->setLayout(centralLayout);
    setCentralWidget(widget);

    resize(800,700);
    setWindowTitle(QApplication::translate("toplevel", "Ground Control Station"));
}

/** Manual control sliders */

void MainWindow::onSetX(int t){
    if(t <= 1000 && t>= -1000){
        char xsldvalue[1024];
        sprintf(xsldvalue,"%d",t);
        x = t;
        xValue->setText(xsldvalue);
        xSlider->setValue(t);
        serial->setX(t);
    }
}

void MainWindow::onSetY(int t){
    if(t <= 1000 && t>= -1000){
        char ysldvalue[1024];
        sprintf(ysldvalue,"%d",t);
        y = t;
        yValue->setText(ysldvalue);
        ySlider->setValue(t);
        serial->setY(t);
    }
}

void MainWindow::onSetZ(int t){
    if(t <= 1000 && t>= 0){
        char zsldvalue[1024];
        sprintf(zsldvalue,"%d",t);
        z = t;
        zValue->setText(zsldvalue);
        zSlider->setValue(t);
        serial->setZ(t);
    }
}

void MainWindow::onSetR(int t){
    if(t <= 1000 && t>= -1000){
        char rsldvalue[1024];
        sprintf(rsldvalue,"%d",t);
        r = t;
        rValue->setText(rsldvalue);
        rSlider->setValue(t);
        serial->setR(t);
    }
}

void MainWindow::resetX(){
    onSetX(0);
}

void MainWindow::resetY(){
    onSetY(0);
}

void MainWindow::resetZ(){
    onSetZ(0);
}

void MainWindow::resetR(){
    onSetR(0);
}

/** update Info */
void MainWindow::onUpdateTime(){
    char time[1024];
    sprintf(time,"time since boot: %d ms",serial->attitude.time_boot_ms);
    timeLabel->setText(time);
}

void MainWindow::onUpdateLocal(){
    char xvalue[1024];
    sprintf(xvalue,"xPos: %3.2fm",serial->local_position_ned.x);
    xPosLabel->setText(xvalue);
    char yvalue[1024];
    sprintf(yvalue,"yPos: %3.2fm",serial->local_position_ned.y);
    yPosLabel->setText(yvalue);
    char zvalue[1024];
    sprintf(zvalue,"zPos: %3.2fm",serial->local_position_ned.z);
    zPosLabel->setText(zvalue);

    char vxvalue[1024];
    sprintf(vxvalue,"V_x: %4.3fm/s",serial->local_position_ned.vx);
    vxLabel->setText(vxvalue);
    char vyvalue[1024];
    sprintf(vyvalue,"V_y: %4.3fm/s",serial->local_position_ned.vy);
    vyLabel->setText(vyvalue);
    char vzvalue[1024];
    sprintf(vzvalue,"V_z: %4.3fm/s",serial->local_position_ned.vz);
    vzLabel->setText(vzvalue);
}

void MainWindow::onUpdateGlobal(){
    char lat_value[1024];
    sprintf(lat_value,"Latitude: %3.2f",(float)serial->global_position_int.lat / 1e7);
    latitudeLabel->setText(lat_value);
    char lon_value[1024];
    sprintf(lon_value,"Longitude: %3.2f",(float)serial->global_position_int.lon/1e7);
    longitudeLabel->setText(lon_value);
    char alt_value[1024];
    sprintf(alt_value,"Altitude: %3.2f",(float)serial->global_position_int.alt/1000);
    altitudeLabel->setText(alt_value);
}

void MainWindow::onUpdateBattery(){
    char v_value[1024];
    sprintf(v_value,"Voltage: %d mV",serial->sys_status.voltage_battery);
    voltageLabel->setText(v_value);

    char remaining_value[1024];
    sprintf(remaining_value,"Remaining: %d",serial->sys_status.battery_remaining);
    remainingLabel->setText(remaining_value);
}

void MainWindow::onUpdateIMU(){
    char xacc_value[1024];
    sprintf(xacc_value,"xAcce: %3.2f",serial->highres_imu.xacc);
    xAcceLabel->setText(xacc_value);
    char yacc_value[1024];
    sprintf(yacc_value,"yAcce: %3.2f",serial->highres_imu.yacc);
    yAcceLabel->setText(yacc_value);
    char zacc_value[1024];
    sprintf(zacc_value,"zAcce: %3.2f",serial->highres_imu.zacc);
    zAcceLabel->setText(zacc_value);

    char xgyro_value[1024];
    sprintf(xgyro_value,"xGyro: %3.2f",serial->highres_imu.xgyro);
    xGyroLabel->setText(xgyro_value);
    char ygyro_value[1024];
    sprintf(ygyro_value,"yGyro: %3.2f",serial->highres_imu.ygyro);
    yGyroLabel->setText(ygyro_value);
    char zgyro_value[1024];
    sprintf(zgyro_value,"zGyro: %3.2f",serial->highres_imu.zgyro);
    zGyroLabel->setText(zgyro_value);

    char xmag_value[1024];
    sprintf(xmag_value,"xMag: %3.2f",serial->highres_imu.xmag);
    xMagLabel->setText(xmag_value);
    char ymag_value[1024];
    sprintf(ymag_value,"yMag: %3.2f",serial->highres_imu.ymag);
    yMagLabel->setText(ymag_value);
    char zmag_value[1024];
    sprintf(zmag_value,"zMag: %3.2f",serial->highres_imu.zmag);
    zMagLabel->setText(zmag_value);

    char pa_value[1024];
    sprintf(pa_value,"Pressure_Altitude: %3.2f",serial->highres_imu.pressure_alt);
    pressure_altitudeLabel->setText(pa_value);
    char tem_value[1024];
    sprintf(tem_value,"temperature: %3.2f",serial->highres_imu.temperature);
    temperatureLabel->setText(tem_value);
}

void MainWindow::onUpdateAttitude(){
    char roll_angle[1024];
    sprintf(roll_angle,"Roll: %3.2f",serial->attitude.roll);
    rollangleLabel->setText(roll_angle);
    char pitch_angle[1024];
    sprintf(pitch_angle,"Pitch: %3.2f",serial->attitude.pitch);
    pitchangleLabel->setText(pitch_angle);
    char yaw_angle[1024];
    sprintf(yaw_angle,"Yaw: %3.2f",serial->attitude.yaw);
    yawangleLabel->setText(yaw_angle);

    char roll_speed[1024];
    sprintf(roll_speed,"Roll_speed: %3.2f",serial->attitude.rollspeed);
    rollspeedLabel->setText(roll_speed);
    char  pitch_speed[1024];
    sprintf( pitch_speed,"Pitch_speed: %3.2f",serial->attitude.pitchspeed);
    pitchspeedLabel->setText( pitch_speed);
    char yaw_speed[1024];
    sprintf(yaw_speed,"Yaw_speed: %3.2f",serial->attitude.yawspeed);
    yawspeedLabel->setText(yaw_speed);
}

/** Arming state
 * use this slot to send out serial command
 */

//will change this later
void MainWindow::onSetArming(){

    if(armButton->isChecked()){
        armButton->setText("Armed");
        serial->set_mode_arm();
    }
    else{
        armButton->setText("Disarmed");
        serial->set_mode_disarm();
    }
}

/** Flight mode
 * use this slot to send out serial command
 */
void MainWindow::onSetFlightMode(){
    if(returnOn->isChecked()){
        flight_mode = RETURN;
        flightModeLabel->setText("Flight Mode: Return to Launch");
        modeSwitch->setEnabled(false);
        assistSwitch->setEnabled(false);
        autoSwitch->setEnabled(false);
        serial->set_mode_return();
    }
    else{
        modeSwitch->setEnabled(true);
        if(manualRadioButton->isChecked()){
            flight_mode = MANUAL;
            flightModeLabel->setText("Flight Mode: Manual");
            assistSwitch->setEnabled(false);
            autoSwitch->setEnabled(false);
            serial->set_mode_manual();
        }
        else if(assistRadioButton->isChecked()){
            assistSwitch->setEnabled(true);
            autoSwitch->setEnabled(false);
            if(altctl->isChecked()){
                flight_mode = ALTCTL;
                flightModeLabel->setText("Flight Mode: Altitude Control");
                serial->set_mode_assist_altctl();
            }
            else{
                flight_mode = POSCTL;
                flightModeLabel->setText("Flight Mode: Position Control");
                serial->set_mode_assist_posctl();
            }
        }
        else{
            assistSwitch->setEnabled(false);
            autoSwitch->setEnabled(true);
            if(mission->isChecked()){
                flight_mode = MISSION;
                flightModeLabel->setText("Flight Mode: Auto Mission");
                serial->set_mode_auto_mission();
            }
            else if (loiter->isChecked()) {
                flight_mode = LOITER;
                flightModeLabel->setText("Flight Mode: Auto Loiter");
                serial->set_mode_auto_loiter();
            }
            else {
                flight_mode = DELIVERY;
                flightModeLabel->setText("Flight Mode: Auto Delivery");
                serial->set_mode_auto_delivery();
            }
        }
    }

}

 void MainWindow::createTimeGroupBox(){
     timeGroupBox = new QGroupBox(tr("Real-time Display"),this);
     QHBoxLayout* layout  = new QHBoxLayout(timeGroupBox);

     timeLabel = new QLabel(this);//time since boot(ms)
     timeLabel->setBaseSize(80,30);
     timeLabel->setText("Time since boot(ms): 0");

     layout->addWidget(timeLabel);
     timeGroupBox->setLayout(layout);
 }

 void MainWindow::createLocalGroupBox(){
     localGroupBox = new QGroupBox(tr("Local Position"),this);
     QGridLayout* layout  = new QGridLayout(localGroupBox);

     xPosLabel = new QLabel(this);
     xPosLabel->setBaseSize(50,30);
     xPosLabel->setText("xPos: Unknown");

     yPosLabel = new QLabel(this);
     yPosLabel->setBaseSize(50,30);
     yPosLabel->setText("yPos: Unknown");

     zPosLabel = new QLabel(this);
     zPosLabel->setBaseSize(50,30);
     zPosLabel->setText("zPos: Unknown");

     vxLabel = new QLabel(this);
     vxLabel->setBaseSize(50,30);
     vxLabel->setText("V_x: Unknown");

     vyLabel = new QLabel(this);
     vyLabel->setBaseSize(50,30);
     vyLabel->setText("V_y: Unknown");

     vzLabel = new QLabel(this);
     vzLabel->setBaseSize(50,30);
     vzLabel->setText("V_z: Unknown");

     layout->addWidget(xPosLabel,0,0);
     layout->addWidget(yPosLabel,1,0);
     layout->addWidget(zPosLabel,2,0);
     layout->addWidget(vxLabel,0,1);
     layout->addWidget(vyLabel,1,1);
     layout->addWidget(vzLabel,2,1);

     localGroupBox->setLayout(layout);
 }

 void MainWindow::createGlobalGroupBox(){
     globalGroupBox = new QGroupBox(tr("Global Position"),this);
     QGridLayout* layout  = new QGridLayout(globalGroupBox);

     latitudeLabel = new QLabel(this);
     latitudeLabel->setBaseSize(50,40);
     latitudeLabel->setText("Latitude: Unknown");

     longitudeLabel = new QLabel(this);
     longitudeLabel->setBaseSize(50,40);
     longitudeLabel->setText("Longitude: Unknown");

     altitudeLabel = new QLabel(this);
     altitudeLabel->setBaseSize(50,40);
     altitudeLabel->setText("Altitude: Unknown");

     layout->addWidget(latitudeLabel,0,0);
     layout->addWidget(longitudeLabel,1,0);
     layout->addWidget(altitudeLabel,2,0);

     globalGroupBox->setLayout(layout);
 }
 void MainWindow::createBluetoothGroupBox(){
      bluetoothGroupBox = new QGroupBox(tr("Bluetooth Connection"),this);
      QVBoxLayout* layout  = new QVBoxLayout(bluetoothGroupBox);

      // will add more code here
      //

      bluetoothGroupBox->setLayout(layout);
}


 void MainWindow::createBatteryGroupBox(){
     batteryGroupBox = new QGroupBox(tr("Battery Info"),this);
     QVBoxLayout* layout  = new QVBoxLayout(batteryGroupBox);

     voltageLabel = new QLabel(this);
     voltageLabel->setBaseSize(50,40);
     voltageLabel->setText("Voltage: Unknown");

     remainingLabel= new QLabel(this);
     remainingLabel->setBaseSize(50,40);
     remainingLabel->setText("Remaining: Unknown");

     layout->addWidget(voltageLabel);
     layout->addWidget(remainingLabel);

     batteryGroupBox->setLayout(layout);
 }

 void MainWindow::createIMUGroupBox(){
     IMUGroupBox = new QGroupBox(tr("High Resolution IMU"),this);
     QGridLayout* layout  = new QGridLayout(IMUGroupBox);

     xAcceLabel = new QLabel(this);
     xAcceLabel->setBaseSize(50,40);
     xAcceLabel->setText("xAcce: Unknown");

     yAcceLabel = new QLabel(this);
     yAcceLabel->setBaseSize(50,40);
     yAcceLabel->setText("yAcce: Unknown");

     zAcceLabel = new QLabel(this);
     zAcceLabel->setBaseSize(50,40);
     zAcceLabel->setText("zAcce: Unknown");

     xGyroLabel = new QLabel(this);
     xGyroLabel->setBaseSize(50,40);
     xGyroLabel->setText("xGyro: Unknown");

     yGyroLabel = new QLabel(this);
     yGyroLabel->setBaseSize(50,40);
     yGyroLabel->setText("yGyro: Unknown");

     zGyroLabel = new QLabel(this);
     zGyroLabel->setBaseSize(50,40);
     zGyroLabel->setText("zGyro: Unknown");

     xMagLabel = new QLabel(this);
     xMagLabel->setBaseSize(50,40);
     xMagLabel->setText("xMag: Uknown");

     yMagLabel = new QLabel(this);
     yMagLabel->setBaseSize(50,40);
     yMagLabel->setText("yMag: Unknown");

     zMagLabel = new QLabel(this);
     zMagLabel->setBaseSize(50,40);
     zMagLabel->setText("zMag: Unknown");

     pressure_altitudeLabel = new QLabel(this);
     pressure_altitudeLabel->setBaseSize(50,40);
     pressure_altitudeLabel->setText("Pressure_Altitude: Uknown");

     temperatureLabel = new QLabel(this);
     temperatureLabel->setBaseSize(50,40);
     temperatureLabel->setText("temperature: Unknown");

     layout->addWidget(xAcceLabel,0,0);
     layout->addWidget(yAcceLabel,1,0);
     layout->addWidget(zAcceLabel,2,0);
     layout->addWidget(xGyroLabel,3,0);
     layout->addWidget(yGyroLabel,4,0);
     layout->addWidget(zGyroLabel,5,0);
     layout->addWidget(xMagLabel,0,1);
     layout->addWidget(yMagLabel,1,1);
     layout->addWidget(zMagLabel,2,1);
     layout->addWidget(pressure_altitudeLabel,3,1);
     layout->addWidget(temperatureLabel,4,1);
     IMUGroupBox->setLayout(layout);
 }

 void MainWindow::createAttitudeGroupBox(){
     attitudeGroupBox = new QGroupBox(tr("Attitude"),this);
     QGridLayout* layout  = new QGridLayout(attitudeGroupBox);

     rollangleLabel = new QLabel(this);
     rollangleLabel->setBaseSize(50,40);
     rollangleLabel->setText("Roll: Unknown");

     pitchangleLabel = new QLabel(this);
     pitchangleLabel->setBaseSize(50,40);
     pitchangleLabel->setText("Pitch: Unknown");

     yawangleLabel = new QLabel(this);
     yawangleLabel->setBaseSize(50,40);
     yawangleLabel->setText("Yaw: Unknown");

     rollspeedLabel = new QLabel(this);
     rollspeedLabel->setBaseSize(50,40);
     rollspeedLabel->setText("Roll_speed: Unknown");

     pitchspeedLabel = new QLabel(this);
     pitchspeedLabel->setBaseSize(50,40);
     pitchspeedLabel->setText("Pitch_speed: Unknown");

     yawspeedLabel = new QLabel(this);
     yawspeedLabel->setBaseSize(50,40);
     yawspeedLabel->setText("Yaw_speed: Unknown");

     layout->addWidget(rollangleLabel,0,0);
     layout->addWidget(pitchangleLabel,1,0);
     layout->addWidget(yawangleLabel,2,0);
     layout->addWidget(rollspeedLabel,3,0);
     layout->addWidget(pitchspeedLabel,4,0);
     layout->addWidget(yawspeedLabel,5,0);

     attitudeGroupBox->setLayout(layout);
 }

void MainWindow::createInfoGroupBox(){
    infoGroupBox = new QGroupBox(tr("Position"),this);
    QGridLayout* layout = new QGridLayout(infoGroupBox);

    createTimeGroupBox();
    createLocalGroupBox();
    createBluetoothGroupBox();
    createBatteryGroupBox();
    createIMUGroupBox();
    createAttitudeGroupBox();

    layout->addWidget(timeGroupBox,0,0,1,2);
    layout->addWidget(localGroupBox,1,0,3,2);
  //layout->addWidget(globalGroupBox,1,1,3,1);
    layout->addWidget(bluetoothGroupBox,0,2,2,1);
    layout->addWidget(batteryGroupBox,2,2,2,1);
    layout->addWidget(IMUGroupBox,4,0,4,2);
    layout->addWidget(attitudeGroupBox,4,2,4,1);

    infoGroupBox->setLayout(layout);
    infoGroupBox->setAlignment(Qt::AlignHCenter);
}

void MainWindow::createControlSlidersGroupBox(){

    controlSlidersGroupBox = new QGroupBox(tr("Control"),this);
    QGridLayout* layout = new QGridLayout(controlSlidersGroupBox);

    //this is the start position of send manual control

    xSlider = new QSlider(Qt::Horizontal,this);
    xSlider->setRange(-1000,1000);
    xSlider->setValue(0);
    xSlider->setBaseSize(300,30);

    ySlider = new QSlider(Qt::Horizontal,this);
    ySlider->setRange(-1000,1000);
    ySlider->setValue(0);
    ySlider->setBaseSize(300,30);

    zSlider = new QSlider(Qt::Horizontal,this);
    zSlider->setRange(0,1000);
    zSlider->setValue(0);
    zSlider->setBaseSize(300,30);

    rSlider = new QSlider(Qt::Horizontal,this);
    rSlider->setRange(-1000,1000);
    rSlider->setValue(0);
    rSlider->setBaseSize(300,30);

    // this is the end of send manual control

    xLabel = new QLabel(tr("back-forward"),this);
    xLabel->setBaseSize(50,10);
    xValue = new QLabel(tr("0"),this);
    xValue->setBaseSize(50,10);
    yLabel = new QLabel(tr("left-right"),this);
    yLabel->setBaseSize(50,10);
    yValue = new QLabel(tr("0"),this);
    yValue->setBaseSize(50,10);
    zLabel = new QLabel(tr("throttle"),this);
    zLabel->setBaseSize(50,10);
    zValue = new QLabel(tr("0"),this);
    zValue->setBaseSize(50,10);
    rLabel = new QLabel(tr("yaw"),this);
    rLabel->setBaseSize(50,10);
    rValue = new QLabel(tr("0"),this);
    rValue->setBaseSize(50,10);

    connect(xSlider, SIGNAL(valueChanged(int)),this,SLOT(onSetX(int)));
    connect(ySlider, SIGNAL(valueChanged(int)),this,SLOT(onSetY(int)));
    connect(zSlider, SIGNAL(valueChanged(int)),this,SLOT(onSetZ(int)));
    connect(rSlider, SIGNAL(valueChanged(int)),this,SLOT(onSetR(int)));

    layout->addWidget(xValue,0,2);
    layout->addWidget(yValue,1,2);
    layout->addWidget(zValue,2,2);
    layout->addWidget(rValue,3,2);

    layout->addWidget(xSlider,0,1);
    layout->addWidget(ySlider,1,1);
    layout->addWidget(zSlider,2,1);
    layout->addWidget(rSlider,3,1);

    layout->addWidget(xLabel,0,0);
    layout->addWidget(yLabel,1,0);
    layout->addWidget(zLabel,2,0);
    layout->addWidget(rLabel,3,0);

    controlSlidersGroupBox->setLayout(layout);
    controlSlidersGroupBox->setAlignment(Qt::AlignHCenter);
}

void MainWindow::createFlightModeControlGroupBox(){
    flightModeControlGroupBox = new QGroupBox(tr("Flight Mode Control"),this);
    QVBoxLayout* layout = new QVBoxLayout(flightModeControlGroupBox);

    returnSwitch = new QGroupBox(tr("Return"),this);
    returnLayout = new QHBoxLayout(returnSwitch);
    returnOn = new QRadioButton("Return On",this);
    returnOff = new QRadioButton("Other Modes",this);
    returnOff->setChecked(true);

    returnLayout->addWidget(returnOn);
    returnLayout->addWidget(returnOff);
    returnSwitch->setLayout(returnLayout);

    modeSwitch = new QGroupBox(tr("Mode"),this);
    modeSwitch->setEnabled(true);

    modeLayout = new QHBoxLayout(modeSwitch);
    manualRadioButton = new QRadioButton("Manual",this);
    assistRadioButton = new QRadioButton("Assist",this);
    autoRadioButton = new QRadioButton("Auto",this);
    manualRadioButton->setChecked(true);
    modeLayout->addWidget(manualRadioButton);
    modeLayout->addWidget(assistRadioButton);
    modeLayout->addWidget(autoRadioButton);
    modeSwitch->setLayout(modeLayout);

    assistSwitch = new QGroupBox(tr("Assist"),this);
    assistSwitch->setEnabled(false);

    assistLayout = new QHBoxLayout(assistSwitch);
    altctl = new QRadioButton("Altctl",this);
    posctl = new QRadioButton("Posctl",this);
    posctl->setChecked(true);
    assistLayout->addWidget(altctl);
    assistLayout->addWidget(posctl);
    assistSwitch->setLayout(assistLayout);

    autoSwitch = new QGroupBox(tr("Auto"),this);
    autoSwitch->setEnabled(false);

    autoLayout = new QHBoxLayout(autoSwitch);
    mission = new QRadioButton("Mission",this);
    loiter = new QRadioButton("Loiter",this);
    delivery = new QRadioButton("Delivery", this);
    loiter->setChecked(true);
    autoLayout->addWidget(mission);
    autoLayout->addWidget(loiter);
    autoLayout->addWidget(delivery);
    autoSwitch->setLayout(autoLayout);

    flightModeLabel = new QLabel(this);
    flightModeLabel->setText("Flight Mode: Standby" );

    armButton = new QPushButton(tr("Disarmed"), this);
    armButton->setCheckable(true);
    armButton->setChecked(false);

    layout->addWidget(returnSwitch);
    layout->addWidget(modeSwitch);
    layout->addWidget(assistSwitch);
    layout->addWidget(autoSwitch);
    layout->addWidget(flightModeLabel);
    layout->addWidget(armButton);

    connect(returnOn,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(returnOff,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(manualRadioButton,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(assistRadioButton,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(autoRadioButton,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(altctl,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(posctl,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(mission,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));
    connect(loiter,SIGNAL(clicked()),this,SLOT(onSetFlightMode()));

    connect(armButton, SIGNAL(clicked()), this, SLOT(onSetArming()));

    flightModeControlGroupBox->setLayout(layout);
    flightModeControlGroupBox->setAlignment(Qt::AlignHCenter);
}

MainWindow::~MainWindow()
{
    delete settings;
    delete ui;
}


void MainWindow::openSerialPort()
{
    SettingsDialog::Settings p = settings->settings();
    //mac uses absolute address
    //windows uses relative address
    serial->setPortName("/dev/cu."+p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    serial->timer->start(200);
    if (serial->open(QIODevice::ReadWrite)) {
            console->setEnabled(true);
            console->setLocalEchoEnabled(p.localEchoEnabled);
            ui->actionConnect->setEnabled(false);
            ui->actionDisconnect->setEnabled(true);
            ui->actionConfigure->setEnabled(false);
            ui->statusBar->showMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                                       .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                                       .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
        serial->timer->stop();
        ui->statusBar->showMessage(tr("Open error"));
    }
}


void MainWindow::closeSerialPort()
{
    if (serial->isOpen())
        serial->close();
    console->setEnabled(false);
    serial->timer->stop();
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionConfigure->setEnabled(true);
    ui->statusBar->showMessage(tr("Disconnected"));
}


void MainWindow::about()
{  //"The <b>UAV Control v1.0</b> is developed by "
    QMessageBox::about(this, tr("About UAV Control Version 1.0"),
                       tr("\tUAV Ground Control 1.0\n"
                          "\tdeveloped on 2015 summer\n"
                          "\tby Harvard-HKUST Research Team"));
}


void MainWindow::writeData(const QByteArray &data)
{
  //  serial->write(data);
 //   serial->send_test_urob();
  //  serial->send_manual_setpoint();
 //   serial->cmd_do_set_mode();//MAV_MODE_STABILIZE_ARMED);
  //  serial->send_manual_control();
}

void MainWindow::writeFlightLog(){
    console->putData(serial->statustext.text);
}


void MainWindow::readData()
{
    QByteArray data = serial->readAll();
    serial->mavRead(&data);
}


void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
        closeSerialPort();
    }
}


void MainWindow::initActionsConnections()
{
    connect(ui->actionConnect, SIGNAL(triggered()), this, SLOT(openSerialPort()));
    connect(ui->actionDisconnect, SIGNAL(triggered()), this, SLOT(closeSerialPort()));
    connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->actionConfigure, SIGNAL(triggered()), settings, SLOT(show()));
    connect(ui->actionClear, SIGNAL(triggered()), console, SLOT(clear()));
    connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(about()));
    connect(ui->actionAboutQt, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(ui->actionBluetooth, SIGNAL(triggered()), settings, SLOT(show()));
    connect(ui->actionLogo, SIGNAL(triggered()), this, SLOT(about()));
}

void MainWindow::initSerialConnections(){

    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
        SLOT(handleError(QSerialPort::SerialPortError)));
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    connect(console, SIGNAL(getData(QByteArray)), this, SLOT(writeData(QByteArray)));
    connect(serial, SIGNAL(flightLogReady()), this, SLOT(writeFlightLog()));
}

void MainWindow::initUpdateConnections(){
    connect(serial,SIGNAL(timeChanged()),this,SLOT(onUpdateTime()));
    connect(serial,SIGNAL(localChanged()),this,SLOT(onUpdateLocal()));
    //will do something with the global
    //will do something with the battery
 //   connect(serial,SIGNAL(globalChanged()),this,SLOT(onUpdateGlobal()));
    connect(serial,SIGNAL(batteryChanged(int)),this,SLOT(onUpdateBattery()));
    connect(serial,SIGNAL(IMUChanged()),this,SLOT(onUpdateIMU()));
    connect(serial,SIGNAL(attitudeChanged()),this,SLOT(onUpdateAttitude()));
}

void MainWindow::initCommandConnections(){
 //   connect(this,SIGNAL(armingStateChanged(ARM_STATE)),this,SLOT());
 //   connect(this,SIGNAL(flightModeChanged(FLIGHT_MODE)),serial,SLOT());
  //  connect(toggleButton,SIGNAL(clicked()),serial,SLOT(send_manual_setpoint()));
}

void MainWindow::keyPressEvent(QKeyEvent *event){
    switch(event->key()){
    case Qt::Key_Up:
        onSetX(x + 20);
        break;
    case Qt::Key_Down:
        onSetX(x - 20);
        break;
    case Qt::Key_Left:
        onSetY(y - 20);
        break;
    case Qt::Key_Right:
        onSetY(y + 20);
        break;
    case Qt::Key_W:
        onSetZ(z + 20);
        break;
    case Qt::Key_S:
        onSetZ(z - 20);
        break;
    case Qt::Key_A:
        onSetR(r - 20);
        break;
    case Qt::Key_D:
        onSetR(r + 20);
    case Qt::Key_Escape: break;
    }
}

 void MainWindow::keyReleaseEvent(QKeyEvent* event){
     switch (event->key()) {
     case Qt::Key_Up:
         resetX();
         break;
     case Qt::Key_Down:
         resetX();
         break;
     case Qt::Key_Left:
         resetY();
         break;
     case Qt::Key_Right:
         resetY();
         break;
     case Qt::Key_A:
         resetR();
         break;
     case Qt::Key_D:
         resetR();
         break;
     default:
         break;
     }
 }


//void MainWindow::keyReleaseEvent(QKeyEvent* event);
