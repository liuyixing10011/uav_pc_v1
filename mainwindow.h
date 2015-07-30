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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QtGlobal>

#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>

#include "mavserialport.h"

QT_BEGIN_NAMESPACE

namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class Console;
class SettingsDialog;
class MavSerialPort;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    enum FLIGHT_MODE {RETURN = 0, MANUAL = 1, ALTCTL = 2, POSCTL = 3, MISSION = 4, LOITER = 5, DELIVERY = 6} flight_mode;
    enum ARM_STATE {DISARMED = 0, ARMED = 1} arm_state;
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);

signals:
    //should look at these two later
    //we can use this directly maybe
    void armingStateChanged(ARM_STATE s);
    void flightModeChanged(FLIGHT_MODE m);

private slots:
    /** Serial port dialog */
    void openSerialPort();
    void closeSerialPort();
    void about();
    void writeData(const QByteArray &data);
    void writeFlightLog();
    void readData();

    void handleError(QSerialPort::SerialPortError error);

    /** Update Info */
    void onUpdateTime();
    void onUpdateLocal();
    void onUpdateGlobal();
    void onUpdateBattery();
    void onUpdateIMU();
    void onUpdateAttitude();

    void onSetX(int x);
    void onSetY(int y);
    void onSetZ(int z);
    void onSetR(int r);
    void resetX();
    void resetY();
    void resetZ();
    void resetR();

   /** Arming state */
   void onSetArming();

   /** Flight mode swtich */
   void onSetFlightMode();

private:
    /** communication */
    void initActionsConnections();
    void initSerialConnections();
    void initUpdateConnections();
    void initCommandConnections();

    /** inside MainWindow */
    void createInfoGroupBox();
    void createControlSlidersGroupBox();
 //   void createMapGroupBox();
    void createFlightModeControlGroupBox();

    /** inside info group */
    void createTimeGroupBox();
    void createLocalGroupBox();
    void createGlobalGroupBox();
    void createBluetoothGroupBox();
    void createBatteryGroupBox();
    void createIMUGroupBox();
    void createAttitudeGroupBox();

    /** inside flight mode control */


private:
    Ui::MainWindow *ui;

    /** Console */
    Console* console;
    SettingsDialog* settings;
    MavSerialPort* serial;

    /** UAV Control main window */
    QWidget* widget;

    QGroupBox* infoGroupBox;
    QGroupBox* controlSlidersGroupBox;
    QGroupBox* flightModeControlGroupBox;
    QGroupBox* consoleGroupBox;

    /** Info: real-time feedback from the UAV */
    QGroupBox* timeGroupBox;
    QGroupBox* localGroupBox;
    QGroupBox* globalGroupBox;
    QGroupBox* bluetoothGroupBox; //it is bluetooth now
    QGroupBox* batteryGroupBox;
    QGroupBox* IMUGroupBox;
    QGroupBox* attitudeGroupBox;

    /** inside timeGroupBox */
    QLabel* timeLabel;//time since boot(ms)

    /** inside localGroupBox */
    QLabel* xPosLabel;
    QLabel* yPosLabel;
    QLabel* zPosLabel;
    QLabel* vxLabel;
    QLabel* vyLabel;
    QLabel* vzLabel;

    /** inside globalGroupBox */
    QLabel* latitudeLabel;
    QLabel* longitudeLabel;
    QLabel* altitudeLabel;

    /** inside IMUgroupBox */
    QLabel* xAcceLabel;
    QLabel* yAcceLabel;
    QLabel* zAcceLabel;
    QLabel* xGyroLabel;
    QLabel* yGyroLabel;
    QLabel* zGyroLabel;
    QLabel* xMagLabel;
    QLabel* yMagLabel;
    QLabel* zMagLabel;
    QLabel* pressure_altitudeLabel;
    QLabel* temperatureLabel;

    /** inside batteryGroupBox */
    //add a low battery alert
    QLabel* voltageLabel;
    QLabel* remainingLabel;

    /** inside altitudeGroupBox */
    QLabel* rollangleLabel;
    QLabel* pitchangleLabel;
    QLabel* yawangleLabel;
    QLabel* rollspeedLabel;
    QLabel* pitchspeedLabel;
    QLabel* yawspeedLabel;

    /** inside bluetoothGroupBox */
    //now it is empty

    /** Manual control sliders */
    QSlider* xSlider;
    QSlider* ySlider;
    QSlider* zSlider;
    QSlider* rSlider;

    QLabel* xLabel;
    QLabel* yLabel;
    QLabel* zLabel;
    QLabel* rLabel;
    QLabel* xValue;
    QLabel* yValue;
    QLabel* zValue;
    QLabel* rValue;

    /** Inside flight mode control */
    QGroupBox* returnSwitch;
    QHBoxLayout* returnLayout;
    QRadioButton* returnOn;
    QRadioButton* returnOff;

    QGroupBox* modeSwitch;
    QHBoxLayout* modeLayout;
    QRadioButton* manualRadioButton;
    QRadioButton* assistRadioButton;
    QRadioButton* autoRadioButton;

    QGroupBox* assistSwitch;
    QHBoxLayout* assistLayout;
    QRadioButton* altctl;
    QRadioButton* posctl;

    QGroupBox* autoSwitch;
    QHBoxLayout* autoLayout;
    QRadioButton* mission;
    QRadioButton* loiter;
    QRadioButton* delivery;

    QPushButton* armButton;
    QLabel* flightModeLabel;

    int x;
    int y;
    int z;
    int r;
};

#endif // MAINWINDOW_H
