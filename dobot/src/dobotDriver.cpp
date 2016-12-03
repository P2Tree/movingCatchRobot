/*******************************************
 * 
 * @file    dobotDriver.c
 * @brief   dobot arm-robot driver file
 * @author  Yang Liuming <dicksonliuming@gmail.com>
 * @date    2016-11-07
 * 
 * ****************************************/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <math.h>

#include "ros/ros.h"
#include "dobot/DobotPoseMsg.h"
#include "dobotDriver.hpp"

using namespace std;

/********************
 * 
 *  DEFINE
 *  
 *  */
#define PI      3.141593
#define DEBUG

#ifdef DEBUG
// #define DEBUG_ORIGIN
#endif

/*******************
 * 
 *  PRIVATE STATIC ARGUMENTS
 *
 * */

// zero arguments: zero point of software
// this values is the world value
// zero values can be changed by rosparams server
float DobotDriver::zeroX = 200.0;
float DobotDriver::zeroY = 0.0;
float DobotDriver::zeroZ = -35;
float DobotDriver::zeroR = 0;

// clamp arguments: limited of space margin
// this values is the world value
// limited values can be changed by rosparams server
float DobotDriver::MaxX = 300;
float DobotDriver::MinX = 200;
float DobotDriver::MaxY = 114;
float DobotDriver::MinY = -114;
float DobotDriver::MaxZ = 65;
float DobotDriver::MinZ = -35;
float DobotDriver::MaxR = 135;
float DobotDriver::MinR = -135;

float DobotDriver::MaxRadius = 300;
float DobotDriver::MinRadius = 200;
float DobotDriver::MaxTheta = 120;
float DobotDriver::MinTheta = -120;

/******************
 * 
 *  PRIVATE METHODS
 *
 * */
DobotDriver::DobotDriver() : uartPort("/dev/dobot"), uartBaud(115200){
    if ( uartInit() )
        exit(-2);
    FullPose_t absPose;
    getCurrentPose(absPose);
    cout << "INFO: boot position(absolute): ";
    cout << "( " << absPose.x << ", " << absPose.y << ", " << absPose.z << ", " << absPose.r << ")" << endl;
    // INFO("boot position(absolute): ( %02f, %02f, %02f, %02f )", absPose.x, absPose.y, absPose.z, absPose.r);
}

DobotDriver::DobotDriver(ros::NodeHandle node) : uartPort("/dev/dobot"), uartBaud(115200){
    // setInit function will get zero values and limited values from rosparams server at this time
    if (setInit(node)) {
        perror("ERR: set init values wrong");
        exit(-1);
    }
    if ( uartInit() )
        exit(-2);

    setPoseSub = node.subscribe<dobot::DobotPoseMsg>(
            "dobot/set_dobot_pose", 10, &DobotDriver::rosSetPoseCB, this);
    currentPosePub = node.advertise<dobot::DobotPoseMsg>(
            "dobot/current_dobot_pose", 10);

    // push mechanical arm outside the base, avoid be blocked by base of itself when find the boot home procedure.
    rosSet2Zero();
    sleep(1);

    cout << "INFO: go to boot home position" << endl;
    rosGoBootHome();
    sleep(18);

    // come to zero position and ready to work
    cout << "INFO: Setting zero position" << endl;
    rosSet2Zero();
    sleep(1);

    // This is only to update currentPose variable, method will read current pose
    // and let it into currentPose
    FullPose_t absPose;
    getCurrentPose(absPose);
    cout << "INFO: boot position(absolute): ";
    cout << "( " << absPose.x << ", " << absPose.y << ", " << absPose.z << ", " << absPose.r << ")" << endl;
    cout << "INFO: boot done ------ " << endl;
    // INFO("boot position(absolute): ( %02f, %02f, %02f, %02f )", absPose.x, absPose.y, absPose.z, absPose.r);
    // INFO("boot done ----- ");
}

int DobotDriver::uartInit() {
    try{
        openPort();
        setUartOpt(uartBaud, 8, 'N', 1);
    }
    catch(int i) {
        if (-1 == i) {
            perror("ERR: open uart port error");
        }
        if (-2 == i) {
            perror("ERR: set uart options error");
        }
        return i;
    }
    return 0;
}

void DobotDriver::openPort() {
    if(-1 == (uartFd = open(uartPort, O_RDWR | O_NOCTTY | O_NONBLOCK)))
        throw -1;
}

void DobotDriver::setUartOpt(const int nSpeed, const int nBits, const char nEvent, const int nStop) {
    struct termios newtio, oldtio;
    if(tcgetattr(uartFd, &oldtio) != 0) {
        throw -2;
    }
    newtio.c_cflag = 0;
    newtio.c_iflag = 0;
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;

    newtio.c_cflag |= CLOCAL | CREAD;

    switch(nBits) {
        case 7:
            newtio.c_cflag &= ~CSIZE;
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag &= ~CSIZE;
            newtio.c_cflag |= CS8;
            break;
        default:
            throw -2;
            return;
    }

    switch(nEvent) {
        case 'O':
            newtio.c_cflag &= ~PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            throw -2;
            return;
    }

    switch(nSpeed) {
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            throw -2;
            return;
    }

    if (1 == nStop)
        newtio.c_cflag &= ~CSTOPB;
    else if(2 == nStop)
        newtio.c_cflag |= CSTOPB;
    
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(uartFd, TCIOFLUSH);
    if ((tcsetattr(uartFd, TCSANOW, &newtio)) != 0) {
        tcsetattr(uartFd, TCSANOW, &oldtio);
        throw -2;
        return;
    }
    tcflush(uartFd, TCIOFLUSH);
}

// get zero value and put it into zero arguments
int DobotDriver::setInit(ros::NodeHandle node) {
    int ret = node.getParam("/runDobot/zeroX", zeroX) &&
        node.getParam("/runDobot/zeroY", zeroY) && 
        node.getParam("/runDobot/zeroZ", zeroZ) &&
        node.getParam("/runDobot/zeroR", zeroR) &&
        node.getParam("/runDobot/limitMinX", MinX) &&
        node.getParam("/runDobot/limitMaxX", MaxX) &&
        node.getParam("/runDobot/limitMinY", MinY) &&
        node.getParam("/runDobot/limitMaxY", MaxY) &&
        node.getParam("/runDobot/limitMinZ", MinZ) &&
        node.getParam("/runDobot/limitMaxZ", MaxZ) &&
        node.getParam("/runDobot/limitMinR", MinR) &&
        node.getParam("/runDobot/limitMaxR", MaxR) &&
        node.getParam("/runDobot/limitMinRadius", MinRadius) &&
        node.getParam("/runDobot/limitMaxRadius", MaxRadius) &&
        node.getParam("/runDobot/limitMinTheta", MinTheta) &&
        node.getParam("/runDobot/limitMaxTheta", MaxTheta);
    if (!ret) {
        cout << "ERR: wrong to get zero or limited values from rosparam server" << endl;
        return -1;
    }

    cout << "INFO: Setting const value done." << endl;
    cout << "INFO: Current zero values: " << "(" << zeroX << ", " << zeroY << ", " << zeroZ << ", " << zeroR << ")" << endl;
    cout << "INFO: Current limited values: " << "X axis: (" << MinX << ", " << MaxX << ")" << ", " << "Y axis: (" << MinY << ", " << MaxY << ")";
    cout << ", " << "Z axis: (" << MinZ << ", " << MaxZ << ")" << ", " << "R axis: (" << MinR << ", " << MaxR << ")";
    cout << ", " << "Radius: (" << MinRadius << ", " << MaxRadius << ")" << "Theta: (" << MinTheta << ", " << MaxTheta << ")" << endl;

    return 0;
}

int DobotDriver::updateZero() {
    ofstream zerofile;
    // ZEROFILE
    zerofile.open("~/driver/zero.file", ios::out);   // open file for write
    if (! zerofile.is_open()) {
        cout << "ERR: open file zero.file fail" << endl;
        return -1;
    }
    zerofile << currentPose.x << '\t';
    zerofile << currentPose.y << '\t';
    zerofile << currentPose.z << '\t';
    zerofile << currentPose.r << '\t';
    cout << "INFO: updated zero position into zero.file" << endl;
    zerofile.close();
    return 0;
}


int DobotDriver::checkChecksum( unsigned char *data, unsigned int datalen) {
    unsigned char addCS = 0x00;
    data = data + 3;
    for (unsigned int i=0; i<datalen-4; i++) {
        addCS += *data;
        data++;
    }
    addCS = 0xFF - addCS + 0x01;
    if (addCS != *data) {
        cout << "received data CS byte is: " << hex << (unsigned int)addCS << endl;
        return -1;      // check cs wrong
    }
    return 0;   //check cs right
}

CmdPointSet_t DobotDriver::createPointsetCmd(Pose_t pose) {
    CmdPointSet_t cmd;
    cmd.header1 = HD;
    cmd.header2 = HD;
    cmd.id = ID_POINTSET;
    cmd.len = LEN_POINTSET;
    cmd.ctrl = CTRLW | CTRLQ;
    // cmd.mode = pose.mode;
    cmd.mode = 0x01;
    char *pchar = (char *)&pose.x;
    for(int i=0; i<4; i++) {
        cmd.x[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.y;
    for(int i=0; i<4; i++) {
        cmd.y[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.z;
    for(int i=0; i<4; i++) {
        cmd.z[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.r;
    for(int i=0; i<4; i++) {
        cmd.r[i] = *pchar;
        pchar++;
    }

    char check = cmd.id + cmd.ctrl + cmd.mode;
    check += cmd.x[0] + cmd.x[1] + cmd.x[2] + cmd.x[3];
    check += cmd.y[0] + cmd.y[1] + cmd.y[2] + cmd.y[3];
    check += cmd.z[0] + cmd.z[1] + cmd.z[2] + cmd.z[3];
    check += cmd.r[0] + cmd.r[1] + cmd.r[2] + cmd.r[3];
    check = 0xFF - check + 0x01;
    cmd.checkSum = check;
#ifdef DEBUG_ORIGIN
    printPointsetCmd(cmd);
#endif
    return cmd;
}

void DobotDriver::sendPointsetCmd(CmdPointSet_t cmd) {
    CmdPointSetRet_t retData;
    CmdPointSetRet_t *pRetData = &retData;
    int retLen = 0;
    CmdPointSet_t * pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(1);
    retLen = read(uartFd, pRetData, 20);
    if (!(retLen > 0)) {
        throw -1;
        return;
    }
#ifdef DEBUG_ORIGIN
    printPointsetRetCmd(retData);
#endif
    tcflush(uartFd, TCIFLUSH);
    if ( -1 == checkChecksum((unsigned char*)pRetData, retLen) ) {
        throw -2;
        return;
    }

    return;
}

CmdGetCurrentPose_t DobotDriver::createGetCurrentPoseCmd() {
    CmdGetCurrentPose_t cmd;
    cmd.header1 = HD;
    cmd.header2 = HD;
    cmd.len = LEN_GETCURRENTPOSE;
    cmd.id = ID_GETCURRENTPOSE;
    cmd.ctrl = CTRLR;
    char check = cmd.id + cmd.ctrl;
    check = 0xFF - check + 0x01;
    cmd.checkSum = check;
#ifdef DEBUG_ORIGIN
    printGetCurrentPoseCmd(cmd);
#endif
    return cmd;
}

void DobotDriver::sendGetCurrentPoseCmd(CmdGetCurrentPose_t cmd, FullPose_t &retPose) {
    CmdGetCurrentPoseRet_t retData;
    CmdGetCurrentPoseRet_t *pRetData = &retData;
    int retLen = 0;
    CmdGetCurrentPose_t *pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(1);
    retLen = read(uartFd, pRetData, 50);
    if (!(retLen > 0)) {
        throw -1;
        return;
    }

#ifdef DEBUG_ORIGIN
    printGetCurrentPoseRetCmd(retData);
#endif
    tcflush(uartFd, TCIFLUSH);
    if ( -1 == checkChecksum((unsigned char*)pRetData, retLen) ) {
        throw -2;
        return;
    }

    retPose.x = *(float *)retData.x;
    retPose.y = *(float *)retData.y;
    retPose.z = *(float *)retData.z;
    retPose.r = *(float *)retData.r;
    retPose.j1 = *(float *)retData.j1;
    retPose.j2 = *(float *)retData.j2;
    retPose.j3 = *(float *)retData.j3;
    retPose.j4 = *(float *)retData.j4;
    retPose.j5 = *(float *)retData.j5;

}

CmdSetHome_t DobotDriver::createSetHomeCmd() {
    CmdSetHome_t cmd;
    cmd.header1 = HD;
    cmd.header2 = HD;
    cmd.len = LEN_SETHOME;
    cmd.id = ID_SETHOME;
    cmd.ctrl = CTRLW | CTRLQ;
    cmd.reserved = 0x00;
    char check = cmd.id + cmd.ctrl + cmd.reserved;
    check = 0xFF - check + 0x01;
    cmd.checkSum = check;
#ifdef DEBUG_ORIGIN
    printSetHomeCmd(cmd);
#endif
    return cmd;
}

void DobotDriver::sendSetHomeCmd(CmdSetHome_t cmd) {
    CmdSetHomeRet_t retData;
    CmdSetHomeRet_t *pRetData = &retData;
    int retLen = 0;
    CmdSetHome_t * pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(1);
    retLen = read(uartFd, pRetData, 20);
    if (!(retLen > 0)) {
        throw -1;
        return;
    }
#ifdef DEBUG_ORIGIN
    // printSetHomeRetCmd(retData);
    // Not completed
#endif
    tcflush(uartFd, TCIFLUSH);
    if ( -1 == checkChecksum((unsigned char*)pRetData, retLen) ) {
        throw -2;
        return;
    }

    return;
}

int DobotDriver::getCurrentPose(FullPose_t &retPose) {
    CmdGetCurrentPose_t cmd;
    Pose_t mPose;
    cmd = createGetCurrentPoseCmd();
    try {
        sendGetCurrentPoseCmd(cmd, retPose);
        // retPose is the world coordinate, return from dobot arm
        // transfer FullPose_t struct variable to Pose_t struct variable
        mPose.x = retPose.x;
        mPose.y = retPose.y;
        mPose.z = retPose.z;
        mPose.r = retPose.r;
        // Add return pose position with software zero calibration
        retPose.x -= zeroX;
        retPose.y -= zeroY;
        retPose.z -= zeroZ;
        retPose.r -= zeroR;
        // now retPose is the absolute coordinate
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("receive data cs check wrong");
            return -1;
        }
    }
    // mPose is the world coordinate
    updateCurrentPose(mPose);
    return 0;
}

void DobotDriver::updateCurrentPose(Pose_t pose) {
    currentPose = pose;
#ifdef DEBUG
    cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
#endif
}

int DobotDriver::runPointset(Pose_t pose) {
    CmdPointSet_t cmd;

    // absPose is the world coordinate in dobot, it is the dobot_base frame
    Pose_t absPose;
    absPose.x = pose.x;
    absPose.y = pose.y;
    absPose.z = pose.z;
    absPose.r = pose.r;

    // decide limited space of margin
    float R = sqrt(pose.x * pose.x + pose.y * pose.y);
    float theta = atan2(pose.y, pose.x);    // theta return -pi to pi
    theta = 180 * theta / PI;   // now theta return -180 to 180
    if ( R < MinRadius || R > MaxRadius || theta < MinTheta || theta > MaxTheta 
            || pose.z < MinZ || pose.z > MaxZ || pose.r < MinR || pose.r > MaxR ) {
        perror("out of marge limitation");
        cout << "MinRadius= " << MinRadius << " MaxRadius= " << MaxRadius;
        cout << "MinTheta= " << MinTheta << " MaxTheta= " << MaxTheta;
        cout << "MinZ= " << MinZ << " MaxZ= " << MaxZ;
        cout << "MinR= " << MinR << " MaxR= " << MaxR << endl;
        cout << "R= " << R << " theta= " << theta << " pose.z= " << pose.z << " pose.r= " << pose.r << endl;
        return -2;
    }
    
    // absPose is the dobot_base coordinate
    cmd = createPointsetCmd(absPose);
    try{
        sendPointsetCmd(cmd);
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("recevie data cs check wrong");
            return -1;
        }
    }
    // updateCurrentPose(absPose);
// #ifdef DEBUG
    // cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    // cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
// #endif
    return 0;
}

/****************************
 * 
 *  PUBLIC METHODS
 *  
 *  */

// This is a command of runDobot, to let arm return ro zero position
int DobotDriver::rosSet2Zero() {
    CmdPointSet_t cmd;

    Pose_t absPose;
    cout << zeroX << " " << zeroY << " " << zeroZ << " " << zeroR << endl;
    absPose.x = zeroX;
    absPose.y = zeroY;
    absPose.z = zeroZ;
    absPose.r = zeroR;
    cmd = createPointsetCmd(absPose);
    try{
        sendPointsetCmd(cmd);
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("recevie data cs check wrong");
            return -1;
        }
    }
    // updateCurrentPose(absPose);
// #ifdef DEBUG
    // cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    // cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
// #endif
    return 0;
}


void DobotDriver::rosPublishPose() {
    FullPose_t currentPose;
    dobot::DobotPoseMsg pubPoseMsg;
    int ret = getCurrentPose(currentPose);
    if ( -1 == ret ) {
        perror("fault to get current dobot position");
    }
    pubPoseMsg.control = 0;
    pubPoseMsg.mode = 0;
    pubPoseMsg.x = currentPose.x;
    pubPoseMsg.y = currentPose.y;
    pubPoseMsg.z = currentPose.z;
    pubPoseMsg.r = currentPose.r;
    currentPosePub.publish(pubPoseMsg);
}

/**************************
 * 
 *  CALLBACK PUBLISH METHODS
 *  
 *  */
void DobotDriver::rosSetPoseCB(const dobot::DobotPoseMsg receivePose) {
    Pose_t poseCommand;
    if ( 1 == receivePose.control ) {
        // control == 1 means that return to zero position in any case
        rosSet2Zero();
    }
    else if ( 0 == receivePose.control ) {
        if ( 0 == receivePose.mode ) {
            // mode == 0 means that coordinate is relative Cartesian coordinate
            cout << "DEBUG: get a relative Cartesian coordinate: (";
            cout << receivePose.x << ", " << receivePose.y << ", " << receivePose.z << ", " << receivePose.r << ")" << endl;
        }
        else if ( 1 == receivePose.mode ) {
            // mode == 1 means that coordinate is dobot_base Cartesian coordinate
            cout << "DEBUG: get a dobot_base Cartesian coordinate: (";
            cout << receivePose.x << ", " << receivePose.y << ", " << receivePose.z << ", " << receivePose.r << ")" << endl;
            poseCommand.x = receivePose.x;
            poseCommand.y = receivePose.y;
            poseCommand.z = receivePose.z;
            poseCommand.r = receivePose.r;
        }
        else if ( 2 == receivePose.mode ) {
            // mode == 2 means that coordinate is Polar coordinate
            // theta is angle value and we should convert it to radian value
            cout << "DEBUG: get a Polar coordinate: (";
            cout << receivePose.radius << ", " << receivePose.theta << ", " << receivePose.z << ", " << receivePose.r << ")" << endl;
            poseCommand.x = receivePose.radius * cos(receivePose.theta / 180 * PI);
            poseCommand.y = receivePose.radius * sin(receivePose.theta / 180 * PI);
            poseCommand.z = receivePose.z;
            poseCommand.r = receivePose.r;
        }
        // poseCommand is the dobot_base frame coordinate
        int ret = runPointset(poseCommand);
        if ( -1 == ret ) {
            perror("fault to run Pointset method to dobot");
        }
    }
    cout << "INFO: move dobot arm done ----- " << endl;
    // INFO("move dobot arm done ----- ");
}

void DobotDriver::rosGoBootHome() {
    CmdSetHome_t cmd;
    cmd = createSetHomeCmd();
    try{
        sendSetHomeCmd(cmd);
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            perror("fault to run rosGoBootHome method to dobot");
            return;
        }
        if (-2 == i) {
            perror("recevie data cs check wrong");
            perror("fault to run rosGoBootHome method to dobot");
            return;
        }
    }
    return;
    cout << "INFO: go boothome done ----- " << endl;
}

/**************************
 *
 *  PRIVATE METHODS of print
 *
 * */
void DobotDriver::printPointsetCmd(CmdPointSet_t cmd) {
    cout << "DEBUG: pointset command is:" << endl;
    cout << "HD HD LN ID CR MD X1 X2 X3 X4 Y1 Y2 Y3 Y4 Z1 Z2 Z3 Z4 R1 R2 R3 R4 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.mode << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[i] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
 //   cout.unsetf(ios::hex);
}
 
void DobotDriver::printPointsetRetCmd(CmdPointSetRet_t cmd) {
    cout << "DEBUG: pointset receive command is:" << endl;
    cout << "HD HD LN ID CR Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    for (int i=0; i<8; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.queuedCmdIndex[i] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
}

void DobotDriver::printGetCurrentPoseCmd(CmdGetCurrentPose_t cmd) {
    cout << "DEBUG: getCurrentPose command is:" << endl;
    cout << "HD HD LN ID CR CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
}

void DobotDriver::printGetCurrentPoseRetCmd(CmdGetCurrentPoseRet_t cmd) {
    cout << "DEBUG: getCurrentPose receive command is:" << endl;
    cout << "HD HD LN ID CR X1 X2 X3 X4 Y1 Y2 Y3 Y4 Z1 Z2 Z3 Z4 R1 R2 R3 R4";
    cout << " M1 M2 M3 M4 N1 N2 N3 N4 S1 S2 S3 S4 L1 L2 L3 L4 U1 U2 U3 U4 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << " " << endl;
}

void DobotDriver::printSetHomeCmd(CmdSetHome_t cmd) {
    cout << "DEBUG: setHome command is:" << endl;
    cout << "HD HD LN ID CR H1 H2 H3 H4 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.reserved << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << " " << endl;

}
