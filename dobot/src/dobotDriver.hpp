#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_ value

#include "ros/ros.h"

#define HD      0xAA
#define CTRLR           0x00
#define CTRLW           0x01
#define CTRLQ           0x02
#define LEN_POINTSET    0x13
#define LEN_POINTSETRET 0x0A
#define LEN_GETCURRENTPOSE  0x02
#define LEN_SETHOME     0x03
#define LEN_SETHOMERET  0x0A
#define ID_POINTSET     0x54
#define ID_GETCURRENTPOSE   0x0A
#define ID_SETHOME      0x1F

typedef struct {
    unsigned int mode;
    float x;
    float y;
    float z;
    float r;
}Pose_t;

typedef struct {
    float x;
    float y;
    float z;
    float r;
    float j1;
    float j2;
    float j3;
    float j4;
    float j5;
}FullPose_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;       // This command is always be write to dobot and isQueued
    unsigned char mode;
    unsigned char x[4];
    unsigned char y[4];
    unsigned char z[4];
    unsigned char r[4];
    unsigned char checkSum;
}CmdPointSet_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char queuedCmdIndex[8];
    unsigned char checkSum;
}CmdPointSetRet_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char checkSum;
}CmdGetCurrentPose_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char x[4];
    unsigned char y[4];
    unsigned char z[4];
    unsigned char r[4];
    unsigned char j1[4];
    unsigned char j2[4];
    unsigned char j3[4];
    unsigned char j4[4];
    unsigned char j5[4];
    unsigned char checkSum;
}CmdGetCurrentPoseRet_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char reserved;
    unsigned char checkSum;
}CmdSetHome_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char queuedCmdIndex[8];
    unsigned char checkSum;
}CmdSetHomeRet_t;

class DobotDriver {
private:
    // It contain the current pose of dobot arm, will be updated by every motion of arm.
    // currentPose is the position of dobot arm with software calibration: (zeroX, zeroY, zeroZ, zeroR)
    Pose_t currentPose;

    // 2 arguments of uart communication to dobot arm.
    int uartFd;
    const char *uartPort;
    const unsigned int uartBaud;

    // Zero position of dobot with (x, y, z, r)
    static float zeroX;
    static float zeroY;
    static float zeroZ;
    static float zeroR;

    // limited of space margin, in case of safe
    static float MaxX;
    static float MinX;
    static float MaxY;
    static float MinY;
    static float MaxZ;
    static float MinZ;
    static float MaxR;
    static float MinR;

    static float MaxRadius;
    static float MinRadius;
    static float MaxTheta;
    static float MinTheta;

    // Methods with uart communication to dobot arm.
    
    // About ROS arguments
    ros::Publisher currentPosePub;
    ros::Subscriber setPoseSub;

    /**
     *  @function:  uartInit
     *  @brief:  initialization of uart to communicate with dobot arm.
     *  @arg:  none
     *  @return:  int: negative return is fault, 0 is done.
     *  */
    int uartInit();

    /**
     *  @func:  openPort
     *  @args:  none
     *  @retn:  none
     *  */
    void openPort();

    /**
     *  @func:  setUartOpt
     *  @args:  some arguments with uart communication: speed, bits, event, stop, etc.
     *  @retn:  none
     *  */
    void setUartOpt(const int nSpeed, const int nBits, const char nEvent, const int nStop);

    /**
     *  @func:  setInit
     *  @brif:  set software zero value from zero.file
     *  @args:  node is the ros hander
     *  @retn:  return 0 is right but negative value is wrong
     *  */
    int setInit(ros::NodeHandle node);

    /**
     *  @func:  updateZero
     *  @brif:  update current position of arm into zero position and write data into zero.file
     *  @retn:  return 0 is right but negative value is wrong
     *  */
    int updateZero();

    /****
     *  Methods about command last byte: Checksum.
     *  This byte in dobot arm command is working with Complement check, 
     *  which is the complement of Content part of command.
     *       Checksum = 0xFF - (cmd.id + cmd.ctrl + cmd.params) + 0x01
    ****/
    /**
     *  @func:  checkChecksum
     *  @brif:  check cs value in the received data to determin the validity of data.
     *  @args:  received command data and this's length
     *  @retn:  check result: 0 is for right, negative value means wrong.
     *  */
    int checkChecksum(unsigned char *data, unsigned int datalen);

    /****
     *  Methods about create a command.
     *  Different command have different length and bits, be attention.
     *  
     * **/
    /**
     *  @func:  createPointsetCmd
     *  @brif:  create a command of move arm with PointSet method.
     *  @args:  pose is the PointSet required, this is the ABSOLUTE COORDINATE.
     *  @retn:  CmdPointSet_t is the command it created.
     *  */
    CmdPointSet_t createPointsetCmd(Pose_t pose);

    /**
     *  @func:  sendPointsetCmd
     *  @brif:  send Pointset command to the dobot arm.
     *  @args:  cmd is the command waiting for send
     *  @retn:  none
     *  */
    void sendPointsetCmd(CmdPointSet_t cmd);

    /**
     *  @function: createGetCurrentPoseCmd
     *  @brief:     use to create a command for get dobot current position
     *  @args:      none
     *  @return:    CmdGetCurrentPose_t: is the return of current position
     *  */
    CmdGetCurrentPose_t createGetCurrentPoseCmd();

    /**
     *  @function:  sendGetCurrentPoseCmd
     *  @brief:     send the command of get dobot current position
     *  @args:      cmd is the command for getting current position
     *              retPose is the return position
     *  @return:    none
     *  */
    void sendGetCurrentPoseCmd(CmdGetCurrentPose_t cmd, FullPose_t &retPose);

    CmdSetHome_t createSetHomeCmd();

    void sendSetHomeCmd(CmdSetHome_t cmd);

    /**
     *  @func:  getCurrentPose
     *  @brif:  get current dobot arm FullPose
     *  @args:  FullPose pose, pose is the return value of current pose of dobot arm,
     *          pose is the ABSORATE COORDINATE
     *  @retn:  return negative value is wrong, 0 is right
     *  */
    int getCurrentPose(FullPose_t &pose);

    /**
     *  @func:  updateCurrentPose
     *  @brif:  update currentPose variable
     *  @args:  pose is the world coordinate
     *  @retn:  none
     *  */
    void updateCurrentPose(Pose_t pose);

    /**
     *  @func:  runPointset
     *  @brif:  publish Pointset command to dobot arm to move it
     *  @args:  pose is a struct value with Pointset command, pose is the RELATIVE COORDINATE
     *  @retn:  return a value to get correct running information of command
     *  */
    int runPointset(Pose_t pose);

    /**
     *  @func:  printPointsetCmd
     *  @brif:  print the command of Pointset method
     *  @args:  cmd is the command of Pointset
     *  @retn:  none
     *  */
    void printPointsetCmd(CmdPointSet_t cmd);

    /**
     *  @func:  printPointsetRetCmd
     *  @brif:  print the receive command of Pointset method
     *  @args:  cmd is the receive command 
     *  @retn:  none
     *  */
    void printPointsetRetCmd(CmdPointSetRet_t cmd);

    void printGetCurrentPoseCmd(CmdGetCurrentPose_t cmd);
    void printGetCurrentPoseRetCmd(CmdGetCurrentPoseRet_t cmd);
    void printSetHomeCmd(CmdSetHome_t cmd);

public:
    /**
     *  @func:  DobotDriver
     *  @brif:  construct methods
     *  @args:  none
     *  */
    DobotDriver();
    DobotDriver(ros::NodeHandle);

    /**
     *  @func:  rosSet2Zero
     *  @brif:  control dobot to zero position
     *  @retn:  return 0 is right but negative value is wrong
     *  */
    int rosSet2Zero();

    /**
     *  @function:  rosSetPoseCB
     *  @brief:     This is a callback function for ros subscriber(relative_pose)
     *              to receive position command
     *  @arg:      receivePose: is a DobotPoseMsg type argument, contain position command
     *  */
    void rosSetPoseCB(const dobot::DobotPoseMsg receivePose);

    /**
     *  @function:  rosPublishPose
     *  @brief:     This is a method for publish current dobot position, every time
     *              call it, a position will be published in ros message
     *  */
    void rosPublishPose();

    void rosGoBootHome();
};

#endif /* ifndef DOBOT_DRIVER_H_ */
