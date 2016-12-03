/* Copyright (C) 
 * 2016 - S-PWE, dicksonliuming@gmail.com
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 */

/**
 * \file    runDobot.cpp
 * \brief   This software is used to control basic dobot moving. Including a unlimited \
 *          loop in the main function.
 * \author  S-PWE, dicksonliuming@gmail.com
 * \version 0.1.0
 * \date    2016-11-24
 */
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "dobot/DobotPoseMsg.h"
#include "dobotDriver.hpp"
#include "ros/topic.h"


using namespace std;

/**
 * \brief   rosSetPoseCB : callback function for 'setPose' command subscriber
 *
 * \param   receivePose : origin request of 'setPose' command in a DobotPoseMsg type message
 */
void rosSetPoseCB(const dobot::DobotPoseMsg receivePose);

int main(int argc, char * argv[])
{
    // else if(!strcmp(argv[1], "Set2Zero")) {
        // DobotDriver dobot;
        // int ret = dobot.set2Zero();
        // if ( -1 == ret ) {
            // perror("fault to set to zero position");
        // }
    // }
    // 
    // Boot ros
    ros::init(argc, argv, "runDobot");
    ros::NodeHandle node;
    DobotDriver initDobot(node);
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        initDobot.rosPublishPose();
        ros::spinOnce();
        // loop_rate.sleep();
        ros::topic::waitForMessage<dobot::DobotPoseMsg>("/dobot/relative_pose", ros::Duration(0.1));
    }

    return 0;
}

