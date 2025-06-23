//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.


#ifndef REDDOGIMUDATA_H
#define REDDOGIMUDATA_H

#include "packetcallback.h"
#include <sensor_msgs/msg/imu.hpp>

struct ReddogIMUDataManager : public PacketCallback
{
    std::string frame_id = DEFAULT_FRAME_ID;

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        bool quaternion_available = packet.containsOrientation();
        bool gyro_available = packet.containsCalibratedGyroscopeData();
        bool accel_available = packet.containsCalibratedAcceleration();

        geometry_msgs::msg::Quaternion quaternion;
        if (quaternion_available)
        {
            XsQuaternion q = packet.orientationQuaternion();

            quaternion.w = q.w();
            quaternion.x = q.x();
            quaternion.y = q.y();
            quaternion.z = q.z();
        }

        geometry_msgs::msg::Vector3 gyro;
        if (gyro_available)
        {
            XsVector g = packet.calibratedGyroscopeData();
            gyro.x = g[0];
            gyro.y = g[1];
            gyro.z = g[2];
        }

        geometry_msgs::msg::Vector3 accel;
        if (accel_available)
        {
            XsVector a = packet.calibratedAcceleration();
            accel.x = a[0];
            accel.y = a[1];
            accel.z = a[2];
        }

        if( quaternion_available || accel_available || gyro_available)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("ReddogIMUDataManager"),
                "IMU Data: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                quaternion.x, quaternion.y, quaternion.z, quaternion.w,
                gyro.x, gyro.y, gyro.z,
                accel.x, accel.y, accel.z
            );
        }
    }
};

#endif
