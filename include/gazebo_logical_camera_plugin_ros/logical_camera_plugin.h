/*
 * Copyright (c) 2018, Federico Nardi
 * Copyright (c) 2022, Amos Smith (DFKI GmbH) and contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <string>
#include <sstream>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <object_pose_msgs/ObjectList.h>
#include <regex>

namespace gazebo {
class LogicalCameraPlugin : public SensorPlugin{
public:
    LogicalCameraPlugin() : SensorPlugin(){}
    virtual ~LogicalCameraPlugin() {}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
protected:
    ros::NodeHandle *nh;
    ros::Publisher image_pub;
private:
    virtual void OnUpdate();
    std::string frameID;
    sensors::LogicalCameraSensorPtr parentSensor;
    event::ConnectionPtr updateConnection;
};
}
