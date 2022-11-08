/*
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

#include <gazebo_logical_camera_plugin_ros/logical_camera_plugin.h>

using namespace gazebo;
using namespace ignition;
using namespace std;
using namespace ros;

GZ_REGISTER_SENSOR_PLUGIN(LogicalCameraPlugin);

void LogicalCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
    // Get the parent sensor.
    this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor){
        gzerr << "LogicalCameraPlugin requires a LogicalCameraSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    ROS_INFO("LogicalCameraPlugin correctly loaded!!!");
    ROS_INFO("_near:=%g",this->parentSensor->Near());
    ROS_INFO("_far:=%g",this->parentSensor->Far());
    ROS_INFO("_horizontalFOV:=%g",this->parentSensor->HorizontalFOV());
    ROS_INFO("_aspect_ratio:=%g",this->parentSensor->AspectRatio());

    if (!_sdf->HasElement("frameName"))
      this->frameID = "rgb_logical_frame";
    else
      this->frameID = _sdf->GetElement("frameName")->Get<std::string>();

    std::string camera_name;

    if (!_sdf->HasElement("cameraName"))
      camera_name = "gripper_astra";
    else
      camera_name = _sdf->GetElement("cameraName")->Get<std::string>();

    std::string topic_name;

    if (!_sdf->HasElement("imageTopicName"))
      topic_name = "rgb/logical_image";
    else
      topic_name = _sdf->GetElement("imageTopicName")->Get<std::string>();

    nh = new ros::NodeHandle();

    image_pub = nh->advertise<object_pose_msgs::ObjectList>(camera_name + "/" + topic_name, 1, true);
}

void LogicalCameraPlugin::OnUpdate(){

    msgs::LogicalCameraImage logical_image;

    object_pose_msgs::ObjectList msg;

    logical_image = this->parentSensor->Image();
    gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
    if (!scene || !scene->Initialized())
      return;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->frameID;

    int number_of_models = logical_image.model_size();
    for(int i=0; i < number_of_models; i++){

        object_pose_msgs::ObjectPose object_msg;

        rendering::VisualPtr visual = scene->GetVisual(logical_image.model(i).name());

        if (!visual)
          continue;

        std::cmatch m;

        //Extract object class id and instance id, assumes class_id format (i.e. box_1)
        if(std::regex_search(logical_image.model(i).name().c_str(), m, std::regex("(.+)_([0-9]+)")))
        {
          object_msg.class_id = m[1];
          object_msg.instance_id = std::stoi(m[2]);
        }else{
          continue;
        }

        auto bounding_box = visual->BoundingBox();

        object_msg.pose.position.x = logical_image.model(i).pose().position().x();
        object_msg.pose.position.y = logical_image.model(i).pose().position().y();
        object_msg.pose.position.z = logical_image.model(i).pose().position().z();

        object_msg.pose.orientation.x = logical_image.model(i).pose().orientation().x();
        object_msg.pose.orientation.y = logical_image.model(i).pose().orientation().y();
        object_msg.pose.orientation.z = logical_image.model(i).pose().orientation().z();
        object_msg.pose.orientation.w = logical_image.model(i).pose().orientation().w();

        object_msg.size.x = bounding_box.XLength();
        object_msg.size.y = bounding_box.YLength();
        object_msg.size.z = bounding_box.ZLength();

        object_msg.min.x = bounding_box.Min().X();
        object_msg.min.y = bounding_box.Min().Y();
        object_msg.min.z = bounding_box.Min().Z();

        object_msg.max.x = bounding_box.Max().X();
        object_msg.max.y = bounding_box.Max().Y();
        object_msg.max.z = bounding_box.Max().Z();

        msg.objects.push_back(object_msg);
    }

    this->image_pub.publish(msg);
}
