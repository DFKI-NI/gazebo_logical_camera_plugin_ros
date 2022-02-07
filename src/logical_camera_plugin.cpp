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
    image_pub = nh->advertise<gazebo_logical_camera_plugin_ros::LogicalImage>("/mobipick/" + camera_name + "/" + topic_name, 1, true);
}

void LogicalCameraPlugin::OnUpdate(){
  
    msgs::LogicalCameraImage logical_image;
    gazebo_logical_camera_plugin_ros::LogicalImage msg;

    logical_image = this->parentSensor->Image();
    gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
    if (!scene || !scene->Initialized())
      return;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = this->frameID;

    msg.pose.position.x = logical_image.pose().position().x();
    msg.pose.position.y = logical_image.pose().position().y();
    msg.pose.position.z = logical_image.pose().position().z();

    msg.pose.orientation.x = logical_image.pose().orientation().x();
    msg.pose.orientation.y = logical_image.pose().orientation().y();
    msg.pose.orientation.z = logical_image.pose().orientation().z();
    msg.pose.orientation.w = logical_image.pose().orientation().w();

    int number_of_models = logical_image.model_size();
    for(int i=0; i < number_of_models; i++){
        gazebo_logical_camera_plugin_ros::Model model_msg;

        if (logical_image.model(i).name() == "smart_factory" || logical_image.model(i).name() == "ground_plane"
            || logical_image.model(i).name() == "")
          continue;

        rendering::VisualPtr visual = scene->GetVisual(logical_image.model(i).name());

        if (!visual)
          continue;

        auto bounding_box = visual->BoundingBox();

        model_msg.pose.position.x = logical_image.model(i).pose().position().x();
        model_msg.pose.position.y = logical_image.model(i).pose().position().y();
        model_msg.pose.position.z = logical_image.model(i).pose().position().z();

        model_msg.pose.orientation.x = logical_image.model(i).pose().orientation().x();
        model_msg.pose.orientation.y = logical_image.model(i).pose().orientation().y();
        model_msg.pose.orientation.z = logical_image.model(i).pose().orientation().z();
        model_msg.pose.orientation.w = logical_image.model(i).pose().orientation().w();

        model_msg.size.x = bounding_box.XLength();
        model_msg.size.y = bounding_box.YLength();
        model_msg.size.z = bounding_box.ZLength();

        model_msg.min.x = bounding_box.Center().X() - bounding_box.Size().X()/2.0;
        model_msg.min.y = bounding_box.Center().Y() - bounding_box.Size().Y()/2.0;
        model_msg.min.z = bounding_box.Center().Z() - bounding_box.Size().Z()/2.0;
        
        model_msg.max.x = bounding_box.Center().X() + bounding_box.Size().X()/2.0;
        model_msg.max.y = bounding_box.Center().Y() + bounding_box.Size().Y()/2.0;
        model_msg.max.z = bounding_box.Center().Z() + bounding_box.Size().Z()/2.0;
	
        model_msg.min.x = bounding_box.Min().X();
        model_msg.min.y = bounding_box.Min().Y();
        model_msg.min.z = bounding_box.Min().Z();
        
        model_msg.max.x = bounding_box.Max().X();
        model_msg.max.y = bounding_box.Max().Y();
        model_msg.max.z = bounding_box.Max().Z();
        
        model_msg.type = logical_image.model(i).name();

        msg.models.push_back(model_msg);
    }

    this->image_pub.publish(msg);
}
