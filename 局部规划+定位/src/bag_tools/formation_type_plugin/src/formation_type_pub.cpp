#include "formation_type_pub.h"

namespace formation_type_plugin
{

  void FormationtypePub::onButton1Clicked()
  {
    // 创建一个消息
    geometry_msgs::Point msg;
    msg.x = 0.0;
    publishMessage(msg);
  }

  void FormationtypePub::onButton2Clicked()
  {
    // 创建一个消息
    geometry_msgs::Point msg;
    msg.x = 2.0;
    publishMessage(msg);
  }

  void FormationtypePub::onButton3Clicked()
  {
    // 创建一个消息
    geometry_msgs::Point msg;
    msg.x = 1.0;
    publishMessage(msg);
  }

  void FormationtypePub::publishMessage(const geometry_msgs::Point& message)
  {
    // 创建一个消息
    geometry_msgs::Point msg;
    msg = message;

    // 发布消息
    pub.publish(msg);
  }

}  // end namespace rviz

// // 声明该类是一个QT插件
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(formation_type_plugin::FormationtypePub, rviz::Panel)

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( formation_type_plugin::FormationtypePub,rviz::Panel )
