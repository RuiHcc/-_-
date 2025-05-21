#ifndef FORMATION_TYPE_RVIZ_PLUGIN_H
#define FORMATION_TYPE_RVIZ_PLUGIN_H


#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <stdio.h>

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QFileDialog>

#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

namespace formation_type_plugin
{

class FormationtypePub : public rviz::Panel
{
  Q_OBJECT
public:
  FormationtypePub(QWidget* parent = 0)
    : rviz::Panel(parent)
  {
    // 创建一个按钮
    QPushButton* button1 = new QPushButton("formation type: column", this);
    QPushButton* button2 = new QPushButton("formation type: oblique triangle", this);
    QPushButton* button3 = new QPushButton("formation type: Anterior triangle", this);

    // 连接按钮点击事件到槽函数
    connect(button1, SIGNAL(clicked()), this, SLOT(onButton1Clicked()));
    connect(button2, SIGNAL(clicked()), this, SLOT(onButton2Clicked()));
    connect(button3, SIGNAL(clicked()), this, SLOT(onButton3Clicked()));

    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(button1);
    layout->addWidget(button2);
    layout->addWidget(button3);

    // 设置面板的布局
    setLayout(layout);

    pub = nh.advertise<geometry_msgs::Point>("/formationType", 5);
  }

public Q_SLOTS:
  // 槽函数，在按钮点击时被调用
  void onButton1Clicked();
  void onButton2Clicked();
  void onButton3Clicked();

private:
  void publishMessage(const geometry_msgs::Point& message);

protected:
  ros::NodeHandle nh;
  ros::Publisher pub;
  // {
  //   // 创建一个ROS节点句柄
  //   ros::NodeHandle nh;

  //   // 创建一个发布者，发布 std_msgs::String 消息到名为 "/custom_topic" 的话题
  //   ros::Publisher pub = nh.advertise<std_msgs::String>("/custom_topic", 1);

  //   // 创建一个消息
  //   std_msgs::String msg;
  //   msg.data = "Hello, RViz!";

  //   // 发布消息
  //   pub.publish(msg);
  // }
};

}

#endif  // GOALPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
