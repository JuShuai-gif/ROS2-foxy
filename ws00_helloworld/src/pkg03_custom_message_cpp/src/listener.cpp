#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "pkg03_custom_message_cpp/msg/student.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class SingleDogNode:public rclcpp::Node
{
public:
  SingleDogNode(std::string name):Node(name)
  {
    RCLCPP_INFO(this->get_logger(),"大家好，我是%s",name.c_str());
    sub_novel = this->create_subscription<pkg03_custom_message_cpp::msg::Student>("sexy_girl",10,std::bind(&SingleDogNode::topic_callback,this,_1));

  }
private:
  rclcpp::Subscription<pkg03_custom_message_cpp::msg::Student>::SharedPtr sub_novel;

  void topic_callback(const pkg03_custom_message_cpp::msg::Student::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),"姓名:%s，身高：%f,年龄：%d",msg->name.c_str(),msg->height,msg->age);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<SingleDogNode>("Wang2");
  rclcpp::spin(node);
  rclcpp::shutdown();

  printf("hello world pkg02_pub_sub_cpp package\n");
  return 0;
}
