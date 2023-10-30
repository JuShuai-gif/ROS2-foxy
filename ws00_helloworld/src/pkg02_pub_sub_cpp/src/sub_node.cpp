#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class SingleDogNode:public rclcpp::Node
{
public:
  SingleDogNode(std::string name):Node(name)
  {
    RCLCPP_INFO(this->get_logger(),"大家好，我是%s",name.c_str());
    sub_novel = this->create_subscription<std_msgs::msg::String>("sexy_girl",10,std::bind(&SingleDogNode::topic_callback,this,_1));

  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_novel;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),"我已经阅读%s",msg->data.c_str());
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
