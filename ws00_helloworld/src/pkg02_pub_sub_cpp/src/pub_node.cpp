#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class SingleDogNode:public rclcpp::Node
{
public:
    SingleDogNode(std::string name):Node(name),count(0)
    {
        RCLCPP_INFO(this->get_logger(),"大家好，我是%s",name.c_str());
        pub_money = this->create_publisher<std_msgs::msg::String>("sexy_girl",10);
        timer_  = this->create_wall_timer(1s,std::bind(&SingleDogNode::on_timer,this));
    }
private:

    void on_timer(){
        auto message = std_msgs::msg::String();
        message.data = "helloworld"+std::to_string(count++);
        RCLCPP_INFO(this->get_logger(),"发布方发布消息:%s",message.data.c_str());
        pub_money->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_money;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SingleDogNode>("chatt"));
    rclcpp::shutdown();
    return 0;
}


