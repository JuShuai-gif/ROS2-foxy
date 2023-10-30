#include "rclcpp/rclcpp.hpp"
#include "pkg03_custom_message_cpp/msg/student.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class SingleDogNode:public rclcpp::Node
{
public:
    SingleDogNode(std::string name):Node(name),count(0)
    {
        RCLCPP_INFO(this->get_logger(),"大家好，我是%s",name.c_str());
        pub_money = this->create_publisher<pkg03_custom_message_cpp::msg::Student>("sexy_girl",10);
        timer_  = this->create_wall_timer(1s,std::bind(&SingleDogNode::on_timer,this));
    }
private:

    void on_timer(){
        auto message = pkg03_custom_message_cpp::msg::Student();
        message.age = count++;
        message.name = "GHR";
        message.height = 180;
        RCLCPP_INFO(this->get_logger(),"姓名:%s，身高：%f,年龄：%d",message.name.c_str(),message.height,message.age);
        pub_money->publish(message);
    }
    rclcpp::Publisher<pkg03_custom_message_cpp::msg::Student>::SharedPtr pub_money;
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


