#include "rclcpp/rclcpp.hpp"
#include "pkg03_custom_message_cpp/srv/add_ints.hpp"

using pkg03_custom_message_cpp::srv::AddInts;
using namespace std::chrono_literals;

class MinimalClient:public rclcpp::Node
{
public:
    MinimalClient():Node("minimal_client"){
        client = this->create_client<AddInts>("add_ints");
        RCLCPP_INFO(this->get_logger(),"客户端创建，等待连接服务端！");

    }

    bool connect_server()
    {
        while(!client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制退出！");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候。。。。");
        }
        return true;
    }

    rclcpp::Client<AddInts>::SharedFuture send_request(int num1,int num2){
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client->async_send_request(request);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr client;
};

int main(int argc,char **argv)
{
    if(argc!=3){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交两个整型数据！");
        return 1;
    }

    rclcpp::init(argc,argv);

    auto client = std::make_shared<MinimalClient>();
    bool flag = client->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接失败！");
        return 0;
    }
    auto response = client->send_request(atoi(argv[1]),atoi(argv[2]));

    if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(),"请求正常处理！");
        RCLCPP_INFO(client->get_logger(),"响应结果：%d!",response.get()->sum);

    }else{
        RCLCPP_INFO(client->get_logger(),"请求异常！");
    }
    rclcpp::shutdown();

    return 0;
}
