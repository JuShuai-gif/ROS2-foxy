#include "rclcpp/rclcpp.hpp"
#include "pkg03_custom_message_cpp/srv/add_ints.hpp"

using pkg03_custom_message_cpp::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;

class MininalService:public rclcpp::Node
{
public:
  MininalService():Node("Minimal_service"){
    server = this->create_service<AddInts>("add_ints",std::bind(&MininalService::add,this,_1,_2));
    RCLCPP_INFO(this->get_logger(),"add_ints服务端启动完毕，等待请求提交...");
  }
private:
  
  rclcpp::Service<AddInts>::SharedPtr server;
  void add(const AddInts::Request::SharedPtr req,const AddInts::Response::SharedPtr res)
  {
    res->sum = req->num1 + req->num2;
    RCLCPP_INFO(this->get_logger(),"请求数据：(%d,%d),响应结果：%d",req->num1,req->num2,res->sum);

  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto server = std::make_shared<MininalService>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
