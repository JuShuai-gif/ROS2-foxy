//包含头文件
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 自定义节点类
class ParamClient : public rclcpp::Node
{
private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;

public:
  ParamClient():Node("param_client_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"参数客户端创建了！");
    // 创建参数客户端对象
    // 参数1：当前对象依赖的节点
    // 参数2：参数服务端节点名称
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
    /*
    问题：服务通信不是通过服务话题关联吗？为什么参数客户端是通过参数服务端的节点名称关联？
    答：
        1. 参数服务端启动后，地层封装了多个服务通信的服务端
        2. 每个服务端的话题，都是采用  /服务端节点名称/*****
        3. 参数客户端创建后，也会封装多个服务通信的客户端
        4. 这些客户端与服务端相呼应，也要使用相同的话题，因此客户端再创建时需要使用服务端节点名称
    
    */
    
    }
    // 连接服务端
    bool connect_server(){
        while (!param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(),"服务连接中.....");
        }
        return true;
    }
    // 查询参数
    void get_param()
    {
        RCLCPP_INFO(this->get_logger(),"---------参数查询操作---------");

        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double width = param_client_->get_parameter<double>("width");

        RCLCPP_INFO(this->get_logger(),"car_name = %s",car_name.c_str());
        RCLCPP_INFO(this->get_logger(),"width = %.2f",width);

        auto params = param_client_->get_parameters({"car_name","width","wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(),"(%s = %s)",param.get_name().c_str(),param.value_to_string().c_str());
        }

        RCLCPP_INFO(this->get_logger(),"是否包含car_name? %d",param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含height? %d",param_client_->has_parameter("width"));

        // 如果这里使用this的话，他不会含有这些参数，因为在当前节点中，并没有声明这几个参数
        RCLCPP_INFO(this->get_logger(),"是否包含car_name? %d",this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含height? %d",this->has_parameter("width"));

    }
    // 修改参数
    void update_param(){

        RCLCPP_INFO(this->get_logger(),"----------参数修改操作-------");

        param_client_->set_parameters({rclcpp::Parameter("car_name","pig"),
            rclcpp::Parameter("width",3.0),
            rclcpp::Parameter("length",5.0)
        });
        RCLCPP_INFO(this->get_logger(),"新设置参数:%.2f",param_client_->get_parameter<double>("length"));
    }

};

int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  auto client = std::make_shared<ParamClient>();
  bool flag = client->connect_server();
  if (!flag)
  {
    return 0;
  }
  client->get_param();
  client->update_param();
  client->get_param();
  
  //rclcpp::spin(std::make_shared<ParamClient>());
  rclcpp::shutdown();
  return 0;
}
