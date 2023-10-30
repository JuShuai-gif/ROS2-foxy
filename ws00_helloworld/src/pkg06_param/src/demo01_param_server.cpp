//包含头文件
#include "rclcpp/rclcpp.hpp"

// 自定义节点类
class ParamServer : public rclcpp::Node
{
private:
  
public:
    // 如果允许删除参数，那么需要通过NodeOption生命
    /*
    如果不加这句话的话，rclcpp::NodeOptions().allow_undeclared_parameters(true)
    this->set_parameter()不能被使用
    否则会报这样的异常：
    terminate called after throwing an instance of 'rclcpp::exceptions::ParameterNotDeclaredException'
  what():  parameter 'height' cannot be set because it was not declared

    */
  ParamServer():Node("param_server_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true)){
    RCLCPP_INFO(this->get_logger(),"参数服务端创建了！");
  }

    //增
    void declare_param(){
        RCLCPP_INFO(this->get_logger(),"---------------增-------------");
        this->declare_parameter("car_name","tiger");
        this->declare_parameter("width",1.55);
        this->declare_parameter("wheels",5);
        //另一种设置参数的方式
        this->set_parameter(rclcpp::Parameter("height",2.00));
    }

    //查
    void get_param(){
        RCLCPP_INFO(this->get_logger(),"---------------查-------------");
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(),"key = %s,value = %s",car.get_name().c_str(),car.as_string().c_str());
        auto params = this->get_parameters({"car_name","width","wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(),"(%s = %s)",param.get_name().c_str(),param.value_to_string().c_str());
        }

        RCLCPP_INFO(this->get_logger(),"是否包含car_name? %d",this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含height? %d",this->has_parameter("width"));

    }

    //改
    void update_param(){
        RCLCPP_INFO(this->get_logger(),"---------------改-------------");
        this->set_parameter(rclcpp::Parameter("width",1.75));
        RCLCPP_INFO(this->get_logger(),"width = %.2f",this->get_parameter("width").as_double());

    }

    //删
    void del_param(){
        RCLCPP_INFO(this->get_logger(),"---------------删-------------");
        this->undeclare_parameter("car_name");
        // 不能删除声明的参数，而那些被设置的参数可以被删除，比如当前的例子，car_name不能被删除，而height可以被删除
        // 但是这里没报错，当前版本是foxy，而hum会报错
        RCLCPP_INFO(this->get_logger(),"删除后还包含car_name吗？%d",this->has_parameter("car_name"));

        this->undeclare_parameter("height");
        // 不能删除声明的参数，而那些被设置的参数可以被删除，比如当前的例子，car_name不能被删除，而height可以被删除
        RCLCPP_INFO(this->get_logger(),"删除后还包含car_name吗？%d",this->has_parameter("height"));

    }

};

int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ParamServer>();
  node->declare_param();
  node->get_param();
  node->update_param();
  //node->del_param();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
