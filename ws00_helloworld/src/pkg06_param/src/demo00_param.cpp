//包含头文件
#include "rclcpp/rclcpp.hpp"

// 自定义节点类
class MyParam : public rclcpp::Node
{
private:
  
public:
  MyParam():Node("my_param_node_cpp"){
    /*
      // 参数对象创建
      rclcpp::Parameter p1("car_name","tiger");
      rclcpp::Parameter p2("height",1.68);
      rclcpp::Parameter p3("wheels",4);
      // 参数对象解析（获取健、值、将值转换成字符串.....）
      RCLCPP_INFO(this->get_logger(),"car_name = %s",p1.as_string().c_str());
      RCLCPP_INFO(this->get_logger(),"height = %.2f",p2.as_double());
      RCLCPP_INFO(this->get_logger(),"wheels = %ld",p3.as_int());
      // 解析值
      RCLCPP_INFO(this->get_logger(),"name = %s",p1.get_name().c_str());
      RCLCPP_INFO(this->get_logger(),"type = %s",p1.get_type_name().c_str());
      RCLCPP_INFO(this->get_logger(),"value2string = %s",p2.value_to_string().c_str());
    */  
  }

};

int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MyParam>());
  rclcpp::shutdown();
  return 0;
}
