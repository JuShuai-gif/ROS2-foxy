#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pkg03_custom_message_cpp/action/progress.hpp"

using pkg03_custom_message_cpp::action::Progress;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionClient:public rclcpp::Node
{
public:
  ProgressActionClient():Node("progress_action_client_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"action 客户端创建！");
    // 创建动作客户端
    client_ = rclcpp_action::create_client<Progress>(this,"get_sum");
  
  
  }

  void send_goal(int num)
  {
    // 需要连接服务端
    if(!client_->wait_for_action_server(10s))
    {
        RCLCPP_ERROR(this->get_logger(),"服务连接失败！");
        return;
    }
    // 发送具体请求
    auto goal = Progress::Goal();
    goal.num = num;
    rclcpp_action::Client<Progress>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback,this,_1);
    options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback,this,_1,_2);
    options.result_callback = std::bind(&ProgressActionClient::result_callback,this,_1);
    auto future = client_->async_send_goal(goal,options);
  }
  // 处理关于目标值的服务端响应（回调函数）
  void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<Progress>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle){
        RCLCPP_INFO(this->get_logger(),"目标请求被服务端拒绝！");
    }else{
        RCLCPP_INFO(this->get_logger(),"目标处理中！");
    }
    
  }
  // 处理连续反馈（回调函数）
  void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,const std::shared_ptr<const Progress::Feedback> feedback)
  {
    (void)goal_handle;
    double progress = feedback->progress;
    int pro = (int)(progress*100);
    RCLCPP_INFO(this->get_logger(),"当前进度：%d%%",pro);
  }
  // 处理最终响应（回调函数）
  void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(),"最终结果：%d",result.result->sum);
    }else if(result.code == rclcpp_action::ResultCode::ABORTED){
        RCLCPP_INFO(this->get_logger(),"被中断！");
    }else if(result.code == rclcpp_action::ResultCode::CANCELED){
        RCLCPP_INFO(this->get_logger(),"被取消！");
    }else{
        RCLCPP_INFO(this->get_logger(),"未知结果异常！！！");
    }
    
  }
private:
  /* data */
  rclcpp_action::Client<Progress>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    if (argc!=2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整型数据！");
        return 1;
        /* code */
    }
    
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ProgressActionClient>();
  node->send_goal(atoi(argv[1]));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
