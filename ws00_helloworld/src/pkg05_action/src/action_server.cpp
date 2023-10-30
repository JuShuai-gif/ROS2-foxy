#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pkg03_custom_message_cpp/action/progress.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using pkg03_custom_message_cpp::action::Progress;

class ProgressActionServer:public rclcpp::Node
{
public:
  ProgressActionServer():Node("progress_action_server_node_cpp"){
    RCLCPP_INFO(this->get_logger(),"action 服务端创建！等待请求...");
    // 创建动作服务端对象
    /*
    
    */
    server_ = rclcpp_action::create_server<Progress>(
      this,
      "get_sum",
      std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
      std::bind(&ProgressActionServer::handle_cancel,this,_1),
      std::bind(&ProgressActionServer::handle_accepted,this,_1)
      );
  }
  /*处理提交的目标值（回调函数）
  std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
  */
 rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal> goal){
  (void)uuid;
  if (goal->num <= 1)
  {
    RCLCPP_INFO(this->get_logger(),"提交的目标值必须大于1！");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(),"提交的目标值合法！");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }

  /*处理取消请求（回调函数）
  std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  */
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(),"接收到任务取消请求！");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  /*生成连续反馈与最终响应（回调函数）
  std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  */
 void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
 {
    // 1. 生成连续反馈返回给客户端
    // 首先要获取目标值，然后遍历，遍历中进行累加，且每循环一次就计算进度，并作为连续反馈发布
    RCLCPP_INFO(this->get_logger(),"处理目标！");
    // 设置休眠
    rclcpp::Rate loop_rate(1);

    int num = goal_handle->get_goal()->num;

    int sum = 0;
    auto feedback = std::make_shared<Progress::Feedback>();
    auto result = std::make_shared<Progress::Result>();
    for (int i = 1; i <= num; ++i)
    {
      sum +=i;
      double progress = i/(double)num;
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),"连续反馈中，进度%.2f",progress);

      // 判断是否接收到了取消请求
      if (goal_handle->is_canceling())
      {
        result->sum = sum;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(),"任务取消了");
        return;
      }
      
      // 如果接收到了，终止程序--return

      loop_rate.sleep();
    }

    // 2. 生成最终响应结果
    if(rclcpp::ok())
    {
      
      result->sum = sum;
      RCLCPP_INFO(this->get_logger(),"最终结果：%d",sum);
    }



 }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
  {
    std::thread(std::bind(&ProgressActionServer::execute,this,goal_handle)).detach();
  }


private:
  rclcpp_action::Server<Progress>::SharedPtr server_;
};




int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ProgressActionServer>());
  rclcpp::shutdown();
  return 0;
}
