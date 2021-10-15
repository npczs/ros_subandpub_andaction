#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "beginner_tutorials/myAction.h"

void finishCB(const actionlib::SimpleClientGoalState& state, const beginner_tutorials::myResultConstPtr& result)//服务端发送执行完成后进行的回调函数
{
	ROS_WARN("Action status: [%s],result:[%d]",state.toString().c_str(),result->result_num);
}
void startCB()//服务端开始接受goal开始执行动作后执行的回调函数
{
	ROS_INFO("Action server start action");
}
void feedbackCB(const beginner_tutorials::myFeedbackConstPtr& feedback)//服务端执行动作时，从服务端得到反馈时调用的回调函数
{
	ROS_INFO("Client get feedback [%d]",feedback->current_num);
}
int main (int argc,char **argv)
{
	ros::init(argc,argv,"action_client");
	if (argc!=3)//运行需要附上两个参数 goal和超时时间
	{
		ROS_INFO("%d",argc);
		ROS_INFO("Usage:demo_action_client <goal> <time_to_preempt_in_sec>");
		return 1;
	}
	//创建客户端，true时代表客户端生成自己的线程
	actionlib::SimpleActionClient<beginner_tutorials::myAction> ac("action_server",true);
	ROS_INFO("waiting for action_server to start");
	ac.waitForServer();
    beginner_tutorials::myGoal goal;
    goal.goal_num=atoi(argv[1]);
    ROS_INFO("Sending goal [%d] and preempt time of [%d]",goal.goal_num,atoi(argv[2]));
    ac.sendGoal(goal,&finishCB,&startCB,&feedbackCB);

    //wait for the action to return
    bool finished_before_timeout=ac.waitForResult(ros::Duration(atoi(argv[2])));
    actionlib::SimpleClientGoalState state=ac.getState();
    ROS_INFO("Action state:[%s]",state.toString().c_str());
    if(!finished_before_timeout)
    {
    	ac.cancelGoal();
    	state=ac.getState();
    	ROS_INFO("Action state:[%s]",state.toString().c_str());
    	ROS_ERROR("Action didin't finish before the time out");
    }

    return 0;
    

	



}
