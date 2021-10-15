#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "beginner_tutorials/myAction.h"

class myActionServer
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<beginner_tutorials::myAction> as;

	beginner_tutorials::myFeedback feedback;
	beginner_tutorials::myResult result;

	std::string action_name;
	int goal;
	int progress;
public:
	myActionServer(std::string name):
	as(nh_,name,boost::bind(&myActionServer::executeCB,this,_1),false),//收到目标值的回调函数
	action_name(name)//构造函数的一种写法，列表初始化，所以用逗号
	{
		as.registerPreemptCallback(boost::bind(&myActionServer::preemptCB,this));//抢占函数
		as.start();   
	}

	void executeCB(const beginner_tutorials::myGoalConstPtr &goal)
	{
		ROS_INFO("%s is progressing the goal [%d]",action_name.c_str(),goal->goal_num);
		ros::Rate rate(5);
		for(progress=1;progress<=goal->goal_num;++progress)
		{
			if(!as.isActive()||as.isPreemptRequested())
				return;
			if(goal->goal_num==progress)
			{
				result.result_num=progress;
				as.setSucceeded(result);
				ROS_INFO("%s succeeded the goal [%d]",action_name.c_str(),goal->goal_num);				
			}
			else
			{
				feedback.current_num=progress;
				as.publishFeedback(feedback);
				ROS_INFO("server send feedback:[%d]",feedback.current_num);
			}
			rate.sleep();
		}
		
	}
	void preemptCB()
	{
		ROS_ERROR("%s got preempted!",action_name.c_str());
		result.result_num=progress;
		as.setPreempted(result,"I got preempted!");
	}
};


int main(int argc,char** argv)
{
	ros::init(argc,argv,"action_server");
	myActionServer actionServer(ros::this_node::getName());//ros::this_node::getName()的作用就是得到上面的”action_server“,直接写”action_server“也可以；

	ROS_INFO("star action_server");
	ros::spin();
	return 0;
}