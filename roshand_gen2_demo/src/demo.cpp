#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "roshand_gen2_msgs/HandCommandAction.h"    

bool finished;
 
typedef actionlib::SimpleActionClient<roshand_gen2_msgs::HandCommandAction> Client; 

void doneaction(const actionlib::SimpleClientGoalState& state, const roshand_gen2_msgs::HandCommandResultConstPtr& result)
{
    ROS_INFO("Goal arrived");
    finished = result -> finish;
}
 

void activeaction()
{

    ROS_INFO("Goal arriving...................");
}
 

void feedbackaction(const roshand_gen2_msgs::HandCommandFeedbackConstPtr& feedback)
{


}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "roshand_action_client_demo"); 

    Client client("/roshand_gen2_bringup/HandCommand", true); 

    ROS_INFO("Waiting for action server to start.");

    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    roshand_gen2_msgs::HandCommandGoal demo_goal; 
    finished = false;
    ROS_INFO("Open the hand without sensor.....................");
    demo_goal.use_sensor = false;
    demo_goal.goal_finger_distance = 80;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);

    while(!finished){};
    finished = false;    
    ROS_INFO("Close the hand without sensor...................");
    demo_goal.use_sensor = false;
    demo_goal.goal_finger_distance = 2;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);


    while(!finished){};
    finished = false;    
    ROS_INFO("Close the hand with sensor.......................");
    demo_goal.use_sensor = true;
    demo_goal.sensor_threadhold = 10;
    demo_goal.close_step_mag = 36;
    demo_goal.open_step_masg = 2;

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);



    while(!finished){};
    finished = false;    
    ROS_INFO("Close the hand with sensor.......................");
    demo_goal.use_sensor = true;
    demo_goal.sensor_threadhold = 10;
    demo_goal.close_step_mag = 36;
    demo_goal.open_step_masg = 2;

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);

    while(!finished){};
    finished = false;
    ROS_INFO("Open the hand without sensor.....................");
    demo_goal.use_sensor = false;
    demo_goal.goal_finger_distance = 80;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);

    while(!finished){};
    finished = false;

    ros::spin();
 
    return 0;
}
