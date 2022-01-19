#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "roshand_gen2_msgs/HandCommandAction.h"    

 
typedef actionlib::SimpleActionClient<roshand_gen2_msgs::HandCommandAction> Client; 

void doneaction(const actionlib::SimpleClientGoalState& state, const roshand_gen2_msgs::HandCommandResultConstPtr& result)
{

    //if(result -> finish == true)ROS_INFO("Goal arrived....");
    //else ROS_WARN("Can't arrive the Goal, please retry...");


}
 

void activeaction()
{
    ROS_INFO("Get a new Goal............");
}
 

void feedbackaction(const roshand_gen2_msgs::HandCommandFeedbackConstPtr& feedback)
{
    bool resl;
    roshand_gen2_msgs::HandCommandFeedback get_back;
    get_back = *feedback;
    resl = get_back.state;


    //std::cout << (int16_t)(resl)<< std::endl;
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "roshand_action_client_demo"); 

    Client client("/roshand_gen2_bringup/HandCommand", true); 

    ROS_INFO("Waiting for action server to start.");

    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    roshand_gen2_msgs::HandCommandGoal demo_goal; 
    ROS_INFO("Open the hand without sensor.....................");
    demo_goal.use_sensor = false;
    demo_goal.target_finger_distance = 50;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);

    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived the goal.....");
    }
    else
    {

        ROS_WARN("Can't arrive the goal....");
        ROS_WARN("Cancle the action....");
        client.cancelAllGoals();
 
    }

    ROS_INFO("Close the hand without sensor...................");
    demo_goal.use_sensor = false;
    demo_goal.target_finger_distance = 2;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);

    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived the goal.....");
    }
    else
    {

        ROS_WARN("Can't arrive the goal....");
        ROS_WARN("Cancle the action....");
        client.cancelAllGoals();
 
    }


    ROS_INFO("Open the hand without sensor.....................");
    demo_goal.use_sensor = false;
    demo_goal.target_finger_distance = 80;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);


    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived the goal.....");
    }
    else
    {

        ROS_WARN("Can't arrive the goal....");
        ROS_WARN("Cancle the action....");
        client.cancelAllGoals();
 
    }

    ROS_INFO("Close the hand with sensor.......................");
    demo_goal.use_sensor = true;
    demo_goal.sensor_threshold = 10;
    demo_goal.close_step_mag = 16;

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);


    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived the goal.....");
    }
    else
    {

        ROS_WARN("Can't arrive the goal....");
        ROS_WARN("Cancle the action....");
        client.cancelAllGoals();
 
    }

    ROS_INFO("Open the hand without sensor.....................");
    demo_goal.use_sensor = false;
    demo_goal.target_finger_distance = 80;    

    client.sendGoal(demo_goal,  doneaction, activeaction, feedbackaction);


    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived the goal.....");
        ros::shutdown();

    }
    else
    {

        ROS_WARN("Can't arrive the goal....");
        ROS_WARN("Cancle the action....");
        client.cancelAllGoals();
        ros::shutdown();

 
    }


    ros::spin();
 
    return 0;
}
