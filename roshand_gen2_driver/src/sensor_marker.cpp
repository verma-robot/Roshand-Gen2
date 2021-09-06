
#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include "roshand_gen2_msgs/Hand.h"
#include "roshand_gen2_msgs/Finger.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>




using namespace std;

ros::Publisher sensor_pub;
//

void publish_sensors_to_rviz(const roshand_gen2_msgs::HandConstPtr& hand) {

  bool contact_val;




  visualization_msgs::MarkerArray marker_array;

  for (int finger = 0; finger < 2; finger++)
  {
    
    for (int i = 0; i < 14; i++)    
    {

      float radius = 0.004;
      float height = 0.005;

      visualization_msgs::Marker contact_marker;

      contact_val = hand->finger[finger].contact[i];

      if(finger==0 && i==0)contact_marker.header.frame_id="left_fingertips_sensor_1_link";
      if(finger==0 && i==1)contact_marker.header.frame_id="left_fingertips_sensor_2_link";
      if(finger==0 && i==2)contact_marker.header.frame_id="left_fingertips_sensor_3_link";
      if(finger==0 && i==3)contact_marker.header.frame_id="left_fingertips_sensor_4_link";
      if(finger==0 && i==4)contact_marker.header.frame_id="left_fingertips_sensor_5_link";
      if(finger==0 && i==5)contact_marker.header.frame_id="left_fingertips_sensor_6_link";
      if(finger==0 && i==6)contact_marker.header.frame_id="left_fingertips_sensor_7_link";
      if(finger==0 && i==7)contact_marker.header.frame_id="left_fingertips_sensor_8_link";
      if(finger==0 && i==8)contact_marker.header.frame_id="left_fingertips_sensor_9_link";
      if(finger==0 && i==9)contact_marker.header.frame_id="left_fingertips_sensor_10_link";
      if(finger==0 && i==10)contact_marker.header.frame_id="left_fingertips_sensor_11_link";
      if(finger==0 && i==11)contact_marker.header.frame_id="left_fingertips_sensor_12_link";
      if(finger==0 && i==12)contact_marker.header.frame_id="left_fingertips_sensor_13_link";
      if(finger==0 && i==13)contact_marker.header.frame_id="left_fingertips_sensor_14_link";

      if(finger==1 && i==0)contact_marker.header.frame_id="right_fingertips_sensor_1_link";
      if(finger==1 && i==1)contact_marker.header.frame_id="right_fingertips_sensor_2_link";
      if(finger==1 && i==2)contact_marker.header.frame_id="right_fingertips_sensor_3_link";
      if(finger==1 && i==3)contact_marker.header.frame_id="right_fingertips_sensor_4_link";
      if(finger==1 && i==4)contact_marker.header.frame_id="right_fingertips_sensor_5_link";
      if(finger==1 && i==5)contact_marker.header.frame_id="right_fingertips_sensor_6_link";
      if(finger==1 && i==6)contact_marker.header.frame_id="right_fingertips_sensor_7_link";
      if(finger==1 && i==7)contact_marker.header.frame_id="right_fingertips_sensor_8_link";
      if(finger==1 && i==8)contact_marker.header.frame_id="right_fingertips_sensor_9_link";
      if(finger==1 && i==9)contact_marker.header.frame_id="right_fingertips_sensor_10_link";
      if(finger==1 && i==10)contact_marker.header.frame_id="right_fingertips_sensor_11_link";
      if(finger==1 && i==11)contact_marker.header.frame_id="right_fingertips_sensor_12_link";
      if(finger==1 && i==12)contact_marker.header.frame_id="right_fingertips_sensor_13_link";
      if(finger==1 && i==13)contact_marker.header.frame_id="right_fingertips_sensor_14_link";


      contact_marker.ns = "contact_markers";
    

     if (contact_val) {
          contact_marker.scale.x = radius;
          contact_marker.scale.y = radius;
          contact_marker.scale.z = height;
          contact_marker.color.r = 255;
          contact_marker.color.g = 0.0;
          contact_marker.color.b = 0.0;
          contact_marker.color.a = 1.0;
          contact_marker.pose.position.z = 0.0005;

     }
     else {
          contact_marker.scale.x = radius ;
          contact_marker.scale.y = radius ;
          contact_marker.scale.z = height ;
          contact_marker.color.r = 1.0;
          contact_marker.color.g = 1.0;
          contact_marker.color.b = 1.0;
          contact_marker.color.a = 1.0;
          contact_marker.pose.position.z = 0.000;
     }
     contact_marker.pose.position.x = 0.0;
     contact_marker.pose.position.y = 0;


     contact_marker.pose.orientation.y = 0.707;
     contact_marker.pose.orientation.w = 0.707;


      contact_marker.id = finger * 14 + i;

      marker_array.markers.push_back(contact_marker);

    }
  }

  sensor_pub.publish(marker_array);


}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "roshand_gen2_visualizer");
    ros::NodeHandle n;

    std::string ns;
    n.getParam("ns", ns);

    sensor_pub = n.advertise<visualization_msgs::MarkerArray>(ns + "visualization_marker_array", 100);


    ros::Subscriber pose_sub = n.subscribe(ns + "/RosHand_Data", 1, publish_sensors_to_rviz);

    ros::spin();
    return 0;
}


