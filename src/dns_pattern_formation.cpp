#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
bool robotsAhead = false;
bool robotsBehind = false;
float targetBearing = 0.;

float formationNormal = 0.;
float formationPerp = 90.;

void BlobBearingsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  
  // Get number of blobs detected
  int nBlobs = msg->data.size();

  float minDiff = 9999.;
  float 

  for(int i = 0; i < nBlobs; i++){
    float blobBearing = fabs(msg->data[i]);

    // Determine if robot is ahead or behind.
    if(fabs(blobBearing) > 90.){
      robotsBehind = true;
    }
    else{
      robotsAhead = true;
    }
    
    // Find blob which is closest to the formation normal.
    if(fabs(formationNormal - blobBearing) < minDiff){
      minDiff = fabs(formationNormal - blobBearing);
      targetBearing = blobBearing;
    } 

  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "evo_pattern_formation");

  ros::NodeHandle n;
 
  ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("targetHeading", 1000);
  ros::Publisher linearVel_pub = n.advertise<std_msgs::Float64>("targetLinearVelocity", 1000);

  ros::Subscriber currentHeadingSub = n.subscribe("blobBearings", 1000, BlobBearingsCallback);  

  ros::Rate loop_rate(10);

  bool keepGoing = true;
  
  while (ros::ok() and keepGoing){
    
    std_msgs::Float64 headingMsg;
    std_msgs::Float64 linearVelMsg;

    float heading;
    float linearVel;
    
    if(robotsBehind and !robotsAhead){
      heading = formationNormal + 180.;
      linearVel = cos(formationNormal - targetBearing);
    }
    else{
      heading = targetBearing + 90.;
      linearVel = cos(formationNormal - targetBearing);
    }
    
    // deal with issues arising from driving "south"
    //if( heading > 170.) heading = 170.;
    //if( heading <-170.) heading = -170.;
    
    headingMsg.data = heading;
    linearVelMsg.data = linearVel;

    heading_pub.publish(headingMsg);
    linearVel_pub.publish(linearVelMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
