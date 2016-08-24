#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"

#include "Vector2d.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
bool robotsAhead = false;
bool robotsBehind = false;
float targetBearing = 0.;

float formationNormal = 0.;
float formationPerp = 90.;
Vector2d * formationNormalVector = new Vector2d(formationNormal*M_PI/180.);

float currentHeading = 0.;
float currentLinearVel = 0.;

bool firstBlobs = false;

void BlobBearingsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

   
  // Get number of blobs detected
  int nBlobs = msg->data.size();

  // reset values
  float maxDot = -2.;
  robotsBehind = false;
  robotsAhead = false;  
  
  for(int i = 0; i < nBlobs; i++){
    firstBlobs = true;
    float blobBearing = fabs(msg->data[i]);
    Vector2d * blobBearingVector = new Vector2d(blobBearing*M_PI/180.);

    // Calculate dot product 
    float dot = DotProduct(blobBearingVector, formationNormalVector);

    // Determine if robot is ahead or behind.
    if(dot < 0.){
      robotsBehind = true;
    }
    else{
      robotsAhead = true;
    }
    
    // Find blob which is closest to the formation normal.
    if( dot > maxDot){
      maxDot = dot;
      targetBearing = blobBearing;
    } 

  }
}

void CurrentHeadingCallback(const std_msgs::Float64::ConstPtr& msg){
  currentHeading = msg->data;
}

void CurrentLinearVelCallback(const std_msgs::Float64::ConstPtr& msg){
  currentLinearVel = msg->data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "evo_pattern_formation");

  ros::NodeHandle n;
 
  //ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("targetHeading", 1000);
  //ros::Publisher linearVel_pub = n.advertise<std_msgs::Float64>("targetLinearVelocity", 1000);

  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber blobBearing_Sub = n.subscribe("blobBearings", 1000, BlobBearingsCallback);  
  ros::Subscriber currentHeading_Sub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);
  ros::Subscriber currentLinearVel_Sub = n.subscribe("currentLinearVel", 1000, CurrentLinearVelCallback);
  
  ros::Rate loop_rate(10);

  bool keepGoing = true;
  
  while (ros::ok() and keepGoing){

    if(!firstBlobs){
      printf("Waiting for first blob...\n");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }
    //std_msgs::Float64 headingMsg;
    //std_msgs::Float64 linearVelMsg;

    float targetHeading;
    float targetLinearVel;
    
    Vector2d * targetBearingVector = new Vector2d(targetBearing*M_PI/180.);
    float dot = DotProduct(targetBearingVector, formationNormalVector);
    
    if(robotsBehind and !robotsAhead){
      targetHeading = formationNormalVector->GetAngle();
      targetLinearVel = dot;
    }
    else if(robotsAhead){
      targetHeading = targetBearingVector->GetPerp()->GetAngle();
      targetLinearVel = dot;
    }
    else{
      targetHeading = formationNormalVector->GetPerp()->GetAngle();
      targetLinearVel = 0.;
    }
    
    //if( fabs(targetLinearVel) < 0.01) targetLinearVel = 0.;
    //else {
    //  if(targetLinearVel > 0.) targetLinearVel = 1.;
    //  else targetLinearVel = -1.;
    //}

    //if(targetLinearVel > 0.) targetLinearVel = 1.;
    //else if(targetLinearVel < 0.) targetLinearVel = -1.;
    //else targetLinearVel = 0.;
    
    // Change to degrees
    targetHeading = targetHeading * 180. / M_PI;
    
    geometry_msgs::Twist msg;

    // Use a proportional controller to determine v and w
    float headingError = -1.*(targetHeading - currentHeading) ;
      
    if(fabs(headingError) > 180.){
      headingError = 180. - headingError;
    }
    
    // Now convert to radians (per second)
    headingError = headingError * M_PI/180.;
    
    // set values of twist msg.
    msg.linear.x = 0.075*targetLinearVel;
    msg.angular.z = 2.*headingError;
    msg.angular.y = -1.;

    // Need to remove excess digits for transmission over serial.
    msg.linear.x = roundf(msg.linear.x * 100.) / 100.;
    msg.angular.z = roundf(msg.angular.z * 100.) / 100.; 
    
    // publish twist msg.
    twist_pub.publish(msg);
  
    printf("RobotAhead: %d\tRobotBehind: %d\n", robotsAhead, robotsBehind);
    printf("Bearing to Target: %f\n", targetBearingVector->GetAngle()*180./M_PI);
    printf("Target Vel: %f (%f)\n", targetLinearVel, currentLinearVel);
    printf("Target Heading: %f (%f)\n", targetHeading, currentHeading);
    printf("Dot: %f\n", dot);
    printf("Msg: %f\t%f\n", msg.linear.x, msg.angular.z);
    //printf("%f\t%f\t%f\n", dot, msg.linear.x, msg.angular.z);
    printf("~~~~~~~~~~~~~~~~~~~~~~~\n");
    //getchar();
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
};
