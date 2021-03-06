#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "bupimo_msgs/Blob.h"
#include "bupimo_msgs/BlobArray.h"
#include "bupimo_msgs/VelocityCommand.h"

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

void BlobBearingsCallback(const bupimo_msgs::BlobArray::ConstPtr& msg){

   
  // Get number of blobs detected
  int nBlobs = msg->blobArray.size();

  // reset values
  float maxDot = -2.;
  robotsBehind = false;
  robotsAhead = false;  
  
  for(int i = 0; i < nBlobs; i++){
    firstBlobs = true;

    float blobBearing = fabs( msg->blobArray[i].bearing);
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
 
  ros::Publisher command_pub = n.advertise<bupimo_msgs::VelocityCommand>("patternFormationBehaviour", 1000);

  ros::Subscriber blobBearing_Sub = n.subscribe("blobsGlobal", 1000, BlobBearingsCallback);  
  ros::Subscriber currentHeading_Sub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);
  ros::Subscriber currentLinearVel_Sub = n.subscribe("currentLinearVel", 1000, CurrentLinearVelCallback);
  
  ros::Rate loop_rate(10);

  while (ros::ok()){

    if(!firstBlobs){
      printf("Waiting for first blob...\n");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

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

    if( fabs(targetLinearVel) < 0.09) targetLinearVel = 0.;

    
    // Change to degrees
    targetHeading = targetHeading * 180. / M_PI;

    // Now publish the velocity command
    bupimo_msgs::VelocityCommand msg;

    msg.bearing = targetHeading;
    msg.linearSpeed = targetLinearVel;

    command_pub.publish(msg);
  
    /*printf("RobotAhead: %d\tRobotBehind: %d\n", robotsAhead, robotsBehind);
    printf("Bearing to Target: %f\n", targetBearingVector->GetAngle()*180./M_PI);
    printf("Target Vel: %f (%f)\n", targetLinearVel, currentLinearVel);
    printf("Target Heading: %f (%f)\n", targetHeading, currentHeading);
    printf("Dot: %f\n", dot);
    printf("Msg: %f\t%f\n", msg.linear.x, msg.angular.z);
    printf("%f\t%f\t%f\n", dot, msg.linear.x, msg.angular.z);
    printf("~~~~~~~~~~~~~~~~~~~~~~~\n");
    getchar();*/
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
};
