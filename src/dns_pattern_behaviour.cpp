#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

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

void BlobBearingsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  
  // Get number of blobs detected
  int nBlobs = msg->data.size();

  // reset values
  float maxDot = -2.;
  robotsBehind = false;
  robotsAhead = false;  
  
  for(int i = 0; i < nBlobs; i++){
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

    Vector2d * targetBearingVector = new Vector2d(targetBearing*M_PI/180.);
    float dot = DotProduct(targetBearingVector, formationNormalVector);
    
    if(robotsBehind and !robotsAhead){
      heading = ScalarMultiplyVector(-1., formationNormalVector)->GetAngle();
      linearVel = fabs(dot);
    }
    else{
      heading = targetBearingVector->GetPerp()->GetAngle();
      linearVel = dot;
    }

    if(linearVel < 0.01) linearVel = 0.;
    else linearVel = 1.;

    heading = heading * 180. / M_PI;
    
    // deal with issues arising from driving "south"
    if( heading > 170.) heading = 170.;
    if( heading <-170.) heading = -170.;   
    //printf("%f\t%f\n", heading, linearVel);
   
    headingMsg.data = heading;
    linearVelMsg.data = linearVel;
    
    printf("RobotAhead: %d\tRobotBehind: %d\n", robotsAhead, robotsBehind);
    printf("Target Bearing: %f\n", targetBearingVector->GetAngle()*180./M_PI);
    printf("Command Vel: %f %f\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n", heading, linearVel);
    //getchar();
    
    heading_pub.publish(headingMsg);
    linearVel_pub.publish(linearVelMsg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
};
