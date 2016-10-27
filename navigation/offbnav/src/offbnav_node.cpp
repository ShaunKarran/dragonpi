/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <stdlib.h>

using namespace std;





float wayPointTolerance = 0.2; // 20 cm
float orientationTolerance = 0.1; // ~6 degrees


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Get current position from MAVROS (UAV emu)
/*
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}
*/

//cb for getting UAV position from vicon Bridge
geometry_msgs::TransformStamped current_pos_t;
void pos_t_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    current_pos_t = *msg;
}

bool imageProcCompl;
void imageProcCompl_cb(const std_msgs::Bool::ConstPtr& msg){
    imageProcCompl = true;
}

bool acidFound;
void acidFound_cb(const std_msgs::Bool::ConstPtr& msg){
    acidFound = msg->data;

}

bool miscFound;
void miscFound_cb(const std_msgs::Bool::ConstPtr& msg){
    miscFound = msg->data;
}

bool flamableFound;
void flamableFound_cb(const std_msgs::Bool::ConstPtr& msg){
    flamableFound = msg->data;

}


/** Helper functions, from kye for controling UAV orientation **/

geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q) {
    geometry_msgs::Vector3 e;

    double q2sqr = q.y * q.y;
    double t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0;
    double t1 = +2.0 * (q.x * q.y + q.w * q.z);
    double t2 = -2.0 * (q.x * q.z - q.w * q.y);
    double t3 = +2.0 * (q.y * q.z + q.w * q.x);
    double t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    e.x = atan2(t3, t4);
    e.y = asin(t2);
    e.z = atan2(t1, t0);

    return e;
}


geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

    return q;
}

/* returns true if x,y and z are within waypoint tolerance of each other of goal waypoint and current waypoint of uav
** and if the orientation is within orientation tolerance with respect to */

bool goalPositionReached(geometry_msgs::PoseStamped goalPose, geometry_msgs::TransformStamped currentPose){
	bool goalReached = false;
        double xDif = fabs(goalPose.pose.position.x - currentPose.transform.translation.x);
        double yDif = fabs(goalPose.pose.position.y - currentPose.transform.translation.y);
        double zDif = fabs(goalPose.pose.position.z - currentPose.transform.translation.z);

	//convert current orientation to euler
	geometry_msgs::Vector3 currentPoseEuler = toEuler(currentPose.transform.rotation);
        double zOriDif = abs(goalPose.pose.orientation.z - currentPoseEuler.z);

	if(xDif <= wayPointTolerance && yDif <= wayPointTolerance && zDif <= wayPointTolerance && zOriDif <= orientationTolerance){
            //ROS_INFO("Calculated Difference: %0.2f, %0.2f, %0.2f", xDif, yDif, zDif);

		goalReached = true;
	}

	return goalReached;
}

/*
bool goalPositionReached(geometry_msgs::PoseStamped goalPose, geometry_msgs::PoseStamped currentPose){
	bool goalReached = false;
	float xDif = abs(goalPose.pose.position.x - currentPose.pose.position.x);
	float yDif = abs(goalPose.pose.position.y - currentPose.pose.position.y);
	float zDif = abs(goalPose.pose.position.z - currentPose.pose.position.z);

	//convert current orientation to euler
	geometry_msgs::Vector3 currentPoseEuler = toEuler(currentPose.pose.orientation);
	float zOriDif = abs(goalPose.pose.orientation.z - currentPoseEuler.z);

	if(xDif <= wayPointTolerance && yDif <= wayPointTolerance && zDif <= wayPointTolerance && zOriDif <= orientationTolerance){
		goalReached = true;
	}

	return goalReached;
}
*/

void readInCords(char* filePath, std::vector<geometry_msgs::PoseStamped>* wayPoints, geometry_msgs::Vector3 rotate)
{
  std::string line;
  std::ifstream inFile(filePath);

  if(inFile.is_open()){
    while(!inFile.eof()) // format is [x, y, z]
    {
      geometry_msgs::PoseStamped pose;
      std::getline(inFile, line, ',');
      if(line == ""){
        break;
      }
      line.erase(remove(line.begin(), line.end(), '['), line.end());
      pose.pose.position.x = atof (line.c_str());

      std::getline(inFile, line, ',');
      pose.pose.position.y = atof (line.c_str());

      std::getline(inFile, line);
      line.erase(remove(line.begin(), line.end(), ']'), line.end());
      pose.pose.position.z = atof (line.c_str());

      pose.pose.orientation = toQuaternion(rotate);

      wayPoints->push_back(pose);
    }
  inFile.close();
  ROS_INFO("Succesful read of %s", filePath);
  }else{
    ROS_INFO("Unsuccesful read of %s", filePath);
  }
}


int main(int argc, char **argv)
{
  // construct vector for way points
  std::vector<geometry_msgs::PoseStamped> wayPoints;
  geometry_msgs::Vector3 rotate;// used for rotation when constucting wayPoints
  int rotate90DegClockWise = 90; // for readability UAV will roate 90deg after each wall is searched

  // read in ground way points
  int numGroundWayPoints;
  char pathToGround[] = "/home/mitchell/catkin_ws/src/offbnav/src/groundWayPoints.txt";
  rotate.z = 0; // 0 rotation for ground way points
  readInCords(pathToGround, &wayPoints, rotate);
  numGroundWayPoints = wayPoints.size(); // use this for knowing when to rotate servo

  //read in right side wayPoints
  char pathToRight[] = "/home/mitchell/catkin_ws/src/offbnav/src/rightSideWayPoints.txt";
  rotate.z += rotate90DegClockWise;
  readInCords(pathToRight, &wayPoints, rotate);


  //read in bottom side wayPoints
  char pathToBottom[] = "/home/mitchell/catkin_ws/src/offbnav/src/bottomSideWayPoints.txt";
  rotate.z += rotate90DegClockWise;
  readInCords(pathToBottom, &wayPoints, rotate);


  //read in left side wayPoints
  char pathToLeft[] = "/home/mitchell/catkin_ws/src/offbnav/src/leftSideWayPoints.txt";
  rotate.z += rotate90DegClockWise;
  readInCords(pathToLeft, &wayPoints, rotate);


  //read in top side wayPoints
  char pathToTop[] = "/home/mitchell/catkin_ws/src/offbnav/src/topSideWayPoints.txt";
  rotate.z += rotate90DegClockWise;
  readInCords(pathToTop, &wayPoints, rotate);

  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;


  ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>
          ("vicon/dragonpi/dragonpi", 10, pos_t_cb);

//    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
//            ("mavros/local_position/pose", 10, pos_cb);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 10, state_cb);

  ros::Subscriber image_proc_complete = nh.subscribe<std_msgs::Bool>
          ("/processing_complete",10, imageProcCompl_cb);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("/breadcrumb/reference/external", 10);

  ros::Publisher take_image_pub = nh.advertise<std_msgs::Empty>
          ("/take_image", 10);

  ros::Publisher move_servo = nh.advertise<std_msgs::Bool>
      ("/position_servo", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  /* image processing */

  ros::Subscriber acid_found = nh.subscribe<std_msgs::Bool>
          ("/acid",10, acidFound_cb);
  ros::Subscriber misc_found = nh.subscribe<std_msgs::Bool>
          ("/pmisc",10, miscFound_cb);
  ros::Subscriber flamable_found = nh.subscribe<std_msgs::Bool>
          ("/flamable",10, flamableFound_cb);






    /* TODO replace arbitary waypoints */
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /* Construct some useful waypoints take off land ect */
	  // this is our takeoff pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    rotate.z = 0;
    pose.pose.orientation = toQuaternion(rotate);

    // landing pose
    geometry_msgs::PoseStamped poseLand;
    poseLand.header.frame_id = "world";
    poseLand.pose.position.x = 0;
    poseLand.pose.position.y = 0;
    poseLand.pose.position.z = 0;
    rotate.z = 0;
    poseLand.pose.orientation = toQuaternion(rotate);

    // prepareland pose for dmg mitigation
    geometry_msgs::PoseStamped prepareLand;
    prepareLand.header.frame_id = "world";
    prepareLand.pose.position.x = 0;
    prepareLand.pose.position.y = 0;
    prepareLand.pose.position.z = 0.5;
    rotate.z = 0;
    prepareLand.pose.orientation = toQuaternion(rotate);


    bool navComplete = false;
    int waypointNumCompleted = 0;
    int numberOfWaypointsToComplete = wayPoints.size();

	  ros::Time currentTime;
    ros::Time flightTimeStart;
    ros::Duration totalFlightTime;

    std_msgs::Empty takeImage;
    bool takeImg = true;

    while(ros::ok()){ // need to timestamp poses pased

        pose.header.seq++;
        pose.header.stamp = ros::Time::now();

        if(!navComplete){
          if(goalPositionReached(pose, current_pos_t)){ // has UAV reached current waypoint
            if(waypointNumCompleted = 1)// UAV has taken off
            {
              flightTimeStart = ros::Time::now(); // get time before flight begins
            }else if(waypointNumCompleted==numberOfWaypointsToComplete){ // this is just the number of waypoints im passing
              ROS_INFO("NAV Complete, preparing for landing");
              totalFlightTime = ros::Time::now() - flightTimeStart;
              double flightTimeInMin = (totalFlightTime.toSec())/60;
              ROS_INFO("Total Flight time of mission is: %f minutes", flightTimeInMin);
              pose.pose = prepareLand.pose;
              waypointNumCompleted++;
            }else if(waypointNumCompleted==numberOfWaypointsToComplete+1) {
              navComplete = true;
              ROS_INFO("Landing");
              pose.pose = poseLand.pose;
            }else if(waypointNumCompleted==numGroundWayPoints){ // we are at the end of our ground way points need to move servo
              ROS_INFO("Moving the Servo");
              std_msgs::Bool move_servo_bool;
              move_servo_bool.data = true;
              move_servo.publish(move_servo_bool);
              waypointNumCompleted++;
            }else{
              if(takeImg){
                ROS_INFO("Way point reached");
                take_image_pub.publish(takeImage); // take image
                ROS_INFO("Instructing Image to capture photo.");
                takeImg = false; // avoid publishing every iteration
              }

            if(imageProcCompl){
              ROS_INFO("Image Process Complete.");
              imageProcCompl = false;
                if(acidFound || miscFound || flamableFound){ // an image has been found
                  currentTime = ros::Time::now();
                  //while time has not elapsed from current time
                  ROS_INFO("Target has been found at waypoint x: %f, y: %f, z: %f  hovering for 10 seconds for AQS to sample", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                  while(ros::Time::now() - currentTime < ros::Duration(10.0)) // hover for 10 seconds at target for AQS
                  {
                    pose.header.stamp = ros::Time::now();
                    local_pos_pub.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                 }
               }else{
                 ROS_INFO("No target found at waypoint x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                 pose.pose = wayPoints[waypointNumCompleted].pose; // set pose to next waypoint
                 waypointNumCompleted++; // increment waypoint counter
                 takeImg = true;
               }
             }

           }
         }
      }



		    pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
