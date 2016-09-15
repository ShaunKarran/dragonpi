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



float wayPointTolerance = 0.2; // 20 cm
float orientationTolerance = 0.1; // ~6 degrees


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//cb for getting UAV position from vicon Bridge
geometry_msgs::TransformStamped current_pos_t;
void pos_t_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    current_pos_t = *msg;
}

bool imageProcCompl;
bool isFound;
void imageProcCompl_cb(const std_msgs::Bool::ConstPtr& msg){
    isFound = msg->data;
    imageProcCompl = true;
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
	float xDif = abs(goalPose.pose.position.x - currentPose.transform.translation.x);
	float yDif = abs(goalPose.pose.position.y - currentPose.transform.translation.y);
	float zDif = abs(goalPose.pose.position.z - currentPose.transform.translation.z);

	//convert current orientation to euler
	geometry_msgs::Vector3 currentPoseEuler = toEuler(currentPose.transform.rotation);			
	float zOriDif = abs(goalPose.pose.orientation.z - currentPoseEuler.z);
 
	if(xDif <= wayPointTolerance && yDif <= wayPointTolerance && zDif <= wayPointTolerance && zOriDif <= orientationTolerance){
		goalReached = true;
	}		
	
	return goalReached;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;


    ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>
            ("vicon/dragonpi/dragonpi", 10, pos_t_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber image_proc_complete = nh.subscribe<std_msgs::Bool>
            ("/processing_complete",10, imageProcCompl_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher take_image_pub = nh.advertise<std_msgs::Empty>
	    ("/take_image", 10); 
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    bool navComplete = false;

    // construct vector for way points
    std::vector<geometry_msgs::PoseStamped> wayPoints;

    // construct 4 abritary way points for testing
    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = -2;
    pose1.pose.position.y = -2;
    pose1.pose.position.z = 1;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = -2;
    pose2.pose.position.y = 2;
    pose2.pose.position.z = 1;

    geometry_msgs::Vector3 v;
    v.x = 0;
    v.y = 0;
    v.z = 1.57; // pi/2 in rad

    geometry_msgs::Quaternion q;
    q = toQuaternion(v);
 

    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 2;
    pose3.pose.position.y = 2;
    pose3.pose.position.z = 1;
    pose3.pose.orientation.z = q.z;

    geometry_msgs::PoseStamped pose4;
    pose4.pose.position.x = 2;
    pose4.pose.position.y = -2;
    pose4.pose.position.z = 1;
    pose4.pose.orientation.z = q.z;
	
    // add to our waypoints to vector
    wayPoints.push_back (pose1);
    wayPoints.push_back (pose2);
    wayPoints.push_back (pose3);
    wayPoints.push_back (pose4);

	
    geometry_msgs::PoseStamped poseLand;
    poseLand.pose.position.x = 0;
    poseLand.pose.position.y = 0;
    poseLand.pose.position.z = 0;

    geometry_msgs::PoseStamped prepareLand;
    prepareLand.pose.position.x = 0;
    prepareLand.pose.position.y = 0;
    prepareLand.pose.position.z = 1;
    



    /* TODO replace arbitary waypoints */
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	// this is our takeoff pose
    	geometry_msgs::PoseStamped pose;
    	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = 1;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();

    int waypointNum = 0;
    int numberOfWaypoints = wayPoints.size();

    std_msgs::Empty takeImage;
    bool takeImg = true;
    
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

	if(!navComplete){
		if(goalPositionReached(pose, current_pos_t)){ // has UAV reached current waypoint
			if(waypointNum==numberOfWaypoints){ // this is just the number of waypoints im passing
				
				ROS_INFO("NAV Complete, preparing for landing");
				pose = prepareLand;
				waypointNum++;
			}else if(waypointNum==numberOfWaypoints+1) {
				navComplete = true;
				ROS_INFO("Landing");
				pose = poseLand;
			}else{
				ROS_INFO("Way point reached");
				if(takeImg){
					take_image_pub.publish(takeImage); // take image
					takeImg = false; // avoid publishing every iteration
				}
		
				
				if(imageProcCompl){
                                        imageProcCompl = false;
					if(isFound){
						//do something
					}else{
                                		pose = wayPoints[waypointNum]; // set pose to next waypoint
						waypointNum++; // increment waypoint counter
						takeImg = true;
					}
				}

			}
		}
	}


	

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



