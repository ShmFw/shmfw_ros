#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Agv/AgvBase.hpp>
#include <shmfw/variable.h>
#include <shmfw/deque.h>
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/segment_agv.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main ( int argc, char** argv ) {


    ros::init ( argc, argv, "simple_navigation_goals" );
    short enabled = false;

    dsa::AgvBase agv ( 15, false );
    agv.state ( dsa::AgvBase::agv_ready );

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac ( "r1/move_base", true );

    //wait for the action server to come up
    while ( !ac.waitForServer ( ros::Duration ( 5.0 ) ) ) {
        ROS_INFO ( "Waiting for the move_base action server to come up" );
    }

    move_base_msgs::MoveBaseGoal goal;

    while ( agv.state() != dsa::AgvBase::agv_enabled ) {
        sleep ( 1 );
        ROS_INFO ( "Waiting for agv to be enabled" );
    }

    tf::Quaternion quaternion;
    while ( agv.shmWayPointsAhead->size() > 0 ) {
        ShmFw::Pose2D p = agv.shmWayPointsAhead->front();
	agv.shmWayPointsAhead->pop_front();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = p.position.x;
        goal.target_pose.pose.position.y = p.position.y;
        goal.target_pose.pose.position.z = 0;
        quaternion.setRPY ( 0, 0, p.orientation );
        goal.target_pose.pose.orientation.x = quaternion.getX();
        goal.target_pose.pose.orientation.y = quaternion.getY();
        goal.target_pose.pose.orientation.z = quaternion.getZ();
        goal.target_pose.pose.orientation.w = quaternion.getW();

        ROS_INFO ( "Sending goal" );

        ac.sendGoal ( goal );
        ROS_INFO ( "set waypoint" );
        ac.waitForResult();
        if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
            ROS_INFO ( "reached waypoint" );

        } else {
            ROS_INFO ( "failed to reach waypoint" );
        }

    }

    return 0;
}
