/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#include <shmfw_bridge/shmfw_bridge.h>

#include <shmfw/variable.h>
#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d_agv.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

Gazebo::Gazebo()
    : target_frame_ ( "map" )
    , shm_name_pose_gt_ ( "pose_gt" )
    , gazebo_model_name_ ( "r1" ) {

}

void Gazebo::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler, const AGVInfo &agv_info ) {

  char name[0xFF];
    n_param.getParam ( "shm_name_pose_gt", shm_name_pose_gt_ );    
    sprintf ( name, "agv%03d_%s", agv_info.id, shm_name_pose_gt_.c_str() );
    ROS_INFO ( "%s/shm_name_pose_gt: %s", n_param.getNamespace().c_str(), name );
    pose_ = boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > ( new ShmFw::Var<ShmFw::Pose2DAGV> ( name, shm_handler ) );
    
    n_param.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param.getNamespace().c_str(), target_frame_.c_str() );
    
    n_param.getParam ( "gazebo_model_name", gazebo_model_name_ );
    ROS_INFO ( "%s/gazebo_model_name: %s", n_param.getNamespace().c_str(), gazebo_model_name_.c_str() );

    msg_.header.frame_id = target_frame_;
    msg_.header.seq = 0;
    pub_ = n.advertise<geometry_msgs::PoseStamped> ( "pose_gt", 1 );
    sub_ = n.subscribe ( "/gazebo/model_states", 1, &Gazebo::callback, this );
}


void Gazebo::callback ( const gazebo_msgs::ModelStates::ConstPtr& msg ) {
    ShmFw::Pose2DAGV pose2D;
    ShmFw::Pose pose3D;
    for ( int i = 0; i < msg->name.size(); i++ ) {
        if ( msg->name[i].compare ( gazebo_model_name_ ) == 0 ) {
            ShmFw::Point p ( msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z );
            ShmFw::Quaternion o ( msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w );
            pose3D.setPose ( p,o );
	    pose3D.getPose2D(pose2D);
            ROS_INFO ( "robot: %s, %s", msg->name[i].c_str(), pose2D.getToString().c_str() );
            pose_->set ( pose2D );	    
	    msg_.header.stamp = ros::Time::now();
	    pose3D.copyTo(msg_.pose);
	    pub_.publish(msg_);
        }
    }
}
