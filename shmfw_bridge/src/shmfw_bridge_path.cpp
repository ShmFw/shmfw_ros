/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <shmfw_bridge/shmfw_bridge.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

Path::Path()
    : frequency_ ( 1.0 )
    , scale_ ( 0.1 )
    , resolution_ ( 0.5 ) {

}

void Path::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    char name_segments_ahead[0xFF];
    sprintf ( name_segments_ahead, "agv%03d_segmentsAhead", agv_info.id );
    ROS_INFO ( "%s/shm_varible_name_segmentsAhead: %s", n_param.getNamespace().c_str(), name_segments_ahead );

    n_param.getParam ( "scale", scale_ );
    ROS_INFO ( "%s/scale: %5.2f", n_param.getNamespace().c_str(), scale_ );

    n_param.getParam ( "resolution", resolution_ );
    ROS_INFO ( "%s/resolution: %5.2f", n_param.getNamespace().c_str(), resolution_ );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    segments_ahead_ = boost::shared_ptr<ShmFw::Deque<ShmFw::SegmentAGV> > ( new ShmFw::Deque<ShmFw::SegmentAGV> ( name_segments_ahead, shm_handler ) );

    pub_path_ = n.advertise<nav_msgs::Path> ( "segments_ahead", 1 );
    pub_waypoints_ = n.advertise<geometry_msgs::PoseArray> ( "waypoints", 1 );
    thread_ = boost::thread ( boost::bind ( &Path::update, this ) );

}

void Path::exit() {
    loop = false;
}

void Path::update() {
    int timeout = 1000.0/frequency_;
    loop = true;
    int timeout_count = 0;
    std::vector<ShmFw::SegmentAGV> segments;
    std::vector<ShmFw::Pose> poses;
    nav_msgs::Path path;
    geometry_msgs::PoseArray waypoints;
    while ( loop ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            segments_ahead_->wait();
            read = true;
        } else {
            segments_ahead_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
            segments_ahead_->get ( segments );
            getWayPoints ( segments, poses );
            path.header.frame_id = "map";
            path.header.stamp = ros::Time::now();
            path.poses.resize ( poses.size() );
            waypoints.header = path.header;
            waypoints.poses.resize ( poses.size() );
            for ( size_t i = 0; i < poses.size(); i++ ) {
                path.poses[i].header = path.header;
                path.poses[i].pose.position.x = poses[i].position.x * scale_;
                path.poses[i].pose.position.y = poses[i].position.y * scale_;
                path.poses[i].pose.position.z = poses[i].position.z * scale_;
                path.poses[i].pose.orientation.x = poses[i].orientation.x;
                path.poses[i].pose.orientation.y = poses[i].orientation.y;
                path.poses[i].pose.orientation.z = poses[i].orientation.z;
                path.poses[i].pose.orientation.w = poses[i].orientation.w;

                waypoints.poses[i] = path.poses[i].pose;
            }
            pub_path_.publish ( path );
            pub_waypoints_.publish ( waypoints );
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }

    }
}

std::vector<ShmFw::Pose> Path::getWayPoints ( const std::vector<ShmFw::SegmentAGV> &segments, std::vector<ShmFw::Pose> &path ) {

    double resolution = ( resolution_/scale_ );
    for ( int i = 0; i < segments.size(); i++ ) {
        const ShmFw::SegmentAGV &s = segments[i];
        if ( s.type == ShmFw::SegmentAGV::TYPE_LINE ) {
            std::cout  << s << std::endl;
            ShmFw::Point2D p0 ( s.start.position.x, s.start.position.y );
            ShmFw::Point2D p1 ( s.end.position.x, s.end.position.y );
            ShmFw::Vector2<double> v ( p1.x-p0.x, p1.y - p0.y );
            double d = v.norm();
            ShmFw::Vector2<double> u = v/d;
            if ( resolution < d ) {
                double l = 0;
                ShmFw::Point2D p1 ( s.start.position.x + u.x * l, s.start.position.y + u.y * l );
                for ( double l = resolution; l < d; l+=resolution ) {
                    ShmFw::Point2D p0 ( p1 );
                    p1 = ShmFw::Point2D ( s.start.position.x + u.x * l, s.start.position.y + u.y * l );
                    path.push_back ( ShmFw::Pose(ShmFw::Pose2D(p0,p1), s.level) );
                }
            } else {
                path.push_back ( ShmFw::Pose(ShmFw::Pose2D(p0,p1), s.level) );
            }
        }
        if ( s.type == ShmFw::SegmentAGV::TYPE_ARC ) {
            std::cout  << s << std::endl;
            double dx0 = s.start.position.x - s.center.position.x;
            double dy0 = s.start.position.y - s.center.position.y;
            double a0 = atan2 ( dy0, dx0 );

            double dx1 = s.end.position.x - s.center.position.x;
            double dy1 = s.end.position.y - s.center.position.y;
            double a1 = atan2 ( dy1, dx1 );

            double r0 = sqrt ( dx0*dx0 + dy0*dy0 );
            double angle_resolution = resolution/r0;


            ShmFw::Point2D p1 ( s.center.position.x + cos ( a0 ) * r0, s.center.position.y+sin ( a0 ) * r0 );
            for ( double a = a0+angle_resolution; a <= a1; a+=angle_resolution ) {
                ShmFw::Point2D p0 ( p1 );
                p1 = ShmFw::Point2D ( s.center.position.x + cos ( a ) * r0, s.center.position.y + sin ( a ) * r0 );
                path.push_back ( ShmFw::Pose ( ShmFw::Pose2D ( p0,p1 ), s.level ) );
            }
        }
    }
}
