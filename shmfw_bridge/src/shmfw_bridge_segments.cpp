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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <shmfw/variable.h>
#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/route_segment.h>
#include <shmfw/serialization/deque.h>

Segments::Segments()
    : frequency_ ( 1.0 )
    , shm_varible_postfix_ ( "segments_ahead" )
    , angle_resolution_ ( M_PI/16 ) {

}


void Segments::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    n_param.getParam ( "shm_varible_postfix", shm_varible_postfix_ );
    ROS_INFO ( "%s/shm_varible_postfix: %s", n_param.getNamespace().c_str(), shm_varible_postfix_.c_str() );

    sprintf ( shm_varible_name_, "agv%03d_%s", agv_info.id, shm_varible_postfix_.c_str() );
    ROS_INFO ( "%s/shm_varible_name: %s", n_param.getNamespace().c_str(), shm_varible_name_ );

    n_param.getParam ( "angle_resolution", angle_resolution_ );
    ROS_INFO ( "%s/angle_resolution: %5.2f", n_param.getNamespace().c_str(), angle_resolution_ );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    segments_ahead_ = boost::shared_ptr<ShmFw::Deque<ShmFw::RouteSegment> > ( new ShmFw::Deque<ShmFw::RouteSegment> ( shm_varible_name_, shm_handler ) );

    pub_path_ = n.advertise<nav_msgs::Path> ( shm_varible_postfix_, 1 );
    pub_waypoints_ = n.advertise<geometry_msgs::PoseArray> ( "waypoints", 1 );
    pub_marker_ = n.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
    thread_ = boost::thread ( boost::bind ( &Segments::update, this ) );

    initMarker();

}
void Segments::initMarker() {

    marker_lines_  = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );
    marker_text_ = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );
    marker_end_  = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );

    marker_lines_->header.frame_id = marker_text_->header.frame_id = marker_end_->header.frame_id = "map";
    marker_lines_->header.stamp = marker_text_->header.stamp = marker_end_->header.stamp = ros::Time::now();
    marker_lines_->ns = "segments_as_line";
    marker_text_->ns =  "segments_id";
    marker_end_->ns = "segment_end_points";
    marker_lines_->action = marker_text_->action = marker_end_->action = visualization_msgs::Marker::ADD;
    marker_lines_->pose.orientation.w = marker_text_->pose.orientation.w = marker_end_->pose.orientation.w = 1.0;
    marker_lines_->id = 0;
    marker_text_->id = 1;
    marker_end_->id = 2;
    marker_lines_->type = visualization_msgs::Marker::LINE_LIST;
    marker_text_->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_end_->type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    marker_lines_->scale.x = 0.01;
    marker_lines_->scale.y = 0.02;
    marker_lines_->scale.z = marker_lines_->scale.y*2;
    marker_text_->scale.x = marker_text_->scale.y = marker_text_->scale.z = 0.2;
    marker_end_->scale.x = marker_end_->scale.y = 0.1;

    // start points are green
    marker_lines_->color.g = 1.0f;
    marker_lines_->color.a = 1.0;

    // center points are blue
    marker_text_->color.b = 1.0;
    marker_text_->color.a = 1.0;

    // end points are red
    marker_end_->color.r = 1.0;
    marker_end_->color.a = 1.0;

}
void Segments::drawMarker ( const std::vector<ShmFw::RouteSegment> &segments ) {
    marker_lines_->points.clear();
    marker_text_->points.clear();
    marker_end_->points.clear();
    char text[0xFF];
    for ( size_t i = 0; i < segments.size(); i++ ) {
        const ShmFw::RouteSegment &s = segments[i];
        geometry_msgs::Point p;
        p.x = s.start.position.x, p.y = s.start.position.y, p.z = s.start.position.z;
        //marker_text_->points.clear();
        marker_text_->pose.position.x = p.x;
        marker_text_->pose.position.y = p.y;
        marker_text_->pose.position.z = p.z;
        marker_lines_->points.push_back ( p );
        p.x = s.end.position.x, p.y = s.end.position.y, p.z = s.end.position.z;
        marker_end_->points.push_back ( p );
        marker_lines_->points.push_back ( p );
        sprintf ( text, "%i", s.id );
        marker_text_->id = s.id;
        marker_text_->text = text;
        pub_marker_.publish ( marker_text_ );
    }
    pub_marker_.publish ( marker_end_ );
    pub_marker_.publish ( marker_lines_ );
}
std::vector<ShmFw::Pose> &Segments::convertRoute2Path ( const std::vector<ShmFw::RouteSegment> &segments, std::vector<ShmFw::Pose> &path, double angle_resolution ) {


    for ( size_t i = 0; i < segments.size(); i++ ) {
        const ShmFw::RouteSegment &s = segments[i];

        if ( s.type == ShmFw::RouteSegment::TYPE_LINE ) {

            ShmFw::Point2D p0 ( s.start.position.x, s.start.position.y );
            ShmFw::Point2D p1 ( s.end.position.x, s.end.position.y );
            ShmFw::Pose2D pose2d ( p0, p1 );
            ShmFw::Pose pose3d ( pose2d );
            path.push_back ( pose3d );
        }
        if ( s.type == ShmFw::RouteSegment::TYPE_ARC ) {


            double dx0 = s.start.position.x - s.center.position.x;
            double dy0 = s.start.position.y - s.center.position.y;
            double a0 = atan2 ( dy0, dx0 );
            double dx1 = s.end.position.x - s.center.position.x;
            double dy1 = s.end.position.y - s.center.position.y;
            double a1 = atan2 ( dy1, dx1 );

            double r0 = sqrt ( dx0*dx0 + dy0*dy0 );

            ShmFw::Point2D p1 ( s.start.position.x, s.start.position.y ), p0;
            ShmFw::Point2D pc ( s.center.position.x, s.center.position.y );
            for ( double a = a0+angle_resolution; a < a1; a += angle_resolution ) {
                p0 = p1;
                p1 = ShmFw::Point2D ( pc.x + cos ( a ) * r0, pc.y + sin ( a ) * r0 );
                ShmFw::Pose2D pose2d ( p0, p1 );
                ShmFw::Pose pose3d ( pose2d );
                path.push_back ( pose3d );
            }
        }
        if ( i == segments.size()-1 ) {
            ShmFw::Point2D p ( s.end.position.x, s.end.position.y );
            ShmFw::Pose pose ( ShmFw::Point ( p.x, p.y, 0 ), ShmFw::Quaternion() );
            path.push_back ( pose );
        }
    }
    return path;
}
void Segments::update() {
    ros::Rate rate ( frequency_ );
    int timeout = 1000.0/frequency_;
    int timeout_count = 0;
    std::vector<ShmFw::RouteSegment> segments;
    std::vector<ShmFw::Pose> poses;
    nav_msgs::Path path;
    while ( ros::ok() ) {
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
            convertRoute2Path ( segments, poses, angle_resolution_ );
            path.header.frame_id = "map";
            path.header.stamp = ros::Time::now();
            path.poses.resize ( poses.size() );
            for ( size_t i = 0; i < poses.size(); i++ ) {
                ShmFw::Pose pose = poses[i];
                path.poses[i].header = path.header;
                path.poses[i].header.seq = i;
                pose.copyTo ( path.poses[i].pose );
            }
            pub_path_.publish ( path );

            if ( pub_marker_.getNumSubscribers() > 0 ) {
                drawMarker ( segments );
            }
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }
        rate.sleep();
    }
}

