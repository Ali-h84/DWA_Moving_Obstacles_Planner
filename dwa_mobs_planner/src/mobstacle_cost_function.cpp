/*
 * mobstacle_cost_function.cpp
 * ROS wrapper for DWAMobsPlanner, developed at RAMP, SFU
 *
 * Modified obstacle cost function from base_local_planner for DWA Mobs Planner
 */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2014 Robotic Algorithms & Motion Planning Laboratory
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse, Alireza Hekmati
 *********************************************************************/

#include "ros/ros.h"
#include <cmath>
#include <string>
#include <Eigen/Core>
#include <ros/console.h>
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "dwa_mobs_planner/mobstacle_cost_function.h"
#include "dwa_mobs_planner/OdometryMovingObstacles.h"
#include "dwa_mobs_planner/dwa_mobs_planner_ros.h"


namespace base_local_planner{

MObstacleCostFunction::MObstacleCostFunction() : sum_scores_( false ), oo_i_t( "/OdometryMovingObstacles" ){
    // obstacle odom subscriber
    sub_oo = mcf_nh.subscribe<dwa_mobs_planner::OdometryMovingObstacles>(
        oo_i_t, 10, boost::bind( &MObstacleCostFunction::mcf_cb, this, _1 )
    );
    ROS_INFO_STREAM( "DWA mobs planner node: " );
    ROS_INFO_STREAM( " Subscribing to " << oo_i_t );
}

MObstacleCostFunction::~MObstacleCostFunction() {
}

void MObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void MObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool MObstacleCostFunction::prepare() {
  return true;
}

// this is where the trajectory is examined
double MObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
    double cost = 0;
    double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
    double px, py, pth;
    double ox, oy;

    if( footprint_spec_.size() == 0 ){
        // Bug, should never happen
        ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
        return -9;
    }
    if( !mobs_ready ){ return 0; }

    double td = traj.time_delta_;     //which is sim_time_ / num_steps
    double ps = traj.getPointsSize();
    if( ps < 1 ){ return 0; }
    // threshold to detect collision
    double thres = 0.6;

    dwa_mobs_planner::OdometryMovingObstacles mobs_w;
    mobs_w.number_of_obstacles = 0;
    if( mobs_c.number_of_obstacles < 1 ){ return 0; } // nothing to look at
    mobs_w = mobs_c; // cache the odoms
    if( traj.getPointsSize() > 0 && mobs_w.number_of_obstacles > 0 ){
        for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
        // for all points in the trajectory
            traj.getPoint(i, px, py, pth);
            // get the point
            for(unsigned int j = 0; j < mobs_w.number_of_obstacles; j++){
            // compare with the moving obstacles
                double tt = double(i) * td;
                ox  = mobs_w.odom[j].pose.pose.position.x;
                oy  = mobs_w.odom[j].pose.pose.position.y;
                // project the object in time
                ox += mobs_w.odom[j].twist.twist.linear.x * tt;
                oy += mobs_w.odom[j].twist.twist.linear.y * tt;
                // distance to compare
                double dis = sqrt(pow(px-ox,2) + pow(py-oy,2));
                if( dis < thres ) { // if detected
                  return -5.0; // trajectory will be dropped
                }
            }
        }
        ROS_DEBUG_STREAM("Trajectory passed DWD");
        return 0;
    }
    return 0;
}

double MObstacleCostFunction::getScalingFactor( Trajectory &traj, double scaling_speed,
                                            double max_trans_vel, double max_scaling_factor ){
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

// call back for odom msg
void MObstacleCostFunction::mcf_cb( const dwa_mobs_planner::OdometryMovingObstaclesConstPtr& mobs_i){
    if( mobs_i != NULL ){
        mobs_c.number_of_obstacles = 0;
        mobs_c = *mobs_i; // caching
        mobs_ready = true; // needed to avoid seg fault
        if( mobs_c.header.frame_id == "velodyne"){
            ROS_WARN("You're using velodyne's frame.  This may cause issues.");
            // check if transform exists and obstacles are present
            if( tf_li_oo.canTransform( "/velodyne", "/odom", ros::Time(0) ) && mobs_c.number_of_obstacles > 0 ){
                for(unsigned int j = 0; j < mobs_c.number_of_obstacles; j++){
                    // transform position
                    tf::Point pos;
                    pos.setX(mobs_c.odom[j].pose.pose.position.x);
                    pos.setY(mobs_c.odom[j].pose.pose.position.y);
                    pos.setZ(mobs_c.odom[j].pose.pose.position.z);
                    tf::Stamped<tf::Point> oo_tf_pos_i( pos, ros::Time(0), "/velodyne" );
                    tf::Stamped<tf::Point> oo_tf_pos_o;
                    tf_li_oo.transformPoint( "/odom", oo_tf_pos_i, oo_tf_pos_o );
                    mobs_c.odom[j].pose.pose.position.x = oo_tf_pos_o.getX();
                    mobs_c.odom[j].pose.pose.position.y = oo_tf_pos_o.getY();
                    mobs_c.odom[j].pose.pose.position.z = oo_tf_pos_o.getZ();

                    // transform velocity
                    tf::Vector3 vel;
                    vel.setX(mobs_c.odom[j].twist.twist.linear.x);
                    vel.setY(mobs_c.odom[j].twist.twist.linear.y);
                    vel.setZ(mobs_c.odom[j].twist.twist.linear.z);
                    tf::Stamped<tf::Point> oo_tf_vel_i( vel, ros::Time(0), "/velodyne" );
                    tf::Stamped<tf::Point> oo_tf_vel_o;
                    tf_li_oo.transformPoint( "/odom", oo_tf_vel_i, oo_tf_vel_o );
                    mobs_c.odom[j].twist.twist.linear.x = oo_tf_vel_o.getX();
                    mobs_c.odom[j].twist.twist.linear.y = oo_tf_vel_o.getY();
                    mobs_c.odom[j].twist.twist.linear.z = oo_tf_vel_o.getZ();
                }
            }
        }
        else if( mobs_c.header.frame_id != "odom" ){
            std::string s_test = "just in case";
            ROS_ERROR("Your frame ID does not make sense.");
            ROS_ERROR_STREAM("Frame = " << mobs_c.header.frame_id);
        }
    }
}

} /* namespace base_local_planner */
