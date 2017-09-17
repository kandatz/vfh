/* ========================================================================
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 *(at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ======================================================================== */
#ifndef VFH_NODE_H_
#define VFH_NODE_H_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "yuiwong/vfhplus.hpp"
namespace yuiwong {
struct VFH_node {
	VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private);
	~VFH_node();
	void update(double const desiredAngle);
private:
	Vfh *m_vfh;
	double m_cell_size;			// 100 mm
	int m_window_diameter;		// cells
	int m_sector_angle;			// in deg
	double m_safety_dist_0ms;
	double m_safety_dist_1ms;
	int m_max_speed;
	int m_max_speed_narrow_opening;
	int m_max_speed_wide_opening;
	int m_max_acceleration;
	int m_min_turnrate;
	int m_max_turnrate_0ms;
	int m_max_turnrate_1ms;
	double m_min_turn_radius_safety_factor;
	double m_free_space_cutoff_0ms;
	double m_obs_cutoff_0ms;
	double m_free_space_cutoff_1ms;
	double m_obs_cutoff_1ms;
	double m_weight_desired_dir;
	double m_weight_current_dir;
	double robot_radius;
	double m_robotVel;
	std::array<double, 361> laserRanges;
	// ros
 ros::NodeHandle nh_;
 ros::NodeHandle nh_private_;
 ros::Subscriber scan_subscriber_;
 ros::Subscriber odom_subscriber_;
 ros::Publisher vel_publisher_;
	void scanCallback(sensor_msgs::LaserScanConstPtr const& scan);
	void odomCallback(nav_msgs::OdometryConstPtr const& odom);
	struct {
		double angle;/* ::atan2(angularzVelocity, linearxVelocity) */
		double stamp;
	} desiredVelocity;
};
}
#endif /* VFH_NODE_H_ */
