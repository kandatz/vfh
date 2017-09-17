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
#include "yuiwong/rosvfhplus.hpp"
#define DEG2RAD(a) ((a) * M_PI / 180.0)
namespace yuiwong
{
VFH_node::VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private):
nh_(nh), nh_private_(nh_private)
{
	ROS_INFO("Starting VFH");
	m_cell_size = 100;							// mm, cell dimension
	m_window_diameter = 60;						// number of cells
	m_sector_angle = 5;							// deg, sector angle
	if(!nh_private_.getParam("m_safety_dist_0ms", m_safety_dist_0ms))
		m_safety_dist_0ms = 100; 				// mm, double, safe distance at 0 m/s
	if(!nh_private_.getParam("m_safety_dist_1ms", m_safety_dist_1ms))
		m_safety_dist_1ms = 100; 				// mm, double, safe distance at 1 m/s
	if(!nh_private_.getParam("m_max_speed", m_max_speed))
		m_max_speed = 200;						// mm/sec, int, max speed
	if(!nh_private_.getParam("m_max_speed_narrow_opening", m_max_speed_narrow_opening))
		m_max_speed_narrow_opening = 200; 		// mm/sec, int, max speed in the narrow opening
	if(!nh_private_.getParam("m_max_speed_wide_opening", m_max_speed_wide_opening))
		m_max_speed_wide_opening = 300; 			// mm/sec, int, max speed in the wide opening
	if(!nh_private_.getParam("m_max_acceleration", m_max_acceleration))
		m_max_acceleration = 200; 			// mm/sec^2, int, max acceleration
	if(!nh_private_.getParam("m_min_turnrate", m_min_turnrate))
		m_min_turnrate = 40;	 				// deg/sec, int, min turn rate <--- not used
	if(!nh_private_.getParam("m_max_turnrate_0ms", m_max_turnrate_0ms))
		m_max_turnrate_0ms = 40;				// deg/sec, int, max turn rate at 0 m/s
	if(!nh_private_.getParam("m_max_turnrate_1ms", m_max_turnrate_1ms))
		m_max_turnrate_1ms = 40;				// deg/sec, int, max turn rate at 1 m/s
	m_min_turn_radius_safety_factor = 1.0; 		// double ????
	if(!nh_private_.getParam("m_free_space_cutoff_0ms", m_free_space_cutoff_0ms))
		m_free_space_cutoff_0ms = 2000000.0; 	//double, low threshold free space at 0 m/s
	if(!nh_private_.getParam("m_obs_cutoff_0ms", m_obs_cutoff_0ms))
		m_obs_cutoff_0ms = 4000000.0;			//double, high threshold obstacle at 0 m/s
	if(!nh_private_.getParam("m_free_space_cutoff_1ms", m_free_space_cutoff_1ms))
		m_free_space_cutoff_1ms = 2000000.0; 	//double, low threshold free space at 1 m/s
	if(!nh_private_.getParam("m_obs_cutoff_1ms", m_obs_cutoff_1ms))
		m_obs_cutoff_1ms = 4000000.0;			//double, high threshold obstacle at 1 m/s
	if(!nh_private_.getParam("m_weight_desired_dir", m_weight_desired_dir))
		m_weight_desired_dir = 5.0;				//double, weight desired direction
	if(!nh_private_.getParam("m_weight_current_dir", m_weight_current_dir))
		m_weight_current_dir = 1.0;				//double, weight current direction
	if(!nh_private_.getParam("m_robot_radius", m_robot_radius))
		m_robot_radius = 300.0;					// robot radius in mm
	m_vfh = new VFH_Algorithm(m_cell_size, m_window_diameter, m_sector_angle,
			m_safety_dist_0ms, m_safety_dist_1ms, m_max_speed,
			m_max_speed_narrow_opening, m_max_speed_wide_opening,
			m_max_acceleration, m_min_turnrate, m_max_turnrate_0ms,
			m_max_turnrate_1ms, m_min_turn_radius_safety_factor,
			m_free_space_cutoff_0ms, m_obs_cutoff_0ms, m_free_space_cutoff_1ms,
			m_obs_cutoff_1ms, m_weight_desired_dir, m_weight_current_dir);
	m_vfh->SetRobotRadius(m_robot_radius);
	m_vfh->Init();
	this->desiredVelocity.angle = 0;
	this->desiredVelocity.stamp = 0;
	// subscribe to topics
	std::string scanTopic("");
	this->nh_private_.param<std::string>("scan_topic", scanTopic, "/scan");
	if(scanTopic.length() <= 0) {
		throw std::logic_error("scan topic is empty");
	}
	scan_subscriber_ = this->nh_.subscribe(
		scanTopic, 1, &VFH_node::scanCallback, this);
	std::string odomTopic("");
	this->nh_private_.param<std::string>("odom_topic", odomTopic, "/odom");
	if(odomTopic.length() <= 0) {
		throw std::logic_error("odom topic is empty");
	}
	odom_subscriber_ = this->nh_.subscribe(
		odomTopic, 1, &VFH_node::odomCallback, this);
	// cmd_vel publisher
	vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",5);
}
VFH_node::~VFH_node()
{
	// stop the robot
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	vel_publisher_.publish(cmd_vel);
	delete m_vfh;
}
void VFH_node::odomCallback(nav_msgs::OdometryConstPtr const& odom)
{
	m_robotVel = odom->twist.twist.linear.x * 1000.0;
	double const yaw = ::atan2(
		odom->twist.twist.angular.z, odom->twist.twist.linear.x);
	ROS_INFO_STREAM("odomCallback " << yaw);
	if (!std::isnan(yaw)) {
		//this->desiredVelocity.angle = yaw;
		this->desiredVelocity.angle = 0;
		if (!odom->header.stamp.isSimTime()) {
			this->desiredVelocity.stamp = odom->header.stamp.toSec();
		} else {
			this->desiredVelocity.stamp = ros::Time::now().toSec();
		}
	}
}
void VFH_node::scanCallback(sensor_msgs::LaserScanConstPtr const& scan)
{
	ROS_DEBUG("scanCallbac ranges %zu",scan->ranges.size());
	double const goalTolerance = 0.2;
	double desiredAngle;
	if ((ros::Time::now().toSec() - this->desiredVelocity.stamp)
		> goalTolerance) {
		ROS_INFO_STREAM(__LINE__ << " scanCallback: no desiredVelocity");
		return;
	}
	desiredAngle = this->desiredVelocity.angle;
	VFH_Algorithm::convertScan(
		scan->ranges,
		scan->angle_min,
		scan->angle_max,
		scan->angle_increment,
		scan->range_max,
		this->laserRanges);
	this->update(desiredAngle);/* perform vfh+ */
}
void VFH_node::update(double const desiredAngle)
{
	double const desiredDist = 100000.0;
	double const currGoalDistanceTolerance = 250;
	m_vfh->update(
		this->laserRanges,
		(int)(m_robotVel),
		(desiredAngle / M_PI * 180.0) + 90.0,
		desiredDist, currGoalDistanceTolerance,
		chosen_speed,
		chosen_turnrate);
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
	vel->linear.x = chosen_speed * 1e-3;
	vel->angular.z = DEG2RAD(chosen_turnrate);
	vel_publisher_.publish(vel);
	ROS_INFO(
		"angular %lf -> linear x %lf, angular z %lf",
		desiredAngle,
		vel->linear.x,
		vel->angular.z);
}
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "VFH");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	yuiwong::VFH_node vfh_node(nh,nh_private);
	ros::spin();
	return 0;
}