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
#include "yuiwong/rosvfhstar.hpp"
namespace yuiwong
{
VfhPlusNode::VfhPlusNode(ros::NodeHandle nh, ros::NodeHandle pnh):
nh(nh), pnh(pnh)
{
	ROS_INFO("Starting VFH");
	VfhPlus::Param p;
	p.cell_size = 100;// mm, cell dimension
	p.window_diameter = 60;// number of cells
	p.sector_angle = 5;// deg, sector angle
	if(!this->pnh.getParam("safety_dist_0ms", p.safety_dist_0ms))
		p.safety_dist_0ms = 100;// mm, double, safe distance at 0 m/s
	if(!this->pnh.getParam("safety_dist_1ms", p.safety_dist_1ms))
		p.safety_dist_1ms = 100;// mm, double, safe distance at 1 m/s
	if(!this->pnh.getParam("max_speed", p.max_speed))
		p.max_speed = 200;// mm/sec, int, max speed
	if(!this->pnh.getParam("max_speed_narrow_opening", p.max_speed_narrow_opening))
		p.max_speed_narrow_opening = 200;// mm/sec, int, max speed in the narrow opening
	if(!this->pnh.getParam("max_speed_wide_opening", p.max_speed_wide_opening))
		p.max_speed_wide_opening = 300;// mm/sec, int, max speed in the wide opening
	if(!this->pnh.getParam("max_acceleration", p.max_acceleration))
		p.max_acceleration = 200;// mm/sec^2, int, max acceleration
	if(!this->pnh.getParam("min_turnrate", p.min_turnrate))
		p.min_turnrate = 40;// deg/sec, int, min turn rate <--- not used
	if(!this->pnh.getParam("max_turnrate_0ms", p.max_turnrate_0ms))
		p.max_turnrate_0ms = 40;// deg/sec, int, max turn rate at 0 m/s
	if(!this->pnh.getParam("max_turnrate_1ms", p.max_turnrate_1ms))
		p.max_turnrate_1ms = 40;// deg/sec, int, max turn rate at 1 m/s
	p.min_turn_radius_safety_factor = 1.0;// double ????
	if(!this->pnh.getParam("free_space_cutoff_0ms", p.free_space_cutoff_0ms))
		p.free_space_cutoff_0ms = 2000000.0;//double, low threshold free space at 0 m/s
	if(!this->pnh.getParam("obs_cutoff_0ms", p.obs_cutoff_0ms))
		p.obs_cutoff_0ms = 4000000.0;//double, high threshold obstacle at 0 m/s
	if(!this->pnh.getParam("free_space_cutoff_1ms", p.free_space_cutoff_1ms))
		p.free_space_cutoff_1ms = 2000000.0;//double, low threshold free space at 1 m/s
	if(!this->pnh.getParam("obs_cutoff_1ms", p.obs_cutoff_1ms))
		p.obs_cutoff_1ms = 4000000.0;//double, high threshold obstacle at 1 m/s
	if(!this->pnh.getParam("weight_desired_dir", p.weight_desired_dir))
		p.weight_desired_dir = 5.0;//double, weight desired direction
	if(!this->pnh.getParam("weight_current_dir", p.weight_current_dir))
		p.weight_current_dir = 1.0;//double, weight current direction
	double robot_radius;
	if(!this->pnh.getParam("robot_radius", robot_radius))
		robot_radius = 300.0;// robot radius in mm
	this->vfh = boost::make_shared<VfhPlus>(p);
	this->vfh->setRobotRadius(robot_radius);
	this->vfh->init();
	this->desiredVelocity.angle = 0;
	this->desiredVelocity.stamp = 0;
	// subscribe to topics
	std::string scanTopic("");
	this->pnh.param<std::string>("scan_topic", scanTopic, "/scan");
	if(scanTopic.length() <= 0) {
		throw std::logic_error("scan topic is empty");
	}
	scanSubscriber = this->nh.subscribe(
		scanTopic, 1, &VfhPlusNode::scanCallback, this);
	std::string odomTopic("");
	this->pnh.param<std::string>("odom_topic", odomTopic, "/odom");
	if(odomTopic.length() <= 0) {
		throw std::logic_error("odom topic is empty");
	}
	odomSubscriber = this->nh.subscribe(
		odomTopic, 1, &VfhPlusNode::odomCallback, this);
	// cmd_vel publisher
	std::string t("");
	this->pnh.param<std::string>("topic", t, "/cmd_vel");
	this->velPublisher = this->nh.advertise<geometry_msgs::Twist>(
		t, sizeof(size_t));
}
VfhPlusNode::~VfhPlusNode()
{
	this->scanSubscriber.shutdown();
	this->odomSubscriber.shutdown();
	/* stop the robot */
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
	vel->linear.x = 0.0;
	vel->angular.z = 0.0;
	velPublisher.publish(vel);
}
void VfhPlusNode::odomCallback(nav_msgs::OdometryConstPtr const& odom)
{
	this->robotLinearX = odom->twist.twist.linear.x;
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
void VfhPlusNode::scanCallback(sensor_msgs::LaserScanConstPtr const& scan)
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
	VfhPlus::convertScan(
		scan->ranges,
		scan->angle_min,
		scan->angle_max,
		scan->angle_increment,
		scan->range_max,
		this->laserRanges);
	this->update(desiredAngle);/* perform vfh+ */
}
void VfhPlusNode::update(double const desiredAngle)
{
	double const desiredDist = 2.0;
	double const currGoalDistanceTolerance = 0.250;
	double chosenLinearX, chosenAngularZ;
	this->vfh->update(
		this->laserRanges,
		0.3,//this->robotLinearX,
		//desiredAngle + (M_PI / 2.0),
		desiredAngle,
		desiredDist,
		currGoalDistanceTolerance,
		chosenLinearX,
		chosenAngularZ);
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
	vel->linear.x = chosenLinearX;
	vel->angular.z = chosenAngularZ;
	velPublisher.publish(vel);
	ROS_INFO(
		"angular %lf -> linear x %lf, angular z %lf",
		desiredAngle,
		vel->linear.x,
		vel->angular.z);
}
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "VFH +");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	yuiwong::VfhPlusNode vfh(nh, pnh);
	ros::spin();
	return 0;
}
