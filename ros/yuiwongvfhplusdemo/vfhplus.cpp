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
#include "yuiwong/debug.hpp"
#include "yuiwong/angle.hpp"
#include "yuiwong/vfh.hpp"
#include "yuiwong/rosvfhplus.hpp"
namespace yuiwong
{
static bool debug = false;
VfhPlusNode::VfhPlusNode(ros::NodeHandle nh, ros::NodeHandle pnh):
nh(nh), pnh(pnh)
{
	ROS_INFO("starting vfh");
	this->pnh.param("debug", debug, false);
	if (debug) {
		yuiwong::kDebugLevel = yuiwong::LogLevel::Deta;
	}
	VfhPlus::Param p;
#	if 0
	p.cellWidth = 0.1;/* cell dimension */
	p.windowDiameter = 60;/* cells count */
	p.sectorAngle = DegreeToRadian(5);/* sector angle */
	this->pnh.getParam("zero_safety_distance", p.zeroSafetyDistance);
	this->pnh.getParam("max_safety_distance", p.maxSafetyDistance);
	this->pnh.param("max_speed", p.maxSpeed, 0.6);
	this->pnh.getParam("max_speed_narrow_opening", p.maxSpeedNarrowOpening);
	if (!this->pnh.getParam(
		"max_speed_wide_opening", p.maxSpeedWideOpening)) {
		p.maxSpeedWideOpening = p.maxSpeed;
	}
	this->pnh.getParam("max_acceleration", p.maxAcceleration);
	this->pnh.getParam("zero_max_turnrate", p.zeroMaxTurnrate);
	this->pnh.getParam("max_max_turnrate", p.maxMaxTurnrate);
	this->pnh.getParam(
		"min_turn_radius_safety_factor", p.minTurnRadiusSafetyFactor);
	this->pnh.getParam("zero_free_space_cutoff", p.zeroFreeSpaceCutoff);
	this->pnh.getParam("max_free_space_cutoff", p.maxFreeSpaceCutoff);
	this->pnh.getParam("zero_obs_cutoff", p.zeroObsCutoff);
	this->pnh.getParam("max_obs_cutoff", p.maxObsCutoff);
	this->pnh.getParam("desired_direction_weight", p.desiredDirectionWeight);
	this->pnh.getParam("current_direction_weight", p.currentDirectionWeight);
#	endif
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
	double robotRadius;
	if (!this->pnh.getParam("robot_radius", robotRadius)) {
		robotRadius = 0.3;
	}
	this->vfh = boost::make_shared<VfhPlus>(p);
	this->vfh->setRobotRadius(robotRadius);
	this->vfh->init();
	this->goal.goal = nullptr;
	this->goal.stamp = 0;
	this->goal.currentLinearX = 0;
	this->goal.currentLinearXStamp = 0;
	/* cmd_vel publisher */
	std::string t("");
	this->pnh.param<std::string>("topic", t, "/cmd_vel");
	this->velPublisher = this->nh.advertise<geometry_msgs::Twist>(
		t, sizeof(size_t), false);
	/* subscribe to topics */
	std::string scanTopic("");
	this->pnh.param<std::string>("scan_topic", scanTopic, "/scan");
	if(scanTopic.length() <= 0) {
		throw std::logic_error("scan topic is empty");
	}
	scanSubscriber = this->nh.subscribe(
		scanTopic, 1, &VfhPlusNode::scanCallback, this);
	std::string odomTopic("");
	std::string goalTopic("");
	this->pnh.param<std::string>("goal_topic", goalTopic, "/goal");
	if (goalTopic.length() <= 0) {
		throw std::logic_error("goal topic is empty");
	}
	this->goalSubscriber = this->nh.subscribe(
		goalTopic, 1, &VfhPlusNode::goalCallback, this);
	this->pnh.param<std::string>("odom_topic", odomTopic, "");
	if (odomTopic.length() > 0) {
		this->odomSubscriber = this->nh.subscribe(
			odomTopic, 1, &VfhPlusNode::odomCallback, this);
	} else {
		ROS_WARN("no odom");
	}
	ROS_INFO("VfhPlusNode started");
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
	if (!odom->header.stamp.isSimTime()) {
		boost::mutex::scoped_lock lock(this->goal.mutex);
		(void)(lock);
		this->goal.currentLinearXStamp = odom->header.stamp.toSec();
	} else {
		boost::mutex::scoped_lock lock(this->goal.mutex);
		(void)(lock);
		this->goal.currentLinearXStamp = ros::Time::now().toSec();
	}
}
void VfhPlusNode::goalCallback(
	yuiwonggeometrymsg::AngleDistanceConstPtr const& goal)
{
	boost::mutex::scoped_lock lock(this->goal.mutex);
	(void)(lock);
	this->goal.goal = goal;
	this->goal.stamp = ros::Time::now().toSec();
}
void VfhPlusNode::scanCallback(sensor_msgs::LaserScanConstPtr const& scan)
{
	yuiwonggeometrymsg::AngleDistanceConstPtr goal;
	double cv;
	double const now = ros::Time::now().toSec();
	{
	boost::mutex::scoped_lock lock(this->goal.mutex);
	(void)(lock);
	if (DoubleCompare(now - this->goal.stamp, 5.0) <= 0) {
		goal = this->goal.goal;
		if (DoubleCompare(
			now - this->goal.currentLinearXStamp,
			VfhPlus::defaultStampTolerance) <= 0) {
			cv = this->goal.currentLinearX;
		} else {
			cv = 0;
		}
	} else {
		goal = nullptr;
		cv = 0;
	}
	}
	if (!goal) {
		static double lastStopped = 0;
		if (DoubleCompare(now - lastStopped, 0.5) > 0) {
			ConvertScan(
				scan->ranges,
				scan->angle_min,
				scan->angle_max,
				scan->angle_increment,
				this->laserRanges);
			this->update(cv, 0, 0);/* perform vfh+ */
			lastStopped = now;
		}
		return;
	}
	ConvertScan(
		scan->ranges,
		scan->angle_min,
		scan->angle_max,
		scan->angle_increment,
		this->laserRanges);
	this->laserRanges *= 1e3;
	if (false && debug) {
		auto const& l = this->laserRanges.transpose();
		for (int i = 0; i < 361; ++i) {
			std::cout << ToString(l[i]) << "\t";
			if (!((i + 1) % 10)) {
				std::cout << "\n";
			}
		}
		std::cout << "\n\n";
		Eigen::Matrix<double, 361, 1> laserRanges;
		VfhPlus::convertScanMM(
			scan->ranges,
			scan->angle_min,
			scan->angle_max,
			scan->angle_increment,
			laserRanges);
		//std::cout << laserRanges.transpose() << "!!!\n";
		auto const& l2 = laserRanges.transpose();
		for (int i = 0; i < 361; ++i) {
			std::cout << ToString(l2[i]) << "\t";
			if (!((i + 1) % 10)) {
				std::cout << "\n";
			}
		}
		std::cout << "\n";
		if (laserRanges != this->laserRanges) {
			exit(1);
		}
	}
	this->update(cv, goal->angle, goal->distance);/* perform vfh+ */
}
void VfhPlusNode::update(double const v, double const a, double const d)
{
	double chosenLinearX, chosenAngularZ;
	this->vfh->update(
		this->laserRanges,
		v,
		a,
		d,
		VfhPlus::defaultTolerance,
		chosenLinearX,
		chosenAngularZ);
	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());
	vel->linear.x = chosenLinearX;
	vel->angular.z = chosenAngularZ;
	this->velPublisher.publish(vel);
	ROS_INFO(
		"goal %lf %lf v %lf: linear x %lf angular z %lf",
		a,
		d,
		v,
		vel->linear.x,
		vel->angular.z);
}
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vfhplus");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	yuiwong::VfhPlusNode vfh(nh, pnh);
	ros::spin();
	return 0;
}
