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
#ifndef YUIWONGVFHPLUSDEMO_VFPPLUS_HPP
#define YUIWONGVFHPLUSDEMO_VFPPLUS_HPP
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "yuiwong/vfhstar.hpp"
namespace yuiwong {
struct VfhPlusNode {
	VfhPlusNode(ros::NodeHandle nh, ros::NodeHandle pnh);
	~VfhPlusNode();
	void update(double const desiredAngle);
private:
	boost::shared_ptr<VfhPlus> vfh;
	double robotLinearX;/* meter/s */
	std::array<double, 361> laserRanges;
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	ros::Subscriber scanSubscriber;
	ros::Subscriber odomSubscriber;
	ros::Publisher velPublisher;
	void scanCallback(sensor_msgs::LaserScanConstPtr const& scan);
	void odomCallback(nav_msgs::OdometryConstPtr const& odom);
	struct {
		double angle;/* ::atan2(angularzVelocity, linearxVelocity) */
		double stamp;
	} desiredVelocity;
};
}
#endif /* YUIWONGVFHPLUSDEMO_VFPPLUS_HPP */
