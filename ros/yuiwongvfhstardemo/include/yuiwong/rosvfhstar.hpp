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
#ifndef YUIWONGVFHPLUSDEMO_ROSVFPSTAR_HPP
#define YUIWONGVFHPLUSDEMO_ROSVFPSTAR_HPP
#include "Eigen/Eigen"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "yuiwong/vfhstar.hpp"
#include "yuiwonggeometrymsg/AngleDistance.h"
namespace yuiwong {
struct VfhStarNode {
	VfhStarNode(ros::NodeHandle nh, ros::NodeHandle pnh);
	~VfhStarNode();
	void update(double const v, double const a, double const d);
private:
	boost::shared_ptr<VfhStar> vfh;
	Eigen::Matrix<double, 361, 1> laserRanges;
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
	ros::Subscriber scanSubscriber;
	ros::Subscriber odomSubscriber;
	ros::Subscriber goalSubscriber;
	ros::Publisher velPublisher;
	void scanCallback(sensor_msgs::LaserScanConstPtr const& scan);
	void odomCallback(nav_msgs::OdometryConstPtr const& odom);
	void goalCallback(yuiwonggeometrymsg::AngleDistanceConstPtr const& goal);
	struct {
		yuiwonggeometrymsg::AngleDistanceConstPtr goal;
		double stamp;
		double currentLinearX;
		double currentLinearXStamp;
		boost::mutex mutex;
	} goal;
};
}
#endif
