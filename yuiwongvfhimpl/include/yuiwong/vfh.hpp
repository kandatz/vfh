/* ========================================================================
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ======================================================================== */
#ifndef YUIWONGVFHIMPL_VFP_HPP
#define YUIWONGVFHIMPL_VFP_HPP 1
#include <vector>
#include "Eigen/Eigen"
namespace yuiwong {
/**
 * @brief 2d cloud points to vfh range
 * @param[in] cloud cloud points in desired frame
 * @param[in] radius if <= @a radius, use +inf
 * @param[out] result converted result
 * result in input params unit (usually meters), and for [-HPi, HPi],
 * NOTE resultidx and resultidx+1 is same
 */
extern void CloudToVfhRange(
	std::vector<Eigen::Vector2d> const& cloud,
	double const radius,
	Eigen::Matrix<double, 361, 1>& result);
/**
 * @deprecated PLEASE USE CloudToVfhRange
 * @brief convert sensor_msgs/LaserScan to M_PI [-HPi, HPi] degree result,
 * if laser range more than [-HPi, HPi], laser will be cut,
 * if laser range less than [-HPi, HPi], part or all result will be +inf
 * @param[in] ranges valid ranges from LaserScan
 * @param[in] angleMin valid angle_min from LaserScan
 * @param[in] angleMax valid angle_max from LaserScan, should <= @a angleMin
 * @param[in] angleIncrement valid angle_increment from LaserScan, should > 0
 * @param[in|out] result convert result
 * result in input params unit (usually meters), and for [-HPi, HPi],
 * NOTE resultidx and resultidx+1 is same
 * @param[in] minDistance if distance < minDistance, then +inf
 * @return
 * 0 front full converted
 * 1 laser cutted and front full converted
 * 2 some or all result no convert
 * 3 laser cutted and result no convert
 * @note
 * - angleMin should <= angleMax
 * - angleIncrement should > 0
 * - angleMin and angleMax should let result at least 1,
 *   else no convert and all result is inf
 * - ranges / .. should be valid
 */
extern int ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	Eigen::Matrix<double, 361, 1>& result,
	double const minDistance = 1e-2);
}
#endif
