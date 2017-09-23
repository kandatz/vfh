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
#include "yuiwong/vfh.hpp"
#include <cmath>
#include <limits>
#include "yuiwong/double.hpp"
#include "yuiwong/math.hpp"
#include "yuiwong/angle.hpp"
namespace yuiwong
{
/**
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
int ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	Eigen::Matrix<double, 361, 1>& result)
{
	double const range = angleMax - angleMin;
	(void)(range);
	double const raysPerDegree = DegreeToRadian(1) / angleIncrement;
	int laserOffset;
	int resultOffset;
	if (DoubleCompare(angleMin, -HPi) < 0) {
		YUIWONGLOGNDEBU("ConvertScan", "cut some beginning laser");
		/* min in range [..., -HPi): cut some beginning laser */
		laserOffset = (::fabs(angleMin) - HPi) / angleIncrement;
		resultOffset = 0;
	} else if ((DoubleCompare(angleMin) < 0)
		&& (DoubleCompare(angleMin, -HPi) > 0)) {
		YUIWONGLOGNDEBU("ConvertScan", "front result no convert");
		/* min: (-HPi, 0): some result cannot be set */
		laserOffset = 0;
		resultOffset = ((HPi - ::fabs(angleMin)) / angleIncrement)
			/ raysPerDegree;
	} else if (DoubleCompare(angleMin, HPi) > 0) {
		YUIWONGLOGNDEBU("ConvertScan", "all result no convert");
		laserOffset = 0;/* invalid */
		resultOffset = 180;
	} else if (DoubleCompare(angleMin) > 0) {
		YUIWONGLOGNDEBU("ConvertScan", "many front result no convert");
		laserOffset = 0;
		resultOffset = ((HPi + angleMin) / angleIncrement) / raysPerDegree;
	} else {
		YUIWONGLOGNDEBU("ConvertScan", "all laser used");
		laserOffset = 0;
		resultOffset = 0;
	}
	int const n = ranges.size();
	YUIWONGLOGNDEBU(
		"ConvertScan",
		"resultOffset %d laserOffset %d n %d raysPerDegree %lf",
		resultOffset, laserOffset, n, raysPerDegree);
	for (int i = 0; i < 361; ++i) {
		result[i] = std::numeric_limits<double>::max();
	}
	int step;
	for (int i = resultOffset; i <= 180; ++i) {
		step = static_cast<int>(raysPerDegree * (i - resultOffset));
		/* calculate position in laser frame */
		if ((laserOffset + step) >= n) {
			int const rest = 180 - i;
			if (0 == rest) {
				result[360] = ranges[n - 1];
			} else {
				YUIWONGLOGNWARN(
					"ConvertScan", "back result no convert, rest %d", rest);
				resultOffset = -1;
			}
			break;
		}
		double const r = ranges[laserOffset + step];
		if (std::isnan(r)) {
			continue;
		}
		result[i * 2] = result[(i * 2) + 1] = r;
	}
	if ((0 == laserOffset) && (0 == resultOffset)) {
		return 0;
	} else if ((0 != laserOffset) && (0 == resultOffset)) {
		return 1;
	} else if ((0 == laserOffset) && (0 != resultOffset)) {
		return 2;
	} else {
		return 3;
	}
}
}
