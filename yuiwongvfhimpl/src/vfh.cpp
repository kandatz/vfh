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
 * @return
 * - full result laser [-HPi, HPi] to idx[0, 361]
 * - result in meters
 */
Eigen::Matrix<double, 361, 1>& ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	Eigen::Matrix<double, 361, 1>& result)
{
	double const raysPerDegree = DegreeToRadian(1) / angleIncrement;
	double const range = angleMax - angleMin;
	int laserOffset;
	int resultOffset;
	if (DoubleCompare(range, M_PI) > 0) {
		laserOffset = (::fabs(angleMin) - HPi) / angleIncrement;
		resultOffset = 0;
	} else if (DoubleCompare(range, M_PI) < 0) {
		laserOffset = 0;
		resultOffset = ((HPi - ::fabs(angleMin)) / angleIncrement)
			/ raysPerDegree;
	} else {
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
	for (int i = resultOffset; i < 180; ++i) {
		step = static_cast<int>(raysPerDegree * i);
		/* calculate position in laser frame */
		if ((laserOffset + step) >= n) {
			continue;
		}
		double const r = ranges[laserOffset + step];
		if (std::isnan(r)) {
			continue;
		}
		result[i * 2] = result[(i * 2) + 1] = r;
	}
	result[360] = result[359];
	return result;
}
/**
 * @return
 * - full result laser [-HPi, HPi] to idx[0, 361]
 * - result in meters
 */
std::array<double, 361>& ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleIncrement,
	std::array<double, 361>& result)
{
	double const raysPerDegree = DegreeToRadian(1) / angleIncrement;
	int laserOffset;
	int resultOffset;
	if (DoubleCompare(::fabs(angleMin), HPi) > 0) {
		laserOffset = (::fabs(angleMin) - HPi) / angleIncrement;
		resultOffset = 0;
	} else if (DoubleCompare(::fabs(angleMin), HPi) < 0) {
		laserOffset = 0;
		resultOffset = ((HPi - ::fabs(angleMin)) / angleIncrement)
			/ raysPerDegree;
	} else {
		laserOffset = 0;
		resultOffset = 0;
	}
	int const n = ranges.size();
	YUIWONGLOGNDEBU(
		"ConvertScan",
		"resultOffset %d laserOffset %d n %d raysPerDegree %lf",
		resultOffset, laserOffset, n, raysPerDegree);
	std::fill(
		result.begin(), result.end(), std::numeric_limits<double>::max());
	int step;
	for (int i = resultOffset; i < 180; ++i) {
		step = static_cast<int>(raysPerDegree * i);
		/* calculate position in laser frame */
		if ((laserOffset + step) >= n) {
			continue;
		}
		double const r = ranges[laserOffset + step];
		if (std::isnan(r)) {
			continue;
		}
		result[i * 2] = result[(i * 2) + 1] = r;
	}
	result[360] = result[359];
	return result;
}
}
