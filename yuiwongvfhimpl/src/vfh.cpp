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
#include "yuiwong/double.hpp"
namespace yuiwong
{
std::array<double, 361>& ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	double const rangeMax,
	std::array<double, 361>& result)
{
	std::fill(result.begin(), result.end(), -1.0);
	size_t const n = ranges.size();
	double const laserSpan = angleMax - angleMin;
	if ((DoubleCompare(laserSpan, M_PI) > 0) || (n > 180)) {
		/* in case we are using hokuyo */
		int const startIndex = (-M_PI / 2 - angleMin) / angleIncrement;
		double const raysPerDegree = (M_PI / 180.0) / angleIncrement;
		int step;
		double r;
		for (unsigned i = 0; i < 180; ++i) {
			step = static_cast<int>(raysPerDegree * i);
			/* calculate position in laser frame */
			if ((startIndex + step) > (static_cast<int>(n) - 1)) {
				/* probably this is not necessary */
				step = step - 1;
			}
			r = ranges[startIndex + step] * 1e3;
			if (DoubleCompare(r, 10.0) < 0) {
				r = rangeMax * 1e3;
			}
			result[i * 2] = r;
			result[i * 2 + 1] = r;
		}
	} else {
		for (unsigned i = 0; i < 180; ++i) {
			/* in case we are using sick
			 * calculate position in laser frame */
			double const r = ranges[i] * 1e3;
			result[i * 2] = r;
			result[i * 2 + 1] = r;
		}
	}
	return result;
}
}