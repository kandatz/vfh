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
#include <array>
#include "Eigen/Eigen"
namespace yuiwong {
/**
 * @return
 * - full result laser [-HPi, HPi] to idx[0, 361]
 * - result in meters
 */
extern Eigen::Matrix<double, 361, 1>& ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	Eigen::Matrix<double, 361, 1>& result);
/**
 * @return
 * - full result laser [-HPi, HPi] to idx[0, 361]
 * - result in meters
 */
extern std::array<double, 361>& ConvertScan(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	std::array<double, 361>& result);
}
#endif
