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
#ifndef YUIWONGVFHIMPL_VFPSTAR_HPP
#define YUIWONGVFHIMPL_VFPSTAR_HPP 1
#include <vector>
#include <array>
namespace yuiwong {
/**
 * @implements vfh*
 * @see
 * - vfh http://www-personal.umich.edu/~johannb/Papers/paper16.pdf
 */
struct VfhStar {
	struct Param {
		/**
		 * @param cellWidth, local occupancy map grid size, in meters,
		 * default: 0.1 m
		 */
		double cellWidth;/* in meters */
		/**
		 * @param windowDiameter, dimensions of occupancy map
		 * (map consists of window diameter X), in cells, default 60
		 */
		int windowDiameter;
		/**
		 * @param sectorAngle, histogram angular resolution, in radians,
		 * default: DegreeToRadian(5) radians
		 */
		double sectorAngle;
		double maxSpeed;/* in m/s */
		double safetyDistance0ms;/* in meters */
		double safetyDistance1ms;/* in meters */
		double maxTurnrate0ms;/* radians/s */
		double maxTurnrate1ms;/* radians/s */
		double binaryHistogramLow0ms;
		double binaryHistogramHigh0ms;
		double binaryHistogramLow1ms;
		double binaryHistogramHigh1ms;
		double maxAcceleration;/* in m/s^2, default 0.1 m/s^2 */
		double minTurnRadiusSafetyFactor;/* default 1.0 */
		/** @param robotRadius, in meters, default 0.2 meters */
		double robotRadius;
		Param();
	};
	VfhStar(Param const& param);
	virtual ~VfhStar() = default;
	/* total projected distance dt = ng * ds */
	/** @brief start up the vfh* algorithm */
	void init();
	/**
	 * @brief update the vfh+ state using the laser readings and the robot
	 * speed
	 * @param laserRanges the laser (or sonar) readings, by convertScan
	 * @param currentLinearX the current robot linear x velocity, in meter/s
	 * @param goalDirection the desired direction, in radian,
	 * 0 is to the right
	 * @param goalDistance the desired distance, in meter
	 * @param goalDistanceTolerance the distance tolerance from the goal, in
	 * meter
	 * @param[out] chosenLinearX the chosen linear x velocity to drive the
	 * robot,
	 * in meter/s
	 * @param[out] chosenAngularZ the chosen turn rathe to drive the robot, in
	 * radian/s
	 */
	void update(
		std::array<double, 361> const& laserRanges,
		double const currentLinearX,
		double const goalDirection,
		double const goalDistance,
		double const goalDistanceTolerance,
		double& chosenLinearX,
		double& chosenAngularZ);
	/**
	 * @brief get the safety distance at the given speed
	 * @param speed given speed, in m/s
	 * @return the safety distance
	 */
	int getSafetyDistance(double const speed) const;
	/**
	 * @brief set the current max speed
	 * @param maxSpeed current max speed, in m/s
	 */
	void setCurrentMaxSpeed(double const maxSpeed);
	/**
	 * @brief get the max turn rate at the given speed
	 * @param speed current speed, m/s
	 * @return max turn rate in radians
	 */
	double getMaxTurnrate(double const speed) const;
protected:
	void allocate();
	/**
	 * @brief build the primary polar histogram
	 * @param laserRanges laser (or sonar) readings
	 * @param speed robot speed
	 * @return false when something's inside our safety distance,
	 * should brake hard and turn on the spot, else return true
	 */
	bool buildPrimaryPolarHistogram(
		std::array<double, 361> const& laserRanges, double const speed);
	/**
	 * @brief build the binary polar histogram
	 * @param speed robot speed, m/s
	 */
	void buildBinaryPolarHistogram(double const speed);
	/**
	 * @brief build the masked polar histogram
	 * @param speed robot speed, m/s
	 * @note this function also sets blocked circle radius
	 */
	void buildMaskedPolarHistogram(double const speed);
	/**
	 * @brief the robot going too fast, such does it overshoot before it can
	 * turn to the goal?
	 * @return true if the robot cannot turn to the goal
	 */
	bool cannotTurnToGoal() const;
	/**
	 * @brief set the motion commands
	 * @param actualSpeed the current speed, m/s
	 * @param linearX the desire linear x speed, m/s
	 * @param turnrate the desire turn rate, radians/s
	 */
	void setMotion(
		double const actualSpeed, double& linearX, double& turnrate);
	/**
	 * @brief get the speed index (for the current local map)
	 * @param speed given speed, m/s
	 * @return the index speed
	 */
	int getSpeedIndex(double const speed) const;
	/**
	 * @brief calcualte the cells magnitude
	 * @param laserRanges laser (or sonar) readings
	 * @param speed robot speed, m/s
	 * @return true
	 */
	bool calculateCellsMagnitude(
		std::array<double, 361> const& laserRanges, double const speed);
	/**
	 * @brief get the current low binary histogram threshold
	 * @param speed given speed, m/s
	 * @return the threshold
	 */
	double getBinaryHistogramLow(double const speed) const;
	double getBinaryHistogramHigh(double const speed) const;
	double const cellWidth;/* in meters */
	int const windowDiameter;/* in cells */
	double const sectorAngle;/* in radians */
	double const maxSpeed;/* m/s */
	double const safetyDistance0ms;/* in meters */
	double const safetyDistance1ms;/* in meters */
	/* scale turnrate linearly between these two */
	double const maxTurnrate0ms;/* radians/s */
	double const maxTurnrate1ms;/* radians/s */
	double const binaryHistogramLow0ms;
	double const binaryHistogramHigh0ms;
	double const binaryHistogramLow1ms;
	double const binaryHistogramHigh1ms;
	double const maxAcceleration;/* m/s^2 */
	double const minTurnRadiusSafetyFactor;/* default 1.0 */
	double robotRadius;/* in meters */
	/*
	 * radius of dis-allowed circles, either side of the robot,
	 * which we can't enter due to our minimum turning radius
	 */
	double blockedCircleRadius;
	int centerX;/* in cells */
	int centerY;/* in cells */
	int histogramSize;/* in sectors (over 360 degree) */
	int cellSectorTablesCount;
	double currentMaxSpeed;/* in m/s */
	/* keep track of last update, so we can monitor acceleration */
	double lastUpdateTime;
	/* minimum turning radius at different speeds, in meters */
	std::vector<double> minTurningRadius;
	double desiredDirection, goalDistance, goalDistanceTolerance;
	double lastChosenLinearX;/* in m/s */
	double pickedDirection, lastPickedDirection;
	double maxSpeedForPickedDirection;
	/*
	 * the histogram.
	 * this is public so that monitoring tools can get at it;
	 * it shouldn't be modified externally.
	 * sweeps in an anti-clockwise direction
	 */
	std::vector<double> histogram;
	std::vector<double> lastBinaryHistogram;
	std::vector<std::vector<double> > cellMag;
	std::vector<std::vector<double> > cellDistance;/* in metres */
	std::vector<std::vector<double> > cellBaseMag;
	std::vector<std::vector<double> > cellDirection;
	std::vector<std::vector<double> > cellEnlarge;
	/*
	 * cellSector[x][y] is a vector of indices to sectors that are effected
	 * if cell (x,y) contains an obstacle.
	 * cell enlargement is taken into account.
	 * access as: cellSector[speedIndex][x][y][sectorIndex]
	 */
	std::vector<std::vector<std::vector<std::vector<int> > > > cellSector;
	double stepDistance;/* ds */
	int processTimes;/* ng */
};
}
#endif
