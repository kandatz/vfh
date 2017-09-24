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
#include "Eigen/Eigen"
namespace yuiwong {
/**
 * @implements vfh* BaseVfhStar, i.e. vfh+
 * @see
 * - vfh http://www-personal.umich.edu/~johannb/Papers/paper17.pdf
 * - vfh+ http://www-personal.umich.edu/~johannb/Papers/paper16.pdf
 * - vfh* http://www.cs.cmu.edu/~iwan/papers/vfhstar.pdf
 */
struct BaseVfhStar {
	constexpr static double defaultTolerance = 0.2;
	constexpr static double defaultStampTolerance = 0.2;
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
		/**
		 * @param maxSpeed the maximum allowable speed of the robot, in m/s,
		 * default 0.4 m/s
		 */
		double maxSpeed;
		/**
		 * @param maxSpeedNarrowOpening
		 * the maximum allowable speed of the robot through a narrow opening
		 * in m/s, default 5e-2 m/s
		 */
		double maxSpeedNarrowOpening;
		/**
		 * @param maxSpeedWideOpening
		 * the maximum allowable speed of the robot through a wide opening
		 * in m/s, default 0.4 m/s
		 */
		double maxSpeedWideOpening;
		/**
		 * @param zeroSafetyDistance
		 * the minimum distance the robot is allowed to get to obstacles when
		 * stopped, in meters, default 1e-2 meters
		 * @note maxSafetyDistance should >= zeroSafetyDistance
		 */
		double zeroSafetyDistance;
		/**
		 * @param maxSafetyDistance
		 * the minimum distance the robot is allowed to get to obstacles when
		 * travelling at max linear velocity, in meters, default 0.3 meters
		 */
		double maxSafetyDistance;
		/**
		 * @param zeroMaxTurnrate
		 * the maximum allowable turnrate of the robot when stopped,
		 * in radians/s, default DegreeToRadian(80) radians/s
		 * @note "zero" should >= "max"
		 */
		double zeroMaxTurnrate;
		/**
		 * @param maxMaxTurnrate
		 * the maximum allowable turnrate of the robot when travelling at
		 * max linear velocity,
		 * in radians/s, default DegreeToRadian(40)
		 */
		double maxMaxTurnrate;
		/**
		 * @param zeroFreeSpaceCutoff
		 * unitless value. the higher the value, the closer the robot will
		 * get to obstacles before avoiding (while stopped), default 4e6
		 * @note
		 * - free should >= obs
		 * - "zero" should >= "max"
		 */
		double zeroFreeSpaceCutoff;
		/**
		 * @param maxFreeSpaceCutoff
		 * unitless value. the higher the value, the closer the robot will
		 * get to obstacles before avoiding (while travelling at 1 m/s),
		 * default 2e6
		 */
		double maxFreeSpaceCutoff;
		/** @param zeroObsCutoff unitless value, default 4e6 */
		double zeroObsCutoff;
		/** @param maxObsCutoff, unitless value, default 2e6 */
		double maxObsCutoff;
		/**
		 * @param maxAcceleration,
		 * the maximum allowable acceleration of the robot, in m/s^2,
		 * default 0.1 m/s^2
		 */
		double maxAcceleration;
		double desiredDirectionWeight;/* default 5.0 */
		double currentDirectionWeight;/* default 1.0 */
		double minTurnRadiusSafetyFactor;/* default 1.0 */
		/** @param robotRadius, in meters, default 0.2 meters */
		double robotRadius;
		Param();
	};
	BaseVfhStar(Param const& param);
	virtual ~BaseVfhStar() = default;
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
	virtual void update(
		Eigen::Matrix<double, 361, 1> const& laserRanges,
		double const currentLinearX,
		double const goalDirection,
		double const goalDistance,
		double const goalDistanceTolerance,
		double& chosenLinearX,
		double& chosenAngularZ);
	inline void setRobotRadius(double const robotRadius) {
		this->robotRadius = robotRadius;
	}
	/**
	 * @brief get the safety distance at the given speed
	 * @param speed given speed, in m/s
	 * @return the safety distance, in meters
	 */
	double getSafetyDistance(double const speed) const;
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
	/** @brief angle to goal, in radians. 0 is to our right */
	inline double getDesiredAngle() const { return this->desiredDirection; }
	inline double getPickedAngle() const { return this->pickedDirection; }
protected:
	void allocate();
	/**
	 * @brief build the primary polar histogram
	 * @param laserRanges laser (or sonar) readings
	 * @param speed robot speed
	 * @return false when something inside our safety distance,
	 * should brake hard and turn on the spot, else return true
	 */
	bool buildPrimaryPolarHistogram(
		Eigen::Matrix<double, 361, 1> const& laserRanges, double const speed);
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
	/** @brief select the used direction */
	void selectDirection();
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
	int getMinTurningRadiusIndex(double const speed) const;
	/**
	 * @brief calcualte the cells magnitude
	 * @param laserRanges laser (or sonar) readings
	 * @param speed robot speed, m/s
	 * @return true
	 */
	virtual bool calculateCellsMagnitude(
		Eigen::Matrix<double, 361, 1> const& laserRanges, double const speed);
	/**
	 * @brief get the current low binary histogram threshold, obs, free
	 * @param speed given speed, m/s
	 * @return the threshold
	 */
	double getFreeBinaryHistogram(double const speed) const;
	double getObsBinaryHistogram(double const speed) const;
	/**
	 * @brief select the candidate angle to decide the direction using the
	 * given weights
	 */
	void selectCandidateAngle();
	double const cellWidth;/* in meters */
	int const windowDiameter;/* in cells */
	double const sectorAngle;/* in radians */
	double const maxSpeed;/* m/s */
	double const maxSpeedNarrowOpening;/* m/s */
	double const maxSpeedWideOpening;/* m/s */
	/** @note maxSafetyDistance should >= zeroSafetyDistance */
	double const zeroSafetyDistance;/* in meters */
	double const maxSafetyDistance;/* in meters */
	/**
	 * @brief scale turnrate linearly between these two
	 * @note "zero" should >= "max"
	 */
	double const zeroMaxTurnrate;/* radians/s */
	double const maxMaxTurnrate;/* radians/s */
	/**
	 * @note
	 * - free should >= obs
	 * - "zero" should >= "max"
	 */
	double const zeroFreeBinaryHistogram;
	double const maxFreeBinaryHistogram;
	double const zeroObsBinaryHistogram;
	double const maxObsBinaryHistogram;
	double const maxAcceleration;/* m/s^2 */
	double const desiredDirectionWeight;
	double const currentDirectionWeight;
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
	/* minimum turning radius at different speeds, in meters */
	std::vector<double> minTurningRadius;
	double maxSpeedForPickedDirection;
	/*
	 * the histogram.
	 * this is public so that monitoring tools can get at it;
	 * it shouldn't be modified externally.
	 * sweeps in an anti-clockwise direction
	 */
	std::vector<double> histogram;
	std::vector<double> lastBinaryHistogram;
	std::vector<std::vector<double> > cellMagnitude;
	std::vector<std::vector<double> > cellDistance;/* in metres */
	std::vector<std::vector<double> > cellBaseMagnitude;
	std::vector<std::vector<double> > cellDirection;
	std::vector<std::vector<double> > cellEnlarge;
	/*
	 * cellSector[x][y] is a vector of indices to sectors that are effected
	 * if cell (x,y) contains an obstacle.
	 * cell enlargement is taken into account.
	 * access as: cellSector[speedIndex][x][y][sectorIndex]
	 */
	std::vector<std::vector<std::vector<std::vector<int> > > > cellSector;
	std::vector<double> candidateAngle;
	std::vector<double> candidateSpeed;
	double desiredDirection, goalDistance, goalDistanceTolerance;
	double pickedDirection;
	/* keep track of last update, so we can monitor acceleration */
	double lastUpdateTime;
	double lastChosenLinearX;/* in m/s */
	double lastPickedDirection;
};
struct VfhStar: public BaseVfhStar {
	VfhStar(Param const& param): BaseVfhStar(param) {}
	virtual ~VfhStar() = default;
	virtual void update(
		Eigen::Matrix<double, 361, 1> const& laserRanges,
		double const currentLinearX,
		double const goalDirection,
		double const goalDistance,
		double const goalDistanceTolerance,
		double& chosenLinearX,
		double& chosenAngularZ) override;
	inline void setProcessTimes(int const processTimes) {
		this->processTimes = processTimes;
	}
protected:
	/*virtual bool calculateCellsMagnitude(
		Eigen::Matrix<double, 361, 1> const& laserRanges,
		double const speed) override;*/
	/** @brief ds, in meters, default 2 * radius */
	double stepDistance;
	/** @brief ng, default ::floor(goalDistance / stepDistance) or 1 */
	int processTimes;
	Eigen::Vector2d position;
};
}
#endif
