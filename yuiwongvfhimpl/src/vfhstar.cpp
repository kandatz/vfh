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
#include "yuiwong/vfhstar.hpp"
#include <math.h>
#include <iostream>
#include "yuiwong/debug.hpp"
#include "yuiwong/time.hpp"
#include "yuiwong/math.hpp"
#include "yuiwong/angle.hpp"
namespace yuiwong
{
static std::ostream& operator<<(std::ostream& os, std::vector<double> const& v)
{
	for (auto const& d: v) {
		os << d << " ";
	}
	return os;
}
BaseVfhStar::Param::Param():
	cellWidth(0.1),
	windowDiameter(60),
	sectorAngle(DegreeToRadian(5)),
	maxSpeed(0.4),
	maxSpeedNarrowOpening(5e-2),
	maxSpeedWideOpening(0.4),
	zeroSafetyDistance(1e-2),
	maxSafetyDistance(0.3),
	zeroMaxTurnrate(DegreeToRadian(80)),
	maxMaxTurnrate(DegreeToRadian(40)),
	turnrateIncre(3.0),
	zeroFreeSpaceCutoff(4e6),
	maxFreeSpaceCutoff(2e6),
	zeroObsCutoff(4e6),
	maxObsCutoff(2e6),
	maxAcceleration(0.2),
	desiredDirectionWeight(1.0),
	currentDirectionWeight(1.0),
	minTurnRadiusSafetyFactor(1.0),
	robotRadius(0.2) {}
BaseVfhStar::BaseVfhStar(Param const& param):
	cellWidth(param.cellWidth),
	windowDiameter(param.windowDiameter),
	sectorAngle(param.sectorAngle),
	maxSpeed(param.maxSpeed),
	maxSpeedNarrowOpening(param.maxSpeedNarrowOpening),
	maxSpeedWideOpening(param.maxSpeedWideOpening),
	zeroSafetyDistance(param.zeroSafetyDistance),
	maxSafetyDistance(param.maxSafetyDistance),
	zeroMaxTurnrate(param.zeroMaxTurnrate),
	maxMaxTurnrate(param.maxMaxTurnrate),
	turnrateIncre(param.turnrateIncre),
	zeroFreeBinaryHistogram(param.zeroFreeSpaceCutoff),
	maxFreeBinaryHistogram(param.maxFreeSpaceCutoff),
	zeroObsBinaryHistogram(param.zeroObsCutoff),
	maxObsBinaryHistogram(param.maxObsCutoff),
	maxAcceleration(param.maxAcceleration),
	desiredDirectionWeight(param.desiredDirectionWeight),
	currentDirectionWeight(param.currentDirectionWeight),
	minTurnRadiusSafetyFactor(param.minTurnRadiusSafetyFactor),
	robotRadius(param.robotRadius),
	desiredDirection(HPi),
	pickedDirection(HPi),
	turnrateScale(1.0),
	lastUpdateTime(-1.0),
	lastChosenLinearX(0),
	lastPickedDirection(pickedDirection)
{
	if (DoubleCompare(
		this->zeroSafetyDistance, this->maxSafetyDistance) == 0) {
		/* for the simple case of a fixed safety_dist, keep things simple */
		this->cellSectorTablesCount = 1;
	} else {
		this->cellSectorTablesCount = 20;
	}
}
/** @brief start up the vfh* algorithm */
void BaseVfhStar::init()
{
	//center_x = (int)floor(window_diameter / 2.0);
	this->centerX = static_cast<int>(::floor(this->windowDiameter / 2.0));
	//center_y = center_x;
	this->centerY = this->centerX;
	//hist_size = (int)rint(360.0 / sector_angle);
	this->histogramSize = static_cast<int>(::rint(DPi / this->sectorAngle));
	/*
	 * it works now
	 * let's leave the verbose debug statement out
	 */
	YUIWONGLOGNDEBU(
		"BaseVfhStar",
		"cellWidth %1.1lf windowDiameter %d sectorAngle %lf histogramSize %d "
		"robotRadius %1.1lf safetyDistance %lf %lf maxSpeed %lf "
		"maxTurnrate %lf %lf freespace cutoff %lf %lf obstacle cutoff %lf %lf"
		"desired direction weight %lf current direction weight %lf",
		this->cellWidth,
		this->windowDiameter,
		this->sectorAngle,
		this->histogramSize,
		this->robotRadius,
		this->zeroSafetyDistance,
		this->maxSafetyDistance,
		this->maxSpeed,
		this->zeroMaxTurnrate,
		this->maxMaxTurnrate,
		this->zeroFreeBinaryHistogram,
		this->maxFreeBinaryHistogram,
		this->zeroObsBinaryHistogram,
		this->maxObsBinaryHistogram,
		this->desiredDirectionWeight,
		this->currentDirectionWeight);
	this->allocate();
	std::fill(this->histogram.begin(), this->histogram.end(), 0);
	std::fill(
		this->lastBinaryHistogram.begin(), this->lastBinaryHistogram.end(), 1);
	/*
	 * for the following:
	 * - (x, y) = (0, 0) is to the front-left of the robot
	 * - (x, y) = (max, 0) is to the front-right of the robot
	 */
	double negsectorToNegdir = 0;
	double negsectorToPlusdir = 0;
	double plussectorToNegdir = 0;
	double plussectorToPlusdir = 0;
	for (int x = 0; x < this->windowDiameter; ++x) {
		for (int y = 0; y < this->windowDiameter; ++y) {
			//cell_mag[x][y] = 0;
			this->cellMagnitude[x][y] = 0;
			//cell_dist[x][y] = sqrt(pow((center_x - x), 2)
			//+ pow((center_y - y), 2)) * cell_width;
			this->cellDistanceMM[x][y] = ::sqrt(
				::pow((this->centerX - x), 2.0)
				+ ::pow((this->centerY - y), 2.0)) * 1e3 * this->cellWidth;
			//cell_base_mag[x][y] = pow((3000.0 - cell_dist[x][y]), 4)
			//	/ 100000000.0;
			this->cellBaseMagnitude[x][y] = ::pow(
				(3e3 - (this->cellDistanceMM[x][y])), 4.0) / 1e8;
			/* set up cell direction with the angle in radians to each cell */
			if (x < this->centerX) {
				if (y < centerY) {
					//cell_direction[x][y] =
					//atan((double)(center_y - y) / (double)(center_x - x));
					this->cellDirection[x][y] = ::atan(
						static_cast<double>(this->centerY - y)
						/ static_cast<double>(this->centerX - x));
					/*this->cellDirection[x][y] *= (360.0 / 6.28);
					this->cellDirection[x][y] =
						180.0 - this->cellDirection[x][y];*/
					this->cellDirection[x][y] =
						M_PI - this->cellDirection[x][y];
				} else if (y == this->centerY) {
					this->cellDirection[x][y] = M_PI;
				} else if (y > this->centerY) {
					//cell_direction[x][y] =
					//atan((double)(y - center_y) / (double)(center_x - x));
					this->cellDirection[x][y] = ::atan(
						static_cast<double>(y - this->centerY)
						/ static_cast<double>(this->centerX - x));
					/*this->cellDirection[x][y] *= (360.0 / 6.28);
					this->cellDirection[x][y] =
						180.0 + this->cellDirection[x][y];*/
					this->cellDirection[x][y] =
						M_PI + this->cellDirection[x][y];
				}
			} else if (x == this->centerX) {
				if (y < centerY) {
					//cell_direction[x][y] = 90.0;
					this->cellDirection[x][y] = HPi;
				} else if (y == this->centerY) {
					//cell_direction[x][y] = -1.0;
					//this->cellDirection[x][y] = -1.0;
					this->cellDirection[x][y] = -HpPiDiv180;
				} else if (y > this->centerY) {
					//cell_direction[x][y] = 270.0;
					this->cellDirection[x][y] = (M_PI / 2.0) * 3.0;
				}
			} else if (x > this->centerX) {
				if (y < this->centerY) {
					//cell_direction[x][y] =
					//atan((double)(center_y - y) / (double)(x - center_x));
					this->cellDirection[x][y] = ::atan(
						static_cast<double>(this->centerY - y)
						/ static_cast<double>(x - this->centerX));
					/*this->cellDirection[x][y] *= (360.0 / 6.28);*/
				} else if (y == this->centerY) {
					this->cellDirection[x][y] = 0.0;
				} else if (y > this->centerY) {
					//cell_direction[x][y] =
					//atan((double)(y - center_y) / (double)(x - center_x));
					this->cellDirection[x][y] = ::atan(
						static_cast<double>(y - this->centerY)
						/ static_cast<double>(x - this->centerX));
					/*this->cellDirection[x][y] *= (360.0 / 6.28);
					this->cellDirection[x][y] =
						360.0 - this->cellDirection[x][y];*/
					this->cellDirection[x][y] =
						DPi - this->cellDirection[x][y];
				}
			}
			/*
			 * for the case where we have a speed-dependent safety distance,
			 * calculate all tables
			 */
			for (int cellSectorTabIdx = 0;
				cellSectorTabIdx < this->cellSectorTablesCount;
				++cellSectorTabIdx) {
				//thistableMaxSpeed = (int)(((double)
				//(cell_sector_tablenum+1)/(double)NUM_CELL_SECTOR_TABLES) *
				//(double) MAX_SPEED);
				int const thistableMaxSpeed =
					(static_cast<double>(cellSectorTabIdx + 1)
					/ static_cast<double>(this->cellSectorTablesCount))
					* this->maxSpeed;
				/*
				 * set cell enlarge to the angle by which a an obstacle must
				 * be enlarged for this cell, at this speed
				 */
				if (DoubleCompare(this->cellDistanceMM[x][y]) > 0) {
					//r = robot_radius + get_safety_dist(thistableMaxSpeed);
					double const r = this->robotRadius
						+ this->getSafetyDistance(thistableMaxSpeed);
					//cell_enlarge[x][y] =
					//(double)asin(r / cell_dist[x][y]) * (180/m_pi);
					this->cellEnlarge[x][y] =
						::asin((r * 1e3) / this->cellDistanceMM[x][y]);
				} else {
					this->cellEnlarge[x][y] = 0;
				}
				this->cellSector[cellSectorTabIdx][x][y].clear();
				//plusdir = cell_direction[x][y] + cell_enlarge[x][y];
				double const plusDirection = this->cellDirection[x][y]
					+ this->cellEnlarge[x][y];
				//negdir = cell_direction[x][y] - cell_enlarge[x][y];
				double const negDirection = this->cellDirection[x][y]
					- this->cellEnlarge[x][y];
				int const n = DPi / this->sectorAngle;
				int i;
				for (i = 0; i < n; ++i) {
					/*
					 * set plusSector and negSector to the angles to the two
					 * adjacent sectors
					 */
					//plussector = (i + 1) * (double)sector_angle;
					double plusSector = (i + 1) * this->sectorAngle;
					//negsector = i * (double)sector_angle;
					double negSector = i * this->sectorAngle;
					if (DoubleCompare(negSector - negDirection, M_PI) > 0) {
						//negsectorToNegdir = negdir - (negsector - 360);
						negsectorToNegdir = negDirection
							- (negSector - DPi);
					} else if (DoubleCompare(negDirection - negSector, M_PI)
						> 0) {
						//negsectorToNegdir = negsector - (negdir + 360);
						negsectorToNegdir = negSector
							- (negDirection + DPi);
					} else {
						//negsectorToNegdir = negdir - negsector;
						negsectorToNegdir = negDirection - negSector;
					}
					if (DoubleCompare(plusSector - negDirection, M_PI) > 0) {
						//plussectorToNegdir = negdir - (plussector - 360);
						plussectorToNegdir = negDirection
							- (plusSector - DPi);
					} else if (DoubleCompare(negDirection - plusSector, M_PI)
						> 0) {
						//plussectorToNegdir = plussector - (negdir + 360);
						plussectorToNegdir = plusSector
							- (negDirection + DPi);
					} else {
						//plussectorToNegdir = negdir - plussector;
						plussectorToNegdir = negDirection - plusSector;
					}
					if (DoubleCompare(plusSector - plusDirection, M_PI) > 0) {
						//plussectorToPlusdir = plusdir - (plussector - 360);
						plussectorToPlusdir = plusDirection
							- (plusSector - DPi);
					} else if (DoubleCompare(plusDirection - plusSector, M_PI)
						> 0) {
						//plussectorToPlusdir = plussector - (plusdir + 360);
						plussectorToPlusdir = plusSector
							- (plusDirection + DPi);
					} else {
						//plussectorToPlusdir = plusdir - plussector;
						plussectorToPlusdir = plusDirection - plusSector;
					}
					if (DoubleCompare(negSector - plusDirection, M_PI) > 0) {
						//negsectorToPlusdir = plusdir - (negsector - 360);
						negsectorToPlusdir = plusDirection
							- (negSector - DPi);
					} else if (DoubleCompare(plusDirection - negSector, M_PI)
						> 0) {
						//negsectorToPlusdir = negsector - (plusdir + 360);
						negsectorToPlusdir = negSector
							- (plusDirection + DPi);
					} else {
						negsectorToPlusdir = plusDirection - negSector;
					}
				}
				bool negdirbw;
				if ((DoubleCompare(negsectorToNegdir) >= 0)
					&& (DoubleCompare(plussectorToNegdir) <= 0)) {
					negdirbw = true;
				} else {
					negdirbw = false;
				}
				bool plusdirbw;
				if ((DoubleCompare(negsectorToPlusdir) >= 0)
					&& (DoubleCompare(plussectorToPlusdir) <= 0)) {
					plusdirbw = true;
				} else {
					plusdirbw = false;
				}
				bool diraroundsector;
				if ((DoubleCompare(negsectorToNegdir) <= 0)
					&& (DoubleCompare(negsectorToPlusdir) >= 0)) {
					diraroundsector = true;
				} else {
					diraroundsector = false;
				}
				if ((DoubleCompare(plussectorToNegdir) <= 0)
					&& (DoubleCompare(plussectorToPlusdir) >= 0)) {
					plusdirbw = true;
				}
				if (plusdirbw || negdirbw || diraroundsector) {
					this->cellSector[cellSectorTabIdx][x][y].push_back(i);
				}
			}
		}
	}
	this->lastUpdateTime = NowSecond();
}
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
void BaseVfhStar::update(
	Eigen::Matrix<double, 361, 1> const& laserRanges,
	double const currentLinearX,
	double const goalDirection0,
	double const goalDistance0,
	double const goalDistanceTolerance,
	double& chosenLinearX,
	double& chosenAngularZ)
{
	double goalDirection;
	double goalDistance;
	if (DoubleCompare(::fabs(goalDirection0), HPi) > 0) {
		goalDirection = ::copysign(HPi, goalDirection0);
		goalDistance = 0;
	} else {
		goalDirection = goalDirection0;
		goalDistance = goalDistance0;
	}
	YUIWONGLOGNWARN("BaseVfhStar", "UPDATE!");
	double const now = NowSecond();
	double const diffSeconds = now - this->lastUpdateTime;
	this->lastUpdateTime = now;
	this->desiredDirection = goalDirection + HPi;
	this->goalDistance = goalDistance;
	this->goalDistanceTolerance = goalDistanceTolerance;
	/*
	 * set currentPoseSpeed to the maximum of
	 * the set point(lastChosenSpeed) and the current actual speed.
	 * this ensures conservative behaviour if the set point somehow ramps up
	 * beyond the actual speed.
	 * ensure that this speed is positive.
	 */
	double currentPoseSpeed;
	if (DoubleCompare(currentLinearX) < 0) {
		currentPoseSpeed = 0;
	} else {
		currentPoseSpeed = currentLinearX;
	}
	if (DoubleCompare(currentPoseSpeed, this->lastChosenLinearX) < 0) {
		currentPoseSpeed = this->lastChosenLinearX;
	}
	/*
	 * work out how much time has elapsed since the last update,
	 * so we know how much to increase speed by, given MAX_ACCELERATION.
	 */
	if (!this->buildPrimaryPolarHistogram(laserRanges,currentPoseSpeed)) {
		/*
		 * something's inside our safety distance:
		 * brake hard and turn on the spot
		 */
		this->pickedDirection = this->lastPickedDirection;
		this->maxSpeedForPickedDirection = 0;
		//this->lastPickedDirection = this->pickedDirection;
		this->turnrateScale = std::max(this->turnrateScale * 0.5, 0.01);
	} else {
		bool bd;
		{
			int const idx = ::rint(RadianToDegree(goalDirection * 2.0));
			double const d = laserRanges[idx];
			double const sd = this->getSafetyDistance(currentPoseSpeed)
				+ this->robotRadius;
			if (DoubleCompare(d, sd) < 0) {
				bd = true;
			} else {
				bd = false;
			}
		}
		if (bd) {
			this->turnrateScale = std::min(this->turnrateScale * 1.01, 1.0);
		} else {
			this->turnrateScale = std::min(this->turnrateScale * 1.1, 1.0);
		}
		this->buildBinaryPolarHistogram(currentPoseSpeed);
		this->buildMaskedPolarHistogram(currentPoseSpeed);
		/*
		 * set pickedDirection, lastPickedDirection,
		 * and maxSpeedForPickedDirection
		 */
		this->selectDirection();
		/*if (bd && (DoubleCompare(
			this->lastPickedDirection, this->desiredDirection) == 0)) {
			YUIWONGLOGNDEBU("BaseVfhStar", "front ok but desiredDire bad");
			this->lastPickedDirection *= 0.5;
			this->pickedDirection = this->lastPickedDirection;
		}*/
	}
	/*
	 * ok, so now we've chosen a direction. time to choose a speed.
	 * how much can we change our speed by?
	 */
	double speedIncr;
	if ((diffSeconds > 0.3) || (diffSeconds < 0)) {
		/*
		 * Either this is the first time we've been updated, or something's
		 * a bit screwy and
		 * update hasn't been called for a while. Don't want a sudden burst of
		 * acceleration,
		 * so better to just pick a small value this time, calculate properly
		 * next time.
		 */
		speedIncr = 1e-2;
	} else {
		speedIncr = this->maxAcceleration * diffSeconds;
	}
	if (DoubleCompare(::fabs(speedIncr), 1e-4) <= 0) {
		speedIncr = 1e-4;
	}
	if (this->cannotTurnToGoal()) {
		/*
		 * the goal too close -- we can't turn tightly enough to
		 * get to it, so slow down
		 */
		YUIWONGLOGNDEBU("BaseVfhStar", "too close: cannotTurnToGoal");
		speedIncr = -speedIncr;
	}
	/*double const maxvForDistance = std::min(
		this->maxSpeed,
		this->maxAcceleration * (goalDistance / ::fabs(speedIncr)));*/
	/* accelerate (if we're not already at maxSpeedForPickedDirection) */
	double const v = this->lastChosenLinearX + speedIncr;
	double chosenLinearX0 = std::min(
		v, static_cast<double>(this->maxSpeedForPickedDirection));
	/* set the chosen turnrate, and possibly modify the chosen speed */
	double chosenTurnrate = 0;
	this->setMotion(currentPoseSpeed, chosenLinearX0, chosenTurnrate);
	/*chosenLinearX = std::min(maxvForDistance, chosenLinearX0);*/
	chosenLinearX = chosenLinearX0;
	chosenAngularZ = NormalizeAngle(chosenTurnrate);
	/*if (DoubleCompare(chosenLinearX) > 0) {
		chosenAngularZ *= this->turnrateScale;
		this->lastPickedDirection = ::atan2(
			chosenAngularZ / this->maxMaxTurnrate,
			chosenLinearX / this->currentMaxSpeed);
		if (DoubleCompare(::fabs(this->lastPickedDirection), HpPiDiv4) > 0) {
			YUIWONGLOGNDEBU("BaseVfhStar", "wanna forward but angle large");
			this->lastPickedDirection = ::copysign(
				HpPiDiv4, this->lastPickedDirection);
		}
		this->lastPickedDirection += HPi;
		this->pickedDirection = this->lastPickedDirection;
		this->setMotion(currentPoseSpeed, chosenLinearX0, chosenTurnrate);
	}*/
	this->lastChosenLinearX = chosenLinearX0;
	YUIWONGLOGNDEBU(
		"BaseVfhStar",
		"[incr %lf] goal %lf %lf -> picked direction %lf -> max lx %lf -> "
		"FINAL %lf %lf",
		speedIncr,
		goalDirection,
		goalDistance,
		this->lastPickedDirection,
		this->maxSpeedForPickedDirection,
		this->lastChosenLinearX,
		chosenAngularZ);
}
/**
 * @brief get the safety distance at the given speed
 * @param speed given speed, in m/s
 * @return the safety distance, in meters
 */
double BaseVfhStar::getSafetyDistance(double const speed) const
{
	double d = this->zeroSafetyDistance + ((speed / this->currentMaxSpeed)
		* (this->maxSafetyDistance - this->zeroSafetyDistance));
	if (DoubleCompare(d) < 0) {
		d = 0;
	}
	return d;
}
/**
 * @brief set the current max speed
 * @param maxSpeed current max speed, in m/s
 */
void BaseVfhStar::setCurrentMaxSpeed(double const maxSpeed)
{
	this->currentMaxSpeed = std::min(maxSpeed, this->maxSpeed);
	int const n = static_cast<int>(this->currentMaxSpeed * 1e3) + 1;
	this->minTurningRadius.resize(n);
	// small chunks of forward movements and turns-in-place used to
	// estimate turning radius, coz I'm too lazy to screw around with limits
	// -> 0
	// Calculate the turning radius, indexed by speed.
	// Probably don't need it to be precise (changing in 1mm increments).
	// WARNING: This assumes that the max_turnrate that has been set for VFH is
	// accurate.
	for (int x = 0; x < n; ++x) {
		//dx = (double) x / 1e6; // dx in m/millisec
		double const dx = x / 1e6;/* dx in m/ms */
		//dtheta = ((m_pi/180)*(double)(getmaxturnrate(x))) / 1000.0;
		//dtheta in radians/millisec
		/* dtheta in radians/s -> radians/ms */
		double const dtheta = this->getMaxTurnrate(x) / 1e3;
		//min_turning_radius[x] = (int) (((dx / tan(dtheta))*1000.0)
		// * min_turn_radius_safety_factor); // in mm
		/* in meters */
		this->minTurningRadius[x] = (((dx / ::tan(dtheta)))
			* this->minTurnRadiusSafetyFactor);
	}
}
/**
 * @brief get the max turn rate at the given speed
 * @param speed current speed, m/s
 * @return max turn rate in radians
 */
double BaseVfhStar::getMaxTurnrate(double const speed) const
{
	//int val = (MAX_TURNRATE_0MS
	//- (int)(speed*(MAX_TURNRATE_0MS-MAX_TURNRATE_1MS)/1000.0));
	double val = this->zeroMaxTurnrate - ((speed / this->currentMaxSpeed)
		* (this->zeroMaxTurnrate - this->maxMaxTurnrate));
	if (DoubleCompare(val) < 0) {
		val = 0;
	}
	return val;
}
void BaseVfhStar::allocate()
{
	YUIWONGLOGNDEBU("BaseVfhStar", "allocate ..");
	this->cellDirection.clear();
	this->cellBaseMagnitude.clear();
	this->cellMagnitude.clear();
	this->cellDistanceMM.clear();
	this->cellEnlarge.clear();
	this->cellSector.clear();
	{
	std::vector<double> const tempv(this->windowDiameter, 0);
	this->cellDirection.resize(this->windowDiameter, tempv);
	this->cellBaseMagnitude.resize(this->windowDiameter, tempv);
	this->cellMagnitude.resize(this->windowDiameter, tempv);
	this->cellDistanceMM.resize(this->windowDiameter, tempv);
	this->cellEnlarge.resize(this->windowDiameter, tempv);
	}
	{
	std::vector<std::vector<int> > tempv(
		this->windowDiameter, std::vector<int>{});
	std::vector<std::vector<std::vector<int> > > const tempv2(
		this->windowDiameter, tempv);
	this->cellSector.resize(this->cellSectorTablesCount, tempv2);
	}
	this->histogram.clear();
	this->lastBinaryHistogram.clear();
	this->histogram.resize(this->histogramSize, 0);
	this->lastBinaryHistogram.resize(this->histogramSize, 0);
	this->setCurrentMaxSpeed(this->maxSpeed);
	YUIWONGLOGNDEBU("BaseVfhStar", "allocate done");
}
/**
 * @brief build the primary polar histogram
 * @param laserRanges laser (or sonar) readings
 * @param speed robot speed
 * @return false when something inside our safety distance,
 * should brake hard and turn on the spot, else return true
 */
bool BaseVfhStar::buildPrimaryPolarHistogram(
	Eigen::Matrix<double, 361, 1> const& laserRanges, double const speed)
{
	/* index into the vector of cell sector tables */
	std::fill(this->histogram.begin(), this->histogram.end(), 0);
	if (!this->calculateCellsMagnitude(laserRanges, speed)) {
		/* set hist to all blocked */
		//YUIWONGLOGNDEBU("BaseVfhStar", "histogram all blocked: 1");
		std::fill(this->histogram.begin(), this->histogram.end(), 1);
		return false;
	}
	int const speedIndex = this->getSpeedIndex(speed);
	/* only have to go through the cells in front */
	int const n = ::ceil(this->windowDiameter / 2.0);
	for (int y = 0; y <= n; ++y) {
		for (int x = 0; x < this->windowDiameter; ++x) {
			//for(i = 0;i<cell_sector[speed_index][x][y].size();i++) {
			//hist[cell_sector[speed_index][x][y][i]] += cell_mag[x][y];
			//}
			auto const& cs = this->cellSector[speedIndex][x][y];
			double const cm = this->cellMagnitude[x][y];
			int const sz = cs.size();
			for(int i = 0; i < sz; ++i) {
				this->histogram[cs[i]] += cm;
			}
		}
	}
	//YUIWONGLOGDEBUS("buildPrimaryPolarHistogram done histogram\n"
	//	<< this->histogram);
	return true;
}
/**
 * @brief build the binary polar histogram
 * @param speed robot speed, m/s
 */
void BaseVfhStar::buildBinaryPolarHistogram(double const speed)
{
	for (int x = 0; x < this->histogramSize; ++x) {
		if (DoubleCompare(
			this->histogram[x], this->getObsBinaryHistogram(speed)) > 0) {
			this->histogram[x] = 1.0;
		} else if (DoubleCompare(
			this->histogram[x], this->getFreeBinaryHistogram(speed)) < 0) {
			this->histogram[x] = 0.0;
		} else {
			this->histogram[x] = this->lastBinaryHistogram[x];
		}
	}
	for (int x = 0; x < this->histogramSize; ++x) {
		this->lastBinaryHistogram[x] = this->histogram[x];
	}
	//YUIWONGLOGDEBUS("buildBinaryPolarHistogram done histogram\n"
	//	<< this->histogram);
}
/**
 * @brief build the masked polar histogram
 * @param speed robot speed, m/s
 * @note this function also sets blocked circle radius
 */
void BaseVfhStar::buildMaskedPolarHistogram(double const speed)
{
	/*
	 * centerX[left|right] is the centre of the circles on either side that
	 * are blocked due to the robot's dynamics.
	 * Units are in cells, in the robot's local coordinate system
	 * (here +y is forward)
	 */
	int const minTurningRadiusIdx = this->getMinTurningRadiusIndex(speed);
	double const minTurningRadius =
		this->minTurningRadius[minTurningRadiusIdx];
	double const centerxright = this->centerX
		+ (minTurningRadius / this->cellWidth);
	double const centerxleft = this->centerX
		- (minTurningRadius / this->cellWidth);
	double const centery = this->centerY;
	this->blockedCircleRadius = minTurningRadius + this->robotRadius
		+ this->getSafetyDistance(speed);
	//YUIWONGLOGDEBUS("buildMaskedPolarHistogram minTurningRadiusIdx "
	//	<< minTurningRadiusIdx
	//	<< " of sz " << this->minTurningRadius.size()
	//	<< " blockedCircleRadius " << this->blockedCircleRadius);
	/*
	 * This loop fixes phi_left and phi_right
	 * so that they go through the inside-most occupied cells inside the
	 * left/right circles.
	 * These circles are centred at the left/right centres of rotation,
	 * and are of radius blocked circle radius.
	 * We have to go between phi_left and phi_right,
	 * due to our minimum turning radius.
	 * Only loop through the cells in front of us.
	 */
	int const n = ::ceil(this->windowDiameter / 2.0);
	double phi_left = M_PI;
	double phi_right = 0;
	double angleahead = HPi;
	for (int y = 0; y < n; ++y) {
		for (int x = 0; x < this->windowDiameter; ++x) {
			if (DoubleCompare(this->cellMagnitude[x][y]) == 0) {
				continue;
			}
			double const d = this->cellDirection[x][y];
			if ((DoubleCompare(DeltaAngle(d, angleahead)) > 0)
				&& (DoubleCompare(DeltaAngle(d, phi_right)) <= 0)) {
				/* the cell is between phi_right and angle_ahead */
				double const distr = ::hypot(centerxright - x, centery - y)
					* this->cellWidth;
				if (DoubleCompare(distr, this->blockedCircleRadius) < 0) {
					phi_right = d;
				}
			} else if ((DoubleCompare(DeltaAngle(d, angleahead)) <= 0)
				&& (DoubleCompare(DeltaAngle(d, phi_left)) > 0)) {
				/* the cell is between phi_left and angle_ahead */
				double const distl = ::hypot(centerxleft - x, centery - y)
					* this->cellWidth;
				if (DoubleCompare(distl, this->blockedCircleRadius) < 0) {
					phi_left = d;
				}
			}
		}
	}
	/* mask out everything outside phi_left and phi_right */
	for (int x = 0; x < this->histogramSize; ++x) {
		double const angle = x * this->sectorAngle;
		auto& h = this->histogram[x];
		if ((DoubleCompare(h) == 0)
			&& (((DoubleCompare(DeltaAngle(angle, phi_right)) <= 0)
			&& (DoubleCompare(DeltaAngle(angle, angleahead)) >= 0))
			|| ((DoubleCompare(DeltaAngle(angle, phi_left)) >= 0)
			&& (DoubleCompare(DeltaAngle(angle, angleahead)) <= 0)))) {
			h = 0;
		} else {
			h = 1;
		}
	}
	//YUIWONGLOGDEBUS("buildMaskedPolarHistogram done histogram\n"
	//	<< this->histogram << "\nl " << phi_left << " r " << phi_right);
}
/** @brief select the used direction */
void BaseVfhStar::selectDirection()
{
	this->candidateAngle.clear();
	this->candidateSpeed.clear();
	/* set start to sector of first obstacle */
	int start = -1;
	{
	//YUIWONGLOGDEBUS("histogram\n" << this->histogram);
	/* only look at the forward 180deg for first obstacle */
	int const n = this->histogramSize / 2;
	for(int i = 0; i < n; ++i) {
		if (DoubleCompare(this->histogram[i], 1) == 0) {
			start = i;
			break;
		}
	}
	}
	if (start == -1) {
		//if (!this->desiredDirHasObstacle) {
		this->pickedDirection = this->desiredDirection;
		this->lastPickedDirection = this->pickedDirection;
		//} else {
		//	this->pickedDirection = HPi;
		//	this->lastPickedDirection = HPi;
		//}
		this->maxSpeedForPickedDirection = this->currentMaxSpeed;
		/*if (DoubleCompare(::fabs(this->pickedDirection - HPi), HPi) <= 0) {
			this->maxSpeedForPickedDirection *= ::cos(
				this->pickedDirection - HPi);
		}*/
		YUIWONGLOGNDEBU(
			"BaseVfhStar",
			"front no obstacle detected: "
			"full speed towards goal: %lf, %lf, %lf",
			 this->pickedDirection,
			 this->lastPickedDirection,
			 this->maxSpeedForPickedDirection);
		return;
	}
	/* find the left and right borders of each opening */
	std::vector<std::pair<int, double> > border;
	std::pair<int, double> newborder;
	int const n = start + this->histogramSize;
	bool left = true;
	for (int i = start; i <= n; ++i) {
		if ((DoubleCompare(this->histogram[i % this->histogramSize]) == 0)
			&& left) {
			newborder.first = (i % this->histogramSize) * this->sectorAngle;
			left = false;
		}
		if ((DoubleCompare(this->histogram[i % this->histogramSize], 1) == 0)
			&& (!left)) {
			newborder.second = ((i % this->histogramSize) - 1)
				* this->sectorAngle;
			//if (new_border.second < 0) {
			//new_border.second += 360;
			//}
			if (DoubleCompare(newborder.second) < 0) {
				newborder.second += DPi;
			}
			border.push_back(newborder);
			left = true;
		}
	}
	/* consider each opening */
	double const veryNarrowO = DegreeToRadian(10);
	double const narrowO = DegreeToRadian(80);
	double const r40 = DegreeToRadian(40);
	for (auto const& b: border) {
		double const angle = DeltaAngle(b.first, b.second);
		if (DoubleCompare(::fabs(angle), veryNarrowO) < 0) {
			continue;/* ignore very narrow openings */
		}
		if (DoubleCompare(::fabs(angle), narrowO) < 0) {
			/* narrow opening: aim for the centre */
			double const newangle = b.first + (b.second - b.first) / 2.0;
			this->candidateAngle.push_back(newangle);
			this->candidateSpeed.push_back(std::min(
				this->currentMaxSpeed, this->maxSpeedNarrowOpening));
		} else {
			/*
			 * wide opening: consider the centre, and 'r40' from each border
			 */
			double newangle = b.first + (b.second - b.first) / 2.0;
			this->candidateAngle.push_back(newangle);
			this->candidateSpeed.push_back(this->currentMaxSpeed);
			//new_angle = (double)((border[i].first + 40) % 360);
			newangle = ::fmod(b.first + r40, DPi);
			this->candidateAngle.push_back(newangle);
			this->candidateSpeed.push_back(std::min(
				this->currentMaxSpeed, this->maxSpeedWideOpening));
			//new_angle = (double)(border[i].second - 40);
			newangle = b.second - r40;
			//if (new_angle < 0)
			//new_angle += 360;
			if (DoubleCompare(newangle) < 0) {
				newangle += DPi;
			}
			this->candidateAngle.push_back(newangle);
			this->candidateSpeed.push_back(std::min(
				this->currentMaxSpeed, this->maxSpeedWideOpening));
			/* see if candidate dir is in this opening */
			if ((DoubleCompare(DeltaAngle(
				this->desiredDirection,
				this->candidateAngle[this->candidateAngle.size() - 2])) < 0)
				&& (DoubleCompare(DeltaAngle(
				this->desiredDirection,
				this->candidateAngle[this->candidateAngle.size() - 1])) > 0)) {
				this->candidateAngle.push_back(this->desiredDirection);
				this->candidateSpeed.push_back(std::min(
					this->currentMaxSpeed, this->maxSpeedWideOpening));
			}
		}
	}
	this->selectCandidateAngle();
}
/**
 * @brief the robot going too fast, such does it overshoot before it can
 * turn to the goal?
 * @return true if the robot cannot turn to the goal
 */
bool BaseVfhStar::cannotTurnToGoal() const
{
	/*
	 * calculate this by seeing if the goal is inside the blocked circles
	 * (circles we can't enter because we're going too fast).
	 * radii set by buildMaskedPolarHistogram.
	 * coords of goal in local coord system:
	 */
	double goalx = this->goalDistance * ::cos(this->desiredDirection);
	double goaly = this->goalDistance * ::sin(this->desiredDirection);
	/*
	 * this is the distance between the centre of the goal and
	 * the centre of the blocked circle
	 */
	double distBetweenCentres = ::hypot(
		goalx - this->blockedCircleRadius, goaly);
	if (DoubleCompare(
		distBetweenCentres + this->goalDistanceTolerance,
		this->blockedCircleRadius) < 0) {
		/* right circle */
		return true;
	}
	distBetweenCentres = ::hypot(
		-goalx - this->blockedCircleRadius, goaly);
	if (DoubleCompare(
		distBetweenCentres + this->goalDistanceTolerance,
		this->blockedCircleRadius) < 0) {
		/* left circle */
		return true;
	}
	return false;
}
/**
 * @brief set the motion commands
 * @param actualSpeed the current speed, m/s
 * @param linearX the desire linear x speed, m/s
 * @param turnrate the desire turn rate, radians/s
 */
void BaseVfhStar::setMotion(
	double const actualSpeed, double& linearX, double& turnrate)
{
	double const mx = this->getMaxTurnrate(actualSpeed);
	int const pd = RadianToDegree(this->pickedDirection);
	/* this happens if all directions blocked, so just spin in place */
	if (DoubleCompare(linearX) <= 0) {
		turnrate = mx;
		linearX = 0;
	} else if ((pd > 270) && (pd < 360)) {
		turnrate = -mx;
	} else if ((pd > 180) && (pd < 270)) {
		turnrate = mx;
	} else {
		//turnrate = (int)rint(((double)(pickedDirection - 90) / 75.0) * mx);
		//turnrate = ((pd - 90) / 75.0) * mx;
		turnrate = (DegreeToRadian(pd - 90) * this->turnrateIncre) * mx;
		if (DoubleCompare(::fabs(turnrate), mx) > 0) {
			turnrate = ::copysign(mx, turnrate);
		}
	}
	YUIWONGLOGNDEBU("BaseVfhStar", "pd %d  mx %lf  TMP %lf", pd, mx,
		(pd - 90.0));
}
/**
 * @brief get the speed index (for the current local map)
 * @param speed given speed, m/s
 * @return the index speed
 */
int BaseVfhStar::getSpeedIndex(double const speed) const
{
	int idx = ::floor((speed / this->currentMaxSpeed)
		* this->cellSectorTablesCount);
	if (idx >= this->cellSectorTablesCount) {
		idx = this->cellSectorTablesCount - 1;
	}
	//YUIWONGLOGNDEBU("BaseVfhStar", "speed idx at %lf m/s: %d", speed, idx);
	return idx;
}
/** @param speed linear x velocity, m/s, >=0 */
int BaseVfhStar::getMinTurningRadiusIndex(double const speed) const
{
	int idx = speed * 1e3;
	ssize_t const sz = this->minTurningRadius.size();
	if (idx >= sz) {
		idx = sz - 1;
	}
	return sz;
}
/**
 * @brief calcualte the cells magnitude
 * @param laserRanges laser (or sonar) readings
 * @param speed robot speed, m/s
 * @return true
 */
bool BaseVfhStar::calculateCellsMagnitude(
	Eigen::Matrix<double, 361, 1> const& laserRanges, double const speed)
{
	double const safeD = this->getSafetyDistance(speed);
	double const r = this->robotRadius + safeD;
	/*
	 * This is a bit dodgy...
	 * Makes it possible to miss really skinny obstacles,
	 * since if the resolution of the cells is finer than the resolution of
	 * laser_ranges, some ranges might be missed.
	 * Rather than looping over the cells,
	 * should perhaps loop over the laserRanges.
	 * Only deal with the cells in front of the robot,
	 * since we can't sense behind.
	 */
	//YUIWONGLOGDEBUS("laserRanges\n" << laserRanges.transpose());
	for (int x = 0; x < this->windowDiameter; ++x) {
		int const n = ::ceil(this->windowDiameter / 2.0);
		for (int y = 0; y < n; ++y) {
			/* controllo se il laser passa attraverso la cella */
			double const cdis = this->cellDistanceMM[x][y] * 1e-3;
			double const cdir = this->cellDirection[x][y];
			int const idx = ::rint(RadianToDegree(cdir * 2.0));
			if ((idx > 360) || (idx < 0)) {
				throw std::logic_error("bad idx");
			}
			double const lr = laserRanges[idx];
			if (DoubleCompare(cdis + (this->cellWidth / 2.0), lr) > 0) {
				if ((DoubleCompare(cdis, r) < 0)
					&& !((x == this->centerX) && (y == this->centerY))) {
					/*
					 * damn, something got inside our safety distance...
					 * short-circuit this process.
					 */
					YUIWONGLOGNDEBU(
						"BaseVfhStar",
						"[dire %lf idx %d laser %lf] %d %d %d %d "
						"inside safety %lf",
						cdir,
						idx,
						lr,
						this->centerX,
						this->centerY,
						x,
						y,
						r);
					//this->cellMagnitude[x][y] = 1.0;
					return false;
				} else {
					// cella piena quindi:
					// assegno alla cella il peso che dipende dalla distanza
					this->cellMagnitude[x][y] = this->cellBaseMagnitude[x][y];
				}
			} else {
				/* è vuota perchè il laser ci passa oltre!!!! */
				this->cellMagnitude[x][y] = 0.0;
			}
		}
	}
	return true;
}
/**
 * @brief get the current low binary histogram threshold, free
 * @param speed given speed, m/s
 * @return the threshold
 */
double BaseVfhStar::getFreeBinaryHistogram(double const speed) const
{
	//return (binary_hist_low_0ms
	//- (speed*(binary_hist_low_0ms-binary_hist_low_1ms)/1000.0));
	return this->zeroFreeBinaryHistogram - (speed
		* (this->zeroFreeBinaryHistogram - this->maxFreeBinaryHistogram));
}
/**
 * @brief get the current high binary histogram threshold, obs
 * @param speed given speed, m/s
 * @return the threshold
 */
double BaseVfhStar::getObsBinaryHistogram(double const speed) const
{
	//return (binary_hist_high_0ms
	//- (speed*(binary_hist_high_0ms-binary_hist_high_1ms)/1000.0));
	return this->zeroObsBinaryHistogram - (speed *
		(this->zeroObsBinaryHistogram - this->maxObsBinaryHistogram));
}
/**
 * @brief select the candidate angle to decide the direction using the
 * given weights
 */
void BaseVfhStar::selectCandidateAngle()
{
	YUIWONGLOGDEBUS("selectCandidateAngle " << this->candidateAngle);
	if (this->candidateAngle.size() <= 0) {
		/*
		 * we're hemmed in by obstacles -- nowhere to go,
		 * so brake hard and turn on the spot.
		 */
		this->pickedDirection = this->lastPickedDirection;
		this->maxSpeedForPickedDirection = 0;
		this->lastPickedDirection = this->pickedDirection;
		return;
	}
	this->pickedDirection = HPi;
	double minweight = std::numeric_limits<double>::max();
	for (auto const& ca: this->candidateAngle) {
		double const weight = this->desiredDirectionWeight * ::fabs(
			DeltaAngle(this->desiredDirection, ca))
			+ this->currentDirectionWeight * ::fabs(DeltaAngle(
			this->lastPickedDirection, ca));
		if (DoubleCompare(weight, minweight) < 0) {
			minweight = weight;
			this->pickedDirection = ca;
			this->maxSpeedForPickedDirection = ca;
		}
	}
	this->lastPickedDirection = this->pickedDirection;
}
void VfhStar::update(
	Eigen::Matrix<double, 361, 1> const& laserRanges,
	double const currentLinearX,
	double const goalDirection,
	double const goalDistance,
	double const goalDistanceTolerance,
	double& chosenLinearX,
	double& chosenAngularZ)
{
	this->position = Eigen::Vector2d::Zero();
	this->BaseVfhStar::update(
		laserRanges,
		currentLinearX,
		goalDirection,
		goalDistance,
		goalDistanceTolerance,
		chosenLinearX,
		chosenAngularZ);
	YUIWONGLOGDEBUS("candidateAngle " << this->candidateAngle);
}
}
