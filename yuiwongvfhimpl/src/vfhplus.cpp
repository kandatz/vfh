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
#include "yuiwong/vfhplus.hpp"
#if !YUIWONGVFHIMPL_NOOLDVFPPLUSIMPL
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <iostream>
#include "yuiwong/time.hpp"
#include "yuiwong/angle.hpp"
#include "yuiwong/debug.hpp"
#define DTOR(d) ((d) * M_PI / 180)
namespace yuiwong
{
/**
* VfhPlus constructor
* @param cell_size local map cell size
* @param window_diameter windows diameter
* @param sector_angle sector angle
* @param safety_dist_0ms safety obstacle distance at 0 m/s
* @param safety_dist_1ms safety obstacle distance at 1 m/s
* @param max_speed max allowed speed
* @param max_speed_narrow_opening max allowed speed in narrow opening
* @param max_speed_wide_opening max allowed speed in wide opening
* @param max_acceleration max allowed acceleration
* @param min_turnrate min turn rate
* @param max_turnrate_0ms max turn rate at 0 m/s
* @param max_turnrate_1ms max turn rate at 1 m/s
* @param min_turn_radius_safety_factor safety factor
* @param free_space_cutoff_0ms free space histogram threshold at 0 m/s
* @param obs_cutoff_0ms obstacle histogram threshold at 0 m/s
* @param free_space_cutoff_1ms free space histogram threshold at 1 m/s
* @param obs_cutoff_1ms obstacle histogram threshold at 1 m/s
* @param weight_desired_dir weight of the desired direction
* @param weight_current_dir weight of the current direction
*/
VfhPlus::VfhPlus(Param const& param):
	CELL_WIDTH(param.cell_size),
	WINDOW_DIAMETER(param.window_diameter),
	SECTOR_ANGLE(param.sector_angle),
	SAFETY_DIST_0MS(param.safety_dist_0ms),
	SAFETY_DIST_1MS(param.safety_dist_1ms),
	Current_Max_Speed(param.max_speed),
	MAX_SPEED(param.max_speed),
	MAX_SPEED_NARROW_OPENING(param.max_speed_narrow_opening),
	MAX_SPEED_WIDE_OPENING(param.max_speed_wide_opening),
	MAX_ACCELERATION(param.max_acceleration),
	MIN_TURNRATE(param.min_turnrate),
	MAX_TURNRATE_0MS(param.max_turnrate_0ms),
	MAX_TURNRATE_1MS(param.max_turnrate_1ms),
	MIN_TURN_RADIUS_SAFETY_FACTOR(param.min_turn_radius_safety_factor),
	Binary_Hist_Low_0ms(param.free_space_cutoff_0ms),
	Binary_Hist_High_0ms(param.obs_cutoff_0ms),
	Binary_Hist_Low_1ms(param.free_space_cutoff_1ms),
	Binary_Hist_High_1ms(param.obs_cutoff_1ms),
	U1(param.weight_desired_dir),
	U2(param.weight_current_dir),
	desiredDirection(90),
	pickedDirection(90),
	lastPickedDirection(pickedDirection),
	lastUpdateTime(-1.0),
	lastChosenLinearX(0)
{
this->Last_Binary_Hist = nullptr;
this->Hist = nullptr;
if (SAFETY_DIST_0MS == SAFETY_DIST_1MS)
{
// For the simple case of a fixed safety_dist, keep things simple.
NUM_CELL_SECTOR_TABLES = 1;
}
else
{
// AB: Made this number up...
NUM_CELL_SECTOR_TABLES = 20;
}
}
/**
* Class destructor
*/
VfhPlus::~VfhPlus()
{
if(this->Hist)
delete[] Hist;
if(this->Last_Binary_Hist)
delete[] Last_Binary_Hist;
}
/**
 * @brief get the max turn rate at the given speed
 * @param speed current speed, mm/s
 * @return max turn rate in degree
 */
int VfhPlus::getMaxTurnrate(int const speed) const
{
	//int val = (MAX_TURNRATE_0MS
	//- (int)(speed*(MAX_TURNRATE_0MS-MAX_TURNRATE_1MS)/1000.0));
	int val = (this->MAX_TURNRATE_0MS
		- ((1.0 * speed / this->Current_Max_Speed)
		* (this->MAX_TURNRATE_0MS - this->MAX_TURNRATE_1MS) / 1e3));
	if (val < 0) {
		val = 0;
	}
	return val;
}
/**
* Set the current max speed
* @param max_speed current max speed
*/
void VfhPlus::SetCurrentMaxSpeed(int max_speed)
{
this->Current_Max_Speed = std::min(max_speed, this->MAX_SPEED);
this->Min_Turning_Radius.resize(Current_Max_Speed+1);
// small chunks of forward movements and turns-in-place used to
// estimate turning radius, coz I'm too lazy to screw around with limits -> 0.
double dx, dtheta;
//
// Calculate the turning radius, indexed by speed.
// Probably don't need it to be precise (changing in 1mm increments).
//
// WARNING: This assumes that the max_turnrate that has been set for VFH is
// accurate.
//
for(int x = 0;x<= Current_Max_Speed;x++)
{
dx = (double) x / 1e6; // dx in m/millisec
dtheta = ((M_PI/180)*(double)(getMaxTurnrate(x))) / 1000.0; // dTheta in radians/millisec
Min_Turning_Radius[x] = (int) (((dx / tan(dtheta))*1000.0) * MIN_TURN_RADIUS_SAFETY_FACTOR); // in mm
}
}
// Doesn't need optimization: only gets called once per update.
/**
* Get the speed index (for the current local map)
* @param speed given speed
* @return the index speed
*/
int
VfhPlus::Get_Speed_Index(int speed)
{
int val = (int) floor(((double)speed/(double)Current_Max_Speed)*NUM_CELL_SECTOR_TABLES);
if (val >= NUM_CELL_SECTOR_TABLES)
val = NUM_CELL_SECTOR_TABLES-1;
// printf("Speed_Index at %dmm/s: %d\n",speed,val);
return val;
}
// Doesn't need optimization: only gets called on init plus once per update.
/**
* Get the safety distance at the given speed
* @param speed given speed
* @return the safety distance
*/
int VfhPlus::Get_Safety_Dist(int speed)
{
	int val = (int) (SAFETY_DIST_0MS + (int)(speed*(SAFETY_DIST_1MS-SAFETY_DIST_0MS)/1000.0));
	if (val < 0) {
		val = 0;
	}
	// printf("Safety_Dist at %dmm/s: %d\n",speed,val);
	return val;
}
// AB: Could optimize this with a look-up table, but it shouldn't make much
// difference: only gets called once per sector per update.
/**
* Get the current low binary histogram threshold
* @param speed given speed
* @return the threshold
*/
double
VfhPlus::Get_Binary_Hist_Low(int speed)
{
return (Binary_Hist_Low_0ms - (speed*(Binary_Hist_Low_0ms-Binary_Hist_Low_1ms)/1000.0));
}
// AB: Could optimize this with a look-up table, but it shouldn't make much
// difference: only gets called once per sector per update.
/**
* Get the current high binary histogram threshold
* @param speed given speed
* @return the threshold
*/
double
VfhPlus::Get_Binary_Hist_High(int speed)
{
return (Binary_Hist_High_0ms - (speed*(Binary_Hist_High_0ms-Binary_Hist_High_1ms)/1000.0));
}
/** @brief start up the vfh+ algorithm */
void VfhPlus::init()
{
	int x, y, i;
	double plus_dir = 0, neg_dir = 0, plus_sector = 0, neg_sector = 0;
	bool plus_dir_bw, neg_dir_bw, dir_around_sector;
	double neg_sector_to_neg_dir = 0, neg_sector_to_plus_dir = 0;
	double plus_sector_to_neg_dir = 0, plus_sector_to_plus_dir = 0;
	int cell_sector_tablenum, max_speed_this_table;
	double r;
	CENTER_X = (int)floor(WINDOW_DIAMETER / 2.0);
	CENTER_Y = CENTER_X;
	HIST_SIZE = (int)rint(360.0 / SECTOR_ANGLE);
	// it works now; let's leave the verbose debug statement out
	/*
	printf("CELL_WIDTH: %1.1f\tWINDOW_DIAMETER: %d\tSECTOR_ANGLE: %d\tROBOT_RADIUS: %1.1f\tSAFETY_DIST: %1.1f\tMAX_SPEED: %d\tMAX_TURNRATE: %d\tFree Space Cutoff: %1.1f\tObs Cutoff: %1.1f\tWeight Desired Dir: %1.1f\tWeight Current_Dir:%1.1f\n", CELL_WIDTH, WINDOW_DIAMETER, SECTOR_ANGLE, ROBOT_RADIUS, SAFETY_DIST, MAX_SPEED, MAX_TURNRATE, Binary_Hist_Low, Binary_Hist_High, U1, U2);
	*/
	VFH_Allocate();
	for(x = 0;x<HIST_SIZE;x++) {
	Hist[x] = 0;
	Last_Binary_Hist[x] = 1;
	}
	// For the following calcs:
	// - (x,y) = (0,0) is to the front-left of the robot
	// - (x,y) = (max,0) is to the front-right of the robot
	//
	for(x = 0;x<WINDOW_DIAMETER;x++) {
	for(y = 0;y<WINDOW_DIAMETER;y++) {
	Cell_Mag[x][y] = 0;
	Cell_Dist[x][y] = sqrt(pow((CENTER_X - x), 2) + pow((CENTER_Y - y), 2)) * CELL_WIDTH;
	Cell_Base_Mag[x][y] = pow((3000.0 - Cell_Dist[x][y]), 4) / 100000000.0;
	// Set up Cell_Direction with the angle in degrees to each cell
	if (x < CENTER_X) {
	if (y < CENTER_Y) {
	Cell_Direction[x][y] = atan((double)(CENTER_Y - y) / (double)(CENTER_X - x));
	Cell_Direction[x][y] *= (360.0 / 6.28);
	Cell_Direction[x][y] = 180.0 - Cell_Direction[x][y];
	} else if (y == CENTER_Y) {
	Cell_Direction[x][y] = 180.0;
	} else if (y > CENTER_Y) {
	Cell_Direction[x][y] = atan((double)(y - CENTER_Y) / (double)(CENTER_X - x));
	Cell_Direction[x][y] *= (360.0 / 6.28);
	Cell_Direction[x][y] = 180.0 + Cell_Direction[x][y];
	}
	} else if (x == CENTER_X) {
	if (y < CENTER_Y) {
	Cell_Direction[x][y] = 90.0;
	} else if (y == CENTER_Y) {
	Cell_Direction[x][y] = -1.0;
	} else if (y > CENTER_Y) {
	Cell_Direction[x][y] = 270.0;
	}
	} else if (x > CENTER_X) {
	if (y < CENTER_Y) {
	Cell_Direction[x][y] = atan((double)(CENTER_Y - y) / (double)(x - CENTER_X));
	Cell_Direction[x][y] *= (360.0 / 6.28);
	} else if (y == CENTER_Y) {
	Cell_Direction[x][y] = 0.0;
	} else if (y > CENTER_Y) {
	Cell_Direction[x][y] = atan((double)(y - CENTER_Y) / (double)(x - CENTER_X));
	Cell_Direction[x][y] *= (360.0 / 6.28);
	Cell_Direction[x][y] = 360.0 - Cell_Direction[x][y];
	}
	}
	// For the case where we have a speed-dependent safety_dist, calculate all tables
	for (cell_sector_tablenum = 0;
	cell_sector_tablenum < NUM_CELL_SECTOR_TABLES;
	cell_sector_tablenum++)
	{
	max_speed_this_table = (int) (((double)(cell_sector_tablenum+1)/(double)NUM_CELL_SECTOR_TABLES) *
	(double) MAX_SPEED);
	// printf("cell_sector_tablenum: %d, max_speed: %d, safety_dist: %d\n",
	// cell_sector_tablenum,max_speed_this_table,Get_Safety_Dist(max_speed_this_table));
	// Set Cell_Enlarge to the _angle_ by which a an obstacle must be
	// enlarged for this cell, at this speed
	if (Cell_Dist[x][y] > 0)
	{
	r = ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table);
	// Cell_Enlarge[x][y] = (double)atan(r / Cell_Dist[x][y]) * (180/M_PI);
	Cell_Enlarge[x][y] = (double)asin(r / Cell_Dist[x][y]) * (180/M_PI);
	}
	else
	{
	Cell_Enlarge[x][y] = 0;
	}
	Cell_Sector[cell_sector_tablenum][x][y].clear();
	plus_dir = Cell_Direction[x][y] + Cell_Enlarge[x][y];
	neg_dir = Cell_Direction[x][y] - Cell_Enlarge[x][y];
	for(i = 0;i<(360 / SECTOR_ANGLE);i++)
	{
	// Set plus_sector and neg_sector to the angles to the two adjacent sectors
	plus_sector = (i + 1) * (double)SECTOR_ANGLE;
	neg_sector = i * (double)SECTOR_ANGLE;
	if ((neg_sector - neg_dir) > 180) {
	neg_sector_to_neg_dir = neg_dir - (neg_sector - 360);
	} else {
	if ((neg_dir - neg_sector) > 180) {
	neg_sector_to_neg_dir = neg_sector - (neg_dir + 360);
	} else {
	neg_sector_to_neg_dir = neg_dir - neg_sector;
	}
	}
	if ((plus_sector - neg_dir) > 180) {
	plus_sector_to_neg_dir = neg_dir - (plus_sector - 360);
	} else {
	if ((neg_dir - plus_sector) > 180) {
	plus_sector_to_neg_dir = plus_sector - (neg_dir + 360);
	} else {
	plus_sector_to_neg_dir = neg_dir - plus_sector;
	}
	}
	if ((plus_sector - plus_dir) > 180) {
	plus_sector_to_plus_dir = plus_dir - (plus_sector - 360);
	} else {
	if ((plus_dir - plus_sector) > 180) {
	plus_sector_to_plus_dir = plus_sector - (plus_dir + 360);
	} else {
	plus_sector_to_plus_dir = plus_dir - plus_sector;
	}
	}
	if ((neg_sector - plus_dir) > 180) {
	neg_sector_to_plus_dir = plus_dir - (neg_sector - 360);
	} else {
	if ((plus_dir - neg_sector) > 180) {
	neg_sector_to_plus_dir = neg_sector - (plus_dir + 360);
	} else {
	neg_sector_to_plus_dir = plus_dir - neg_sector;
	}
	}
	plus_dir_bw = 0;
	neg_dir_bw = 0;
	dir_around_sector = 0;
	if ((neg_sector_to_neg_dir >= 0) && (plus_sector_to_neg_dir <= 0)) {
	neg_dir_bw = 1;
	}
	if ((neg_sector_to_plus_dir >= 0) && (plus_sector_to_plus_dir <= 0)) {
	plus_dir_bw = 1;
	}
	if ((neg_sector_to_neg_dir <= 0) && (neg_sector_to_plus_dir >= 0)) {
	dir_around_sector = 1;
	}
	if ((plus_sector_to_neg_dir <= 0) && (plus_sector_to_plus_dir >= 0)) {
	plus_dir_bw = 1;
	}
	if ((plus_dir_bw) || (neg_dir_bw) || (dir_around_sector)) {
	Cell_Sector[cell_sector_tablenum][x][y].push_back(i);
	}
	}
	}
	}
	}
	this->lastUpdateTime = NowSecond();
}
/**
* Allocate the VFH+ memory
*/
int VfhPlus::VFH_Allocate()
{
std::vector<double> temp_vec;
std::vector<int> temp_vec3;
std::vector<std::vector<int> > temp_vec2;
std::vector<std::vector<std::vector<int> > > temp_vec4;
int x;
Cell_Direction.clear();
Cell_Base_Mag.clear();
Cell_Mag.clear();
Cell_Dist.clear();
Cell_Enlarge.clear();
Cell_Sector.clear();
temp_vec.clear();
for(x = 0;x<WINDOW_DIAMETER;x++) {
temp_vec.push_back(0);
}
temp_vec2.clear();
temp_vec3.clear();
for(x = 0;x<WINDOW_DIAMETER;x++) {
temp_vec2.push_back(temp_vec3);
}
for(x = 0;x<WINDOW_DIAMETER;x++) {
Cell_Direction.push_back(temp_vec);
Cell_Base_Mag.push_back(temp_vec);
Cell_Mag.push_back(temp_vec);
Cell_Dist.push_back(temp_vec);
Cell_Enlarge.push_back(temp_vec);
temp_vec4.push_back(temp_vec2);
}
for(x = 0;x<NUM_CELL_SECTOR_TABLES;x++)
{
Cell_Sector.push_back(temp_vec4);
}
Hist = new double[HIST_SIZE];
Last_Binary_Hist = new double[HIST_SIZE];
this->SetCurrentMaxSpeed(MAX_SPEED);
return(1);
}
/**
 * @brief update the vfh+ state using the laser readings and the robot speed
 * @param laserRanges the laser (or sonar) readings, by convertScan
 * @param currentLinearX the current robot linear x velocity, in meter/s
 * @param goalDirection the desired direction, in radian, 0 is to the right
 * @param goalDistance the desired distance, in meter
 * @param goalDistanceTolerance the distance tolerance from the goal, in
 * meter
 * @param[out] chosenLinearX the chosen linear x velocity to drive the robot,
 * in meter/s
 * @param[out] chosenAngularZ the chosen turn rathe to drive the robot, in
 * radian/s
 */
void VfhPlus::update(
	Eigen::Matrix<double, 361, 1> const& laserRanges,
	double const currentLinearX,
	double const goalDirection,
	double const goalDistance,
	double const goalDistanceTolerance,
	double& chosenLinearX,
	double& chosenAngularZ)
{
	double const now = NowSecond();
	double const diffSeconds = now - this->lastUpdateTime;
	this->lastUpdateTime = now;
	this->desiredDirection = RadianToDegree(goalDirection + (M_PI / 2.0));
	this->goaldist = goalDistance * 1e3;
	this->goaldistTolerance = goalDistanceTolerance * 1e3;
	// Set currentPoseSpeed to the maximum of
	// the set point (lastChosenSpeed) and the current actual speed.
	// This ensures conservative behaviour if the set point somehow ramps up
	// beyond the actual speed.
	// Ensure that this speed is positive.
	int currentPoseSpeed;
	if (DoubleCompare(currentLinearX) < 0) {
		currentPoseSpeed = 0;
	} else {
		currentPoseSpeed = currentLinearX * 1e3;
	}
	if (DoubleCompare(currentPoseSpeed, this->lastChosenLinearX * 1e3)
		< 0) {
		currentPoseSpeed = this->lastChosenLinearX * 1e3;
	}
	// printf("update: currentPoseSpeed = %d\n",currentPoseSpeed);
	// Work out how much time has elapsed since the last update,
	// so we know how much to increase speed by, given MAX_ACCELERATION.
	// printf("update: buildPrimaryPolarHistogram\n");
	if (buildPrimaryPolarHistogram(laserRanges,currentPoseSpeed) == 0) {
		// Something's inside our safety distance: brake hard and
		// turn on the spot
		pickedDirection = lastPickedDirection;
		maxSpeedForPickedDirection = 0;
		lastPickedDirection = pickedDirection;
	} else {
		buildBinaryPolarHistogram(currentPoseSpeed);
		buildMaskedPolarHistogram(currentPoseSpeed);
		// Sets pickedDirection, lastPickedDirection,
		// and maxSpeedForPickedDirection
		selectDirection();
	}
	// printf("Picked Angle: %f\n", pickedDirection);
	// OK, so now we've chosen a direction. Time to choose a speed.
	// How much can we change our speed by?
	double speedIncr;
	if ((diffSeconds > 0.3) || (diffSeconds < 0)) {
		// Either this is the first time we've been updated, or something's
		// a bit screwy and
		// update hasn't been called for a while. Don't want a sudden burst of
		// acceleration,
		// so better to just pick a small value this time, calculate properly
		// next time.
		//speedIncr = 10;
		speedIncr = 1e-2;
	} else {
		speedIncr = MAX_ACCELERATION * diffSeconds * 1e-3;
	}
	if (DoubleCompare(::fabs(speedIncr), 1e-4) <= 0) {
		speedIncr = 1e-4;
	}
	if (this->cantTurnToGoal()) {
		// The goal's too close -- we can't turn tightly enough to
		// get to it, so slow down...
		speedIncr = -speedIncr;
	}
	// Accelerate (if we're not already at maxSpeedForPickedDirection).
	double const v = this->lastChosenLinearX + speedIncr;
	double chosenLinearX0 = std::min(
		v, static_cast<double>(maxSpeedForPickedDirection) * 1e-3);
	// printf("Max Speed for picked angle: %d\n",maxSpeedForPickedDirection);
	// Set the chosen_turnrate, and possibly modify the chosen_speed
	int chosenTurnrate = 0;
	this->setMotion(chosenLinearX0, chosenTurnrate, currentPoseSpeed);
	chosenLinearX = chosenLinearX0;
	chosenAngularZ = NormalizeAngle(DegreeToRadian(chosenTurnrate));
	this->lastChosenLinearX = chosenLinearX0;
	YUIWONGLOGDEBUS("FINAL speedIncr m/s " << speedIncr
		<< " lx m/s " << chosenLinearX << " az r/s " << chosenAngularZ
		<< " pickdire " << this->pickedDirection);
}
/**
* The robot going too fast, such does it overshoot before it can turn to the goal?
* @return true if the robot cannot turn to the goal
*/
bool VfhPlus::cantTurnToGoal()
{
// Calculate this by seeing if the goal is inside the blocked circles
// (circles we can't enter because we're going too fast). Radii set
// by buildMaskedPolarHistogram.
// Coords of goal in local coord system:
double goal_x = this->goaldist * cos(DTOR(this->desiredDirection));
double goal_y = this->goaldist * sin(DTOR(this->desiredDirection));
// AlexB: Is this useful?
// if (goal_y < 0)
// {
// printf("Goal behind\n");
// return true;
// }
// This is the distance between the centre of the goal and
// the centre of the blocked circle
double dist_between_centres;
// printf("cantTurnToGoal: goaldist = %f\n",goaldist);
// printf("cantTurnToGoal: Angle_To_Goal = %f\n",desiredDirection);
// printf("cantTurnToGoal: Blocked_Circle_Radius = %f\n",Blocked_Circle_Radius);
// right circle
dist_between_centres = hypot(goal_x - this->Blocked_Circle_Radius, goal_y);
if (dist_between_centres+this->goaldistTolerance < this->Blocked_Circle_Radius)
{
// printf("Goal close & right\n");
return true;
}
// left circle
dist_between_centres = hypot(-goal_x - this->Blocked_Circle_Radius, goal_y);
if (dist_between_centres+this->goaldistTolerance < this->Blocked_Circle_Radius)
{
// printf("Goal close & left.\n");
return true;
}
return false;
}
/**
* Difference between two integer angle
* @param a1 first angle
* @param a2 second angle
* @return the difference
*/
double VfhPlus::deltaAngle(int a1, int a2)
{
return(deltaAngle((double)a1, (double)a2));
}
/**
 * @brief difference between two double angle
 * @param a1 first angle
 * @param a2 second angle
 * @return the difference [-180, 180]
 */
double VfhPlus::deltaAngle(double const& a1, double const& a2)
{
	double const diff = a2 - a1;
	return NormalizeDegreeAngle(diff);
}
/**
 * @brief calculate the bisector between two angle
 * @param angle1 first angle
 * @param angle2 second angle
 * @return the bisector angle [-360, 360)
 */
int VfhPlus::bisectAngle(int const angle1, int const angle2)
{
	double const a = deltaAngle((double)angle1, (double)angle2);
	int const angle = static_cast<int>(::rint(angle1 + (a / 2.0)));
	return NormalizeDegreeAnglePositive(angle);
}
/**
* Select the candidate angle to decide the direction using the given weights
* @return 1
*/
int VfhPlus::Select_Candidate_Angle()
{
unsigned int i;
double weight, min_weight;
if (Candidate_Angle.size() == 0)
{
// We're hemmed in by obstacles -- nowhere to go,
// so brake hard and turn on the spot.
pickedDirection = lastPickedDirection;
maxSpeedForPickedDirection = 0;
lastPickedDirection = pickedDirection;
return(1);
}
pickedDirection = 90;
min_weight = 10000000;
for(i = 0;i<Candidate_Angle.size();i++)
{
//printf("CANDIDATE: %f\n", Candidate_Angle[i]);
weight = U1 * fabs(deltaAngle(desiredDirection, Candidate_Angle[i])) +
U2 * fabs(deltaAngle(lastPickedDirection, Candidate_Angle[i]));
if (weight < min_weight)
{
min_weight = weight;
pickedDirection = Candidate_Angle[i];
maxSpeedForPickedDirection = Candidate_Speed[i];
}
}
lastPickedDirection = pickedDirection;
return(1);
}
/**
* Select the used direction
* @return 1
*/
int VfhPlus::selectDirection()
{
int start, i, left;
double angle, new_angle;
std::vector<std::pair<int,int> > border;
std::pair<int,int> new_border;
Candidate_Angle.clear();
Candidate_Speed.clear();
//
// set start to sector of first obstacle
//
start = -1;
// only look at the forward 180deg for first obstacle.
for(i = 0;i<HIST_SIZE/2;i++)
{
if (Hist[i] == 1)
{
start = i;
break;
}
}
if (start == -1) {
pickedDirection = desiredDirection;
lastPickedDirection = pickedDirection;
maxSpeedForPickedDirection = Current_Max_Speed;
	YUIWONGLOGNDEBU(
		"VfhPlus",
		"No obstacles detected in front of us: full speed towards goal: "
		"%f, %f, %d",
		pickedDirection, lastPickedDirection, maxSpeedForPickedDirection);
return(1);
}
//
// Find the left and right borders of each opening
//
border.clear();
//printf("Start: %d\n", start);
left = 1;
for(i = start;i<= (start+HIST_SIZE);i++) {
if ((Hist[i % HIST_SIZE] == 0) && (left)) {
new_border.first = (i % HIST_SIZE) * SECTOR_ANGLE;
left = 0;
}
if ((Hist[i % HIST_SIZE] == 1) && (!left)) {
new_border.second = ((i % HIST_SIZE) - 1) * SECTOR_ANGLE;
if (new_border.second < 0) {
new_border.second += 360;
}
border.push_back(new_border);
left = 1;
}
}
//
// Consider each opening
//
for(i = 0;i<(int)border.size();i++)
{
// printf("BORDER: %d %d\n", border[i].first, border[i].second);
angle = deltaAngle(border[i].first, border[i].second);
if (fabs(angle) < 10)
{
// ignore very narrow openings
continue;
}
if (fabs(angle) < 80)
{
// narrow opening: aim for the centre
new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;
Candidate_Angle.push_back(new_angle);
Candidate_Speed.push_back(std::min(
	Current_Max_Speed,MAX_SPEED_NARROW_OPENING));
}
else
{
// wide opening: consider the centre, and 40deg from each border
new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;
Candidate_Angle.push_back(new_angle);
Candidate_Speed.push_back(Current_Max_Speed);
new_angle = (double)((border[i].first + 40) % 360);
Candidate_Angle.push_back(new_angle);
Candidate_Speed.push_back(std::min(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
new_angle = (double)(border[i].second - 40);
if (new_angle < 0)
new_angle += 360;
Candidate_Angle.push_back(new_angle);
Candidate_Speed.push_back(std::min(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
// See if candidate dir is in this opening
if ((deltaAngle(desiredDirection, Candidate_Angle[Candidate_Angle.size()-2]) < 0) &&
(deltaAngle(desiredDirection, Candidate_Angle[Candidate_Angle.size()-1]) > 0)) {
Candidate_Angle.push_back(desiredDirection);
Candidate_Speed.push_back(std::min(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
}
}
}
Select_Candidate_Angle();
return(1);
}
/**
* Print the cells directions
*/
void VfhPlus::Print_Cells_Dir()
{
int x, y;
printf("\nCell Directions:\n");
printf("****************\n");
for(y = 0;y<WINDOW_DIAMETER;y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
printf("%1.1f\t", Cell_Direction[x][y]);
}
printf("\n");
}
}
/**
* Print the cells magnitude
*/
void VfhPlus::Print_Cells_Mag()
{
int x, y;
printf("\nCell Magnitudes:\n");
printf("****************\n");
for(y = 0;y<WINDOW_DIAMETER;y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
printf("%1.1f\t", Cell_Mag[x][y]);
}
printf("\n");
}
}
/**
* Print the cells distances
*/
void VfhPlus::Print_Cells_Dist()
{
int x, y;
printf("\nCell Distances:\n");
printf("****************\n");
for(y = 0;y<WINDOW_DIAMETER;y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
printf("%1.1f\t", Cell_Dist[x][y]);
}
printf("\n");
}
}
/**
* Print the cells sectors
*/
void VfhPlus::Print_Cells_Sector()
{
int x, y;
unsigned int i;
printf("\nCell Sectors for table 0:\n");
printf("***************************\n");
for(y = 0;y<WINDOW_DIAMETER;y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
for(i = 0;i<Cell_Sector[0][x][y].size();i++) {
if (i < (Cell_Sector[0][x][y].size() -1)) {
printf("%d,", Cell_Sector[0][x][y][i]);
} else {
printf("%d\t\t", Cell_Sector[0][x][y][i]);
}
}
}
printf("\n");
}
}
/**
* Print the cells enlargement angles
*/
void VfhPlus::Print_Cells_Enlargement_Angle()
{
int x, y;
printf("\nEnlargement Angles:\n");
printf("****************\n");
for(y = 0;y<WINDOW_DIAMETER;y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
printf("%1.1f\t", Cell_Enlarge[x][y]);
}
printf("\n");
}
}
/**
* Print the histogram
*/
void VfhPlus::Print_Hist()
{
int x;
printf("Histogram:\n");
printf("****************\n");
for(x = 0;x<= (HIST_SIZE/2);x++) {
printf("%d,%1.1f\n", (x * SECTOR_ANGLE), Hist[x]);
}
printf("\n\n");
}
/**
* Calcualte the cells magnitude
* @param laser_ranges laser (or sonar) readings
* @param speed robot speed
* @return 1
*/
int VfhPlus::Calculate_Cells_Mag(
	Eigen::Matrix<double, 361, 1> const& laserRanges, int speed)
{
int x, y;
double safeSpeed = (double) Get_Safety_Dist(speed);
double r = ROBOT_RADIUS + safeSpeed;
//printf("Laser Ranges\n");
//printf("************\n");
//for(x = 0;x<= 360;x++) {
//printf("%d: %f %f\n", x, laser_ranges[x][0], r);
//}
// AB: This is a bit dodgy... Makes it possible to miss really skinny obstacles, since if the
// resolution of the cells is finer than the resolution of laser_ranges, some ranges might be missed.
// Rather than looping over the cells, should perhaps loop over the laser_ranges.
// Only deal with the cells in front of the robot, since we can't sense behind.
for(x = 0;x<WINDOW_DIAMETER;x++)
{
for(y = 0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
{
// 	 printf("Cell %d,%d: Cell_Dist is %f, range is %f i: %d (minimum is %f)\n",
// 	 x,
// 	 y,
// 	 Cell_Dist[x][y] + CELL_WIDTH / 2.0,
// 	 laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
// 	 (int)rint(Cell_Direction[x][y] * 2.0),
// 	 r);
// controllo se il laser passa attraverso la cella
if ((Cell_Dist[x][y] + CELL_WIDTH / 2.0) >
laserRanges[(int)rint(Cell_Direction[x][y] * 2.0)])
{
if (Cell_Dist[x][y] < r && !(x == CENTER_X && y == CENTER_Y))
{
// printf("Cell %d,%d: Cell_Dist is %f, range is %f (minimum is %f): too close...\n",
// x,
// y,
// Cell_Dist[x][y] + CELL_WIDTH / 2.0,
// laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
// r);
// printf("ROBOT_RADIUS %f, Get_Safety_Dist(speed) %f\n", ROBOT_RADIUS, safeSpeed);
// Damn, something got inside our safety_distance...
// Short-circuit this process.
return(0);
}
else
{
// cella piena quindi:
// assegno alla cella il peso che dipende dalla distanza
Cell_Mag[x][y] = Cell_Base_Mag[x][y];
}
} else {
// è vuota perchè il laser ci passa oltre!!!!
Cell_Mag[x][y] = 0.0;
}
}
}
return(1);
}
/**
* Build the primary polar histogram
* @param laser_ranges laser (or sonar) readings
* @param speed robot speed
* @return 1
*/
int VfhPlus::buildPrimaryPolarHistogram(
	Eigen::Matrix<double, 361, 1> const& laserRanges, int speed)
{
int x, y;
unsigned int i;
// index into the vector of Cell_Sector tables
int speed_index = Get_Speed_Index(speed);
// printf("buildPrimaryPolarHistogram: speed_index %d %d\n", speed_index, HIST_SIZE);
for(x = 0;x<HIST_SIZE;x++) {
Hist[x] = 0;
}
if (Calculate_Cells_Mag(laserRanges, speed) == 0)
{
	YUIWONGLOGNDEBU("VfhPlus", "histogram all blocked: 1");
// set Hist to all blocked
for(x = 0;x<HIST_SIZE;x++) {
Hist[x] = 1;
}
return 0;
}
// Print_Cells_Dist();
// Print_Cells_Dir();
// Print_Cells_Mag();
// Print_Cells_Sector();
// Print_Cells_Enlargement_Angle();
// Only have to go through the cells in front.
for(y = 0;y<= (int)ceil(WINDOW_DIAMETER/2.0);y++) {
for(x = 0;x<WINDOW_DIAMETER;x++) {
for(i = 0;i<Cell_Sector[speed_index][x][y].size();i++) {
Hist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];
}
}
}
return(1);
}
/**
* Build the binary polar histogram
* @param speed robot speed
* @return 1
*/
int VfhPlus::buildBinaryPolarHistogram(int speed)
{
int x;
for(x = 0;x<HIST_SIZE;x++) {
if (Hist[x] > Get_Binary_Hist_High(speed)) {
Hist[x] = 1.0;
} else if (Hist[x] < Get_Binary_Hist_Low(speed)) {
Hist[x] = 0.0;
} else {
Hist[x] = Last_Binary_Hist[x];
}
}
for(x = 0;x<HIST_SIZE;x++) {
Last_Binary_Hist[x] = Hist[x];
}
return(1);
}
//
// This function also sets Blocked_Circle_Radius.
//
/**
* Build the masked polar histogram
* @param speed robot speed
* @return 1
*/
int VfhPlus::buildMaskedPolarHistogram(int speed)
{
int x, y;
double center_x_right, center_x_left, center_y, dist_r, dist_l;
double angle_ahead, phi_left, phi_right, angle;
// center_x_[left|right] is the centre of the circles on either side that
// are blocked due to the robot's dynamics. Units are in cells, in the robot's
// local coordinate system (+y is forward).
center_x_right = CENTER_X + (Min_Turning_Radius[speed] / (double)CELL_WIDTH);
center_x_left = CENTER_X - (Min_Turning_Radius[speed] / (double)CELL_WIDTH);
center_y = CENTER_Y;
angle_ahead = 90;
phi_left = 180;
phi_right = 0;
Blocked_Circle_Radius = Min_Turning_Radius[speed] + ROBOT_RADIUS + Get_Safety_Dist(speed);
//
// This loop fixes phi_left and phi_right so that they go through the inside-most
// occupied cells inside the left/right circles. These circles are centred at the
// left/right centres of rotation, and are of radius Blocked_Circle_Radius.
//
// We have to go between phi_left and phi_right, due to our minimum turning radius.
//
//
// Only loop through the cells in front of us.
//
for(y = 0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
{
for(x = 0;x<WINDOW_DIAMETER;x++)
{
if (Cell_Mag[x][y] == 0)
continue;
if ((deltaAngle(Cell_Direction[x][y], angle_ahead) > 0) &&
(deltaAngle(Cell_Direction[x][y], phi_right) <= 0))
{
// The cell is between phi_right and angle_ahead
dist_r = hypot(center_x_right - x, center_y - y) * CELL_WIDTH;
if (dist_r < Blocked_Circle_Radius)
{
phi_right = Cell_Direction[x][y];
}
}
else if ((deltaAngle(Cell_Direction[x][y], angle_ahead) <= 0) &&
(deltaAngle(Cell_Direction[x][y], phi_left) > 0))
{
// The cell is between phi_left and angle_ahead
dist_l = hypot(center_x_left - x, center_y - y) * CELL_WIDTH;
if (dist_l < Blocked_Circle_Radius)
{
phi_left = Cell_Direction[x][y];
}
}
}
}
//
// Mask out everything outside phi_left and phi_right
//
for(x = 0;x<HIST_SIZE;x++)
{
angle = x * SECTOR_ANGLE;
if ((Hist[x] == 0) && (((deltaAngle((double)angle, phi_right) <= 0) &&
(deltaAngle((double)angle, angle_ahead) >= 0)) ||
((deltaAngle((double)angle, phi_left) >= 0) &&
(deltaAngle((double)angle, angle_ahead) <= 0))))
{
Hist[x] = 0;
}
else
{
Hist[x] = 1;
}
}
return(1);
}
/**
 * @brief set the motion commands
 * @param speed the desire speed, meter/s
 * @param turnrate the desire turn rate
 * @param actualSpeed the current speed, mm/s
 */
void VfhPlus::setMotion(double& linearX, int& turnrate, int const actualSpeed)
{
	int const mx = this->getMaxTurnrate(actualSpeed);
	/* this happens if all directions blocked, so just spin in place */
	if (DoubleCompare(linearX) <= 0) {
		turnrate = mx;
		linearX = 0;
	} else if ((pickedDirection > 270) && (pickedDirection < 360)) {
		turnrate = -mx;
	} else if ((pickedDirection < 270) && (pickedDirection > 180)) {
		turnrate = mx;
	} else {
		//turnrate = (int)rint(((double)(pickedDirection - 90) / 75.0) * mx);
		turnrate = ::rint(((this->pickedDirection - 90.0) / 45.0) * mx);
		if (::std::abs(turnrate) > mx) {
			turnrate = ::copysign(mx, turnrate);
		}
	}
}
/** @deprecated please use ConvertScan * 1e3 */
Eigen::Matrix<double, 361, 1>& VfhPlus::convertScanMM(
	std::vector<float> const ranges,
	double const angleMin,
	double const angleMax,
	double const angleIncrement,
	Eigen::Matrix<double, 361, 1>& result)
{
	for (int i = 0; i < 361; ++i) {
		result[i] = std::numeric_limits<double>::max();
	}
	ssize_t const n = ranges.size();
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
			if ((startIndex + step) >= n) {
				continue;
			}
			r = ranges[startIndex + step];
			if (std::isnan(r)) {
				continue;
			}
			r *= 1e3;
			/*if (DoubleCompare(r, 10.0) < 0) {
				r = rangeMax * 1e3;
			}*/
			result[i * 2] = r;
			result[i * 2 + 1] = r;
		}
	} else {
		for (unsigned i = 0; i < 180; ++i) {
			/* in case we are using sick */
			/* calculate position in laser frame */
			double const r = ranges[i] * 1e3;
			result[i * 2] = r;
			result[i * 2 + 1] = r;
		}
	}
	result[360] = result[359];
	return result;
}
}
#endif /* if !YUIWONGVFHIMPL_NOOLDVFPPLUSIMPL */
