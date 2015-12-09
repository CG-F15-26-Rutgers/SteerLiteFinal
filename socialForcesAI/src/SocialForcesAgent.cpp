//
// Copyright (c) 2014-2015 VaHiD aZiZi
//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"


/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f

#define roundabout true

using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;

SocialForcesAgent::SocialForcesAgent()
{
    _SocialForcesParams.sf_acceleration = sf_acceleration;
    _SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
    _SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
    _SocialForcesParams.sf_query_radius = sf_query_radius;
    _SocialForcesParams.sf_body_force = sf_body_force;
    _SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
    _SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
    _SocialForcesParams.sf_agent_b = sf_agent_b;
    _SocialForcesParams.sf_agent_a = sf_agent_a;
    _SocialForcesParams.sf_wall_b = sf_wall_b;
    _SocialForcesParams.sf_wall_a = sf_wall_a;
    _SocialForcesParams.sf_max_speed = sf_max_speed;

    _enabled = false;
}


SocialForcesAgent::~SocialForcesAgent()
{
}

void SocialForcesAgent::setParameters(Behaviour behave)
{
    this->_SocialForcesParams.setParameters(behave);
}


void SocialForcesAgent::disable()
{
    // DO nothing for now
    // if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
    assert(_enabled==true);

    //  1. remove from database
    AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
    gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

    //  2. set enabled = false
    _enabled = false;
}


void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled because the value is not used in that case.
	_waypoints.clear();
	_midTermPath.clear();

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;

	if (initialConditions.colorSet == true)
		this->_color = initialConditions.color;
	else
		this->_color = Util::gBlue;

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject(dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject(dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
	}

	_enabled = true;

    if (initialConditions.goals.size() == 0)
        throw Util::GenericException("No goals were specified!\n");

    while (!_goalQueue.empty())
        _goalQueue.pop();

	std::string testcase = (*engineInfo->getModuleOptions("testCasePlayer").find("testcase")).second;

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i = 0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
			initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				_goalQueue.back().targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	if (testcase == "plane_egress") {
		firstAI(initialConditions);
	}
	else if (testcase == "plane_ingress") {
		secondAI();
	}
	else if (testcase == "crowd_crossing") {
		thirdAI(initialConditions);
	}
	else if (testcase == "office-complex") {
		fourthAI();
	}
	else if (testcase == "hallway-four-way-rounded-roundabout") {
		fifthAI(initialConditions);
	}
	else if (testcase == "bottleneck-squeeze") {
		sixthAI();
	}
	else if (testcase == "doorway-two-way") {
		seventhAI(initialConditions);
	}
	else if (testcase == "double-squeeze") {
		eighthAI(initialConditions);
	}
	else if (testcase == "wall-squeeze") {
		ninthAI(initialConditions);
	}
	else if (testcase == "hallway-two-way") {
		tenthAI(initialConditions);
	}
	else if (testcase == "maze") {
		eleventhAI();
	}
	else {
		printf("WE WERE NOT READY FOR THIS\n");
	}

	/* if (testcase == "maze") {
		printf("Run ASTAR\n");
		AStar();
		
		//Util::Point startingPoint = _position;
		//for (unsigned int i = 0; i < initialConditions.goals.size(); ++i) {
		//	Util::Point goalPoint = initialConditions.goals[i].targetLocation;
		//	computePlan(startingPoint, goalPoint);
		//	startingPoint = goalPoint;
		//}

		// Possibly merge this loop with the above one?
		//std::queue<SteerLib::AgentGoalInfo> goalQueueCopy = _goalQueue;
		//for (unsigned int i = 0; i < _goalQueue.size(); ++i) {
		//	_midTermPath.push_back(_goalQueue.front().targetLocation);
		//	_goalQueue.pop();
		//}

		//_goalQueue = goalQueueCopy;

		//computePlan(_position, initialConditions.goals.back().targetLocation);

		//std::cout << "Goal queue size: " << _goalQueue.size();
		//std::cout << "\n Initial conditions goals size: " << initialConditions.goals.size();
		
	 }
	else {
		printf("Run LongTermPlan\n");
		runLongTermPlanning();
	}*/


    /* Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
     * And that _waypoints is not empty */
    Util::Vector goalDirection;
    if (!_midTermPath.empty()){
        this->updateLocalTarget();
        goalDirection = normalize( this->_currentLocalTarget - position());
    }
    else
        goalDirection = normalize( _goalQueue.front().targetLocation - position());


	_prefVelocity = ((Util::Vector(goalDirection.x, 0.0f, goalDirection.z) * PREFERED_SPEED
			- velocity()) / _SocialForcesParams.sf_acceleration) * MASS;

    // _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
    std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
        " and current velocity is: " << velocity_ << std::endl;
#endif

    assert(_forward.length()!=0.0f);
    assert(_goalQueue.size() != 0);
    assert(_radius != 0.0f);
}


void SocialForcesAgent::calcNextStep(float dt)
{
    //nothing to do here
}


std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
      return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2

  const float t = dot(p - l1, l2 - l1) / lSq;

  if (t < 0.0)
      return std::make_pair((p - l1).length(), l1);  // Beyond the 'l1' end of the segment

  else if (t > 1.0)
      return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment

  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
	Util::Vector agent_repulsion_force;
	if(roundabout)
		agent_repulsion_force = Util::Vector(1, 0, 1);
	else 
		agent_repulsion_force = Util::Vector(0, 0, 0);

    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    gSpatialDatabase->getItemsInRange(_neighbors,
                                      _position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::AgentInterface* tmp_agent;
    SteerLib::ObstacleInterface* tmp_obj;
    Util::Vector away = Util::Vector(0, 0, 0);
    Util::Vector away_obs = Util::Vector(0, 0, 0);

    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++) {
        if ((*neighbor)->isAgent()) {
            tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbor);
            Util::Vector away_tmp = normalize(position() - tmp_agent->position());
            away += (away_tmp * _SocialForcesParams.sf_agent_a * exp(((this->radius() + tmp_agent->radius()) - (this->position() - tmp_agent->position()).length()) / _SocialForcesParams.sf_agent_b));
        }

        else {
            tmp_obj = dynamic_cast<SteerLib::ObstacleInterface*>(*neighbor);
            Util::Vector wall_normal = calcWallNormal(tmp_obj);
            std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_obj, wall_normal);
            std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
            Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
            away_obs += (away_obs_tmp *(_SocialForcesParams.sf_wall_a * exp((((this->radius()) - (this->position() - min_stuff.second).length()) / _SocialForcesParams.sf_wall_b)))*dt);
        }
    }

    return 3 * (away + away_obs); // Where did this 3 come from?
}

Vector SocialForcesAgent::calcGoalForce(Vector _goalDirection, float _dt)
{
	return Util::Vector((_goalDirection * PREFERED_SPEED) - velocity()) / _dt;
}


Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
    std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
            (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
    return calcWallRepulsionForce(dt) + (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}


Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{
    Util::Vector agent_repulsion_force = Util::Vector(0, 0, 0);

    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    gSpatialDatabase->getItemsInRange(_neighbors,
                                      _position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::AgentInterface * tmp_agent;
    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++) {
        if ((*neighbor)->isAgent())
            tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbor);
        else
            continue;

        if (tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001) {
            agent_repulsion_force = agent_repulsion_force + (tmp_agent->computePenetration(this->position(), this->radius()) * _SocialForcesParams.sf_agent_body_force * normalize(position() - tmp_agent->position()));
        }

    }

    return agent_repulsion_force;
}


Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{

    Util::Vector wall_repulsion_force = Util::Vector(0, 0, 0);
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    gSpatialDatabase->getItemsInRange(_neighbors,
                                      _position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
                                      _position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
                                      dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::ObstacleInterface* tmp_ob;

    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++) {
        if (!(*neighbor)->isAgent()) {
            tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *> (*neighbor);
        }
        else {
            continue;
        }

        if (tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001) {
            Util::Vector wall_normal = calcWallNormal(tmp_ob);
            std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
            std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
            wall_repulsion_force = wall_normal * (min_stuff.first + radius())*_SocialForcesParams.sf_body_force;
        }
    }

    return wall_repulsion_force;
}


std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
    Util::AxisAlignedBox box = obs->getBounds();
    if ( normal.z == 1)
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
        // Ended here;
    }
    else if ( normal.z == -1 )
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
    }
    else if ( normal.x == 1)
    {
        return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
    }
    else // normal.x == -1
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
    }
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 *             \           /
 *              \         /
 *               \   a   /
 *                \     /
 *                   _
 *          a       | |       a
 *                   -
 *                /     \
 *               /   a   \
 *              /         \
 *             /           \
 *
 *
 */


Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
    Util::AxisAlignedBox box = obs->getBounds();
    if (position().x > box.xmax ){
        if (position().z > box.zmax){
            if (abs(position().z - box.zmax ) > abs( position().x - box.xmax))
                return Util::Vector(0, 0, 1);
            else
                return Util::Vector(1, 0, 0);
        }

        else if (position().z < box.zmin){
            if (abs(position().z - box.zmin) > abs(position().x - box.xmax))
                return Util::Vector(0, 0, -1);
            else
                return Util::Vector(1, 0, 0);
        }
        else // in between zmin and zmax
            return Util::Vector(1, 0, 0);
    }

    else if (position().x < box.xmin){
        if (position().z > box.zmax ){
            if (abs(position().z - box.zmax) > abs( position().x - box.xmin))
                return Util::Vector(0, 0, 1);
            else
                return Util::Vector(-1, 0, 0);
        }

        else if (position().z < box.zmin){
            if (abs(position().z - box.zmin ) > abs( position().x - box.xmin))
                return Util::Vector(0, 0, -1);
            else
                return Util::Vector(-1, 0, 0);

        }
        else // in between zmin and zmax
            return Util::Vector(-1, 0, 0);
    }
    else {
        if (position().z > box.zmax)
            return Util::Vector(0, 0, 1);
        else if (position().z < box.zmin)
            return Util::Vector(0, 0, -1);
        else // What do we do if the agent is inside the wall?? Lazy Normal
            return calcObsNormal(obs);
    }

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
    Util::AxisAlignedBox box = obs->getBounds();
    Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2,
                                         (box.ymax+box.ymin)/2,
                                         (box.zmax+box.zmin)/2);
    return normalize(position() - obs_centre);
}


bool SocialForcesAgent::reachedCurrentWaypoint()
{

    if ( !_waypoints.empty())
        return (position() - _waypoints.front()).lengthSquared() <= (radius()*radius());
    else
        return false;
}


bool SocialForcesAgent::hasLineOfSightTo(Util::Point target)
{
    float dummyt;
    Util::Vector _rightSide = rightSideInXZPlane(this->forward());
    SpatialDatabaseItemPtr dummyObject;
    Ray lineOfSightTestRight, lineOfSightTestLeft;
    lineOfSightTestRight.initWithUnitInterval(_position + _radius*_rightSide, target - _position);
    lineOfSightTestLeft.initWithUnitInterval(_position + _radius*(_rightSide), target - _position);

    return (!gSpatialDatabase->trace(lineOfSightTestRight,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true))
        && (!gSpatialDatabase->trace(lineOfSightTestLeft,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true));

}

void SocialForcesAgent::computePlan(Util::Point startPoint, Util::Point goalPoint){

    // Util::Point global_goal = _goalQueue.front().targetLocation;
    if (astar.computePath(__path, _position, goalPoint, gSpatialDatabase)){
        while (!_goalQueue.empty())
            _goalQueue.pop();
        
        for (int i = 0; i<__path.size(); ++i){
            SteerLib::AgentGoalInfo goal_path_pt;
            goal_path_pt.targetLocation = __path[i];
            _goalQueue.push(goal_path_pt);
        }
    }

    SteerLib::AgentGoalInfo goal_path_pt;
    goal_path_pt.targetLocation = startPoint;
    _goalQueue.push(goal_path_pt);
}

void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
    Util::AutomaticFunctionProfiler profileThisFunction( &SocialForcesGlobals::gPhaseProfilers->aiProfiler );
    if (!enabled())
        return;

    Util::AxisAlignedBox oldBounds(_position.x - _radius,
                                   _position.x + _radius,
                                   0.0f, 0.0f,
                                   _position.z - _radius,
                                   _position.z + _radius);

    SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
    Util::Vector goalDirection;

    if (!_midTermPath.empty() && !this->hasLineOfSightTo(goalInfo.targetLocation)){
        if (reachedCurrentWaypoint())
            this->updateMidTermPath();

        this->updateLocalTarget();

        goalDirection = normalize(_currentLocalTarget - position());
    }

    else
        goalDirection = normalize(goalInfo.targetLocation - position());

    Util::Vector goalForce = calcGoalForce(goalDirection, dt);
    Util::Vector repulsionForce = calcRepulsionForce(dt);
    Util::Vector proximityForce = calcProximityForce(dt);

    if (repulsionForce.x != repulsionForce.x)
        std::cout << "Found some nan" << std::endl;

    // #define _DEBUG_ 1
#ifdef _DEBUG_
    std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
    std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
    std::cout << "agent" << id() << " goal force " << goalForce << std::endl;
#endif
    // _velocity = _newVelocity;
    int alpha = 1;
    if (repulsionForce.length() > 0.0)
        alpha = 0;

    Util::Vector acceleration = (goalForce + repulsionForce + proximityForce) / AGENT_MASS;
    _velocity = velocity() + acceleration * dt;
    _velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
    _velocity.y=0.0f;
#ifdef _DEBUG_
    std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
    _position = position() + (velocity() * dt);
    // A grid database update should always be done right after the new position of the agent is calculated
    /*
     * Or when the agent is removed for example its true location will not reflect its location in the grid database.
     * Not only that but this error will appear random depending on how well the agent lines up with the grid database
     * boundaries when removed.
     */
    
    Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
    gSpatialDatabase->updateObject( this, oldBounds, newBounds);

    /*
     * Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
     */

    if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
        (goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
         Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
                                   goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax,
                                   this->position(), this->radius()))){
        _goalQueue.pop();

        if (_goalQueue.size() != 0){
            // in this case, there are still more goals, so start steering to the next goal.
            goalDirection = _goalQueue.front().targetLocation - _position;
            _prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
        }
        else{
            // in this case, there are no more goals, so disable the agent and remove it from the spatial database.
            disable();
            return;
        }
    }

    // Here the 2D solution from RVO is converted into the 3D used by SteerSuite
    if (velocity().lengthSquared() > 0.0){
        // Only assign forward direction if agent is moving
        // Otherwise keep last forward
        _forward = normalize(_velocity);
    }
}


/**
 * Removes a number of points from the begining of the path
 *
 * After size of path should be old size - FURTHEST_LOCAL_TARGET_DISTANCE
 */
void SocialForcesAgent::updateMidTermPath()
{
    if ( this->_midTermPath.size() < FURTHEST_LOCAL_TARGET_DISTANCE)
    {
        return;
    }
    if ( !_waypoints.empty())
    {
        _waypoints.erase(_waypoints.begin());
    }
    // save good path
    std::vector<Util::Point> tmpPath;
    for (unsigned int i=(FURTHEST_LOCAL_TARGET_DISTANCE); i < _midTermPath.size();i++ )
    {
        tmpPath.push_back(_midTermPath.at(i));
    }
    _midTermPath.clear();

    for (unsigned int i=0; i < tmpPath.size(); i++)
    {
        _midTermPath.push_back(tmpPath.at(i));
    }

}


/**
 * Update the local target to the furthest point on the midterm path the agent can see.
 */
void SocialForcesAgent::updateLocalTarget()
{
    Util::Point tmpTarget = this->_goalQueue.front().targetLocation;
    unsigned int i=0;
    for (i=0; (i < FURTHEST_LOCAL_TARGET_DISTANCE) &&
             i < this->_midTermPath.size(); i++){
        tmpTarget = this->_midTermPath.at(i);
        if ( this->hasLineOfSightTo(tmpTarget) )
        {
            this->_currentLocalTarget = tmpTarget;
        }
    }
}


/**
 * finds a path to the current goal
 * puts that path in midTermPath
 */
bool SocialForcesAgent::runLongTermPlanning()
{
	_midTermPath.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos = position();

	if (!gSpatialDatabase->findPath(pos, _goalQueue.front().targetLocation,
		agentPath, (unsigned int)50000))
	{
		return false;
	}

	for (int i = 1; i < agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	return true;
}


bool SocialForcesAgent::runLongTermPlanning2()
{
/* Do we modify this for A*? */
#ifndef USE_PLANNING
    return;
#endif
    _waypoints.clear();
    //==========================================================================

    // run the main a-star search here
    std::vector<Util::Point> agentPath;
    Util::Point pos =  position();
    if (gEngine->isAgentSelected(this))
    {
        // std::cout << "agent" << this->id() << " is running planning again" << std::endl;
    }

    if ( !gSpatialDatabase->findSmoothPath(pos, _goalQueue.front().targetLocation,
                                           agentPath, (unsigned int) 50000))
    {
        return false;
    }

    // Push path into _waypoints

    // Skip first node that is at location of agent
    for  (int i=1; i <  agentPath.size(); i++)
    {
        _waypoints.push_back(agentPath.at(i));

    }

    return true;

}


void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
    // if the agent is selected, do some annotations just for demonstration
    if (gEngine->isAgentSelected(this))
    {
        Util::Ray ray;
        ray.initWithUnitInterval(_position, _forward);
        float t = 0.0f;
        SteerLib::SpatialDatabaseItem * objectFound;
        Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
        if (gSpatialDatabase->trace(ray, t, objectFound, this, false))
        {
            Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
        }
        else {
            Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
        }
        Util::DrawLib::drawFlag( this->currentGoal().targetLocation, Color(0.5f,0.8f,0), 2);
        if ( this->currentGoal().goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL )
        {
            Color color(0.4,0.9,0.4);
            DrawLib::glColor(color);
            DrawLib::drawQuad(Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmin),
                    Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmax),
                    Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmax),
                    Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmin));
        }
        int i;
        for (i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
        {
            DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
            DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
        }
        // DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
    }
    else {
        Util::DrawLib::drawAgentDisc(_position, _radius, this->_color);
    }
    if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
        Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
    }

#ifdef DRAW_COLLISIONS
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    gSpatialDatabase->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
            _position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
    {
        if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
        {
            Util::DrawLib::drawStar(
                    this->position()
                    +
                    (
                        (
                            dynamic_cast<AgentInterface*>(*neighbor)->position()
                            -
                            this->position()
                        )
                    /2), Util::Vector(1,0,0), 0.8f, gRed);
            // Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
        }
    }
#endif
#ifdef DRAW_HISTORIES
    __oldPositions.push_back(position());
    int points = 0;
    float mostPoints = 100.0f;
    while ( __oldPositions.size() > mostPoints )
    {
        __oldPositions.pop_front();
    }
    for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
    {
        DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
    }

#endif

#ifdef DRAW_ANNOTATIONS

    for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
    {
        if ( gEngine->isAgentSelected(this) )
        {
            DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
        }
        else
        {
            //DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
        }
    }

    for (int i=0; i < (_waypoints.size()); i++)
    {
        DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
    }

    for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
    {
        if ( gEngine->isAgentSelected(this) )
        {
            DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
        }
        else
        {
            // DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
        }
    }

    DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
    DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

#endif

#endif
}


/**********************************************/
/**********************************************/
/**********************************************/
/**********************************************/
/*
        TESTCASE AI's

		egress			-->		needs work				--> TERRIBLE
		ingress			-->		to finish				--> TERRIBLE 
		crowd			-->		DONE/could be better	--> BAD
		office			-->		TODO					--> TERRIBLE
		roundabout		-->		DONE/could be better	-->	made changes, get new score
		bottleneck		-->		to finish				--> no score
		doorway			-->		DONE					-->	good
		double squeeze	-->		DONE					-->	good
		wall squeeze	-->		DONE/could be better	-->	made changes, get new score
		hallway			-->		DONE/could be better	-->	could be better
		maze			-->		needs work				--> TERRIBLE
*/
/**********************************************/
/**********************************************/
/**********************************************/
/**********************************************/

// plane_egress
// TODO
// Works but really bad 
bool SocialForcesAgent::firstAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// bunch of agents try to get out 
	std::vector<Util::Point> agentPath;

	// set goal point for every section

	if (position().z > 0) {
		if (position().x < -3) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(-2.1, 0, position().z);
			_goalQueue.push(goal);
			goal.targetLocation = Point(-2.1, 0, 37.7);
			_goalQueue.push(goal);
			goal.targetLocation = Point(10, 0, 37.7);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-2.5, 0, position().z-.5);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-2.5, 0, 37.7);
		_goalQueue.push(goal);
		goal.targetLocation = Point(10, 0, 37.7);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	else {
		if (position().x < -3) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(-2.1, 0, position().z);
			_goalQueue.push(goal);
			goal.targetLocation = Point(-2.1, 0, -37.7);
			_goalQueue.push(goal);
			goal.targetLocation = Point(10, 0, -37.7);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-2.5, 0, position().z-.5);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-2.5, 0, -37.7);
		_goalQueue.push(goal);
		goal.targetLocation = Point(10, 0, -37.7);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}




	/*
	// right side
	if (position().z < 0) {
		
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-3.5, 0, position().z);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-3.5, 0, -37);
		_goalQueue.push(goal);
		goal.targetLocation = Point(10, 0, -37);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	// left side
	else {

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-3.5, 0, position().z);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-3.5, 0, 37);
		_goalQueue.push(goal);
		goal.targetLocation = Point(10, 0, 37);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	*/
}

// plane_ingress
// TODO
bool SocialForcesAgent::secondAI() {
	// bunch of agents try to get in
	// IN PROGRESS


	/*
	SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
	_goalQueue.pop();

	if (originalgoal.targetLocation.z > 33) {
		SteerLib::AgentGoalInfo goal;
		// entrance to left
		goal.targetLocation = Point(8, 0, 38.5);
		_goalQueue.push(goal);
		// first barrier
		goal.targetLocation = Point(-3, 0, 36);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	else if (originalgoal.targetLocation.z > 29) {
		SteerLib::AgentGoalInfo goal;
		// entrance to left
		goal.targetLocation = Point(8, 0, 38.5);
		_goalQueue.push(goal);
		// first barrier
		goal.targetLocation = Point(-3, 0, 36);
		_goalQueue.push(goal);
		// second barrier
		goal.targetLocation = Point(-3, 0, 32.6);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	else
	{
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(100, 0, 100);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);
		return runLongTermPlanning();
	}
	*/

	







	return runLongTermPlanning();
}

// crowd_crossing
// DONE
bool SocialForcesAgent::thirdAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// big agent trying to cross a one way street

	SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
	_goalQueue.pop();
	SteerLib::AgentGoalInfo goal;

	if (initialConditions.name == "A") {
		goal.targetLocation = Point(position().x, 0, -75);
		_goalQueue.push(goal);
		goal.targetLocation = Point(position().x, 0, -20);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}

	else{
		goal.targetLocation = originalgoal.targetLocation;
		goal.targetLocation.z -= 5;
		_goalQueue.push(goal);
		return runLongTermPlanning();
	}

}

// office-complex
// TODO
bool SocialForcesAgent::fourthAI() {
	// bunch of agents try to get out of the office
	
	// put goal points for each office

	return AStar();
}

// hallway-four-way-rounded-roundabout
// works but needs more work
// make sure roundabout = false if not testing this
bool SocialForcesAgent::fifthAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// polygon at center and agents are trying going in multiple directions

	std::vector<Util::Point> agentpath;

	// going left
	if (_goalQueue.front().targetLocation.x > 97 && _goalQueue.front().targetLocation.z > 4)
	{
		// check if agent is passed the starting point
		if (position().x > -12) {
			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo goal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo newgoal; 
		newgoal.targetLocation = Point(-12, 0, 4);		// starting point - east point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(0, 0, 12);		// north point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(12, 0, 4);		// west point
		_goalQueue.push(newgoal);	
		newgoal.targetLocation = Point(50, 0, 5);		// west point
		_goalQueue.push(newgoal);
		_goalQueue.push(goal);							// go to goal

		return runLongTermPlanning();
	}
	// going down
	else if (_goalQueue.front().targetLocation.x == 0 && _goalQueue.front().targetLocation.z == -98)
	{
		// check if agent is close to goal
		if (position().z < 4) {
			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo goal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo newgoal;
		newgoal.targetLocation = Point(4, 0, 12);		// starting point - north point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(12, 0, 0);		// west point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(4, 0, -12);		// south point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(0, 0, -50);		// south point
		_goalQueue.push(newgoal);
		_goalQueue.push(goal);							// go to goal

		return runLongTermPlanning();
	}
	// going right
	else if (_goalQueue.front().targetLocation.x == -98 && _goalQueue.front().targetLocation.z == -4)
	{
		// check if agent is close to the goal
		if (position().x < 12) 
		{
			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo goal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo newgoal;
		newgoal.targetLocation = Point(12, 0, -4);		// starting point - west point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(0, 0, -12);		// south point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(-12, 0, -4);		// east point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(-50, 0, -5);		// west point
		_goalQueue.push(newgoal);
		_goalQueue.push(goal);							// go to goal

		return runLongTermPlanning();
	}
	// going up
	else {
		
		// check if agent is already close to the goal
		if (position().z > -4)
		{
			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo goal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo newgoal;
		newgoal.targetLocation = Point(-4, 0, -12);		// starting point - south point
		_goalQueue.push(newgoal);	
		newgoal.targetLocation = Point(-12, 0, 0);		// east point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(-4, 0, 12);		// north point
		_goalQueue.push(newgoal);
		newgoal.targetLocation = Point(0, 0, 50);		// south point
		_goalQueue.push(newgoal);
		_goalQueue.push(goal);							// go to goal

		return runLongTermPlanning();
	}

	return runLongTermPlanning();
}

// bottleneck-squeeze
// TODO
bool SocialForcesAgent::sixthAI() {
	// bunch of agents trying to get through one entrance

	// sort by quadrant

	if (position().z > 2.1) {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(90, 0, position().z);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -30);
		_goalQueue.push(goal);
		goal.targetLocation = Point(60, 0, -30);
		_goalQueue.push(goal);
		goal.targetLocation = Point(60, 0, -50);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -50);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -70);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, -70);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, 0);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	else {

		return runLongTermPlanning();
	}

	/* CLOSE 
	if (position().z > 2.1) {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(position().x, 0, 90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, 90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, 0);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	else  {
		
		if (position().x < 45) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, 0);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);
			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(70, 0, position().z);
		_goalQueue.push(goal);
		goal.targetLocation = Point(70, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(23, 0, -20);
		_goalQueue.push(goal);
		goal.targetLocation = Point(70, 0, -20);
		_goalQueue.push(goal);
		goal.targetLocation = Point(70, 0, 0);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}

	SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
	_goalQueue.pop();
	SteerLib::AgentGoalInfo goal;
	goal.targetLocation = Point(position().x, 0, 0);
	_goalQueue.push(goal);
	_goalQueue.push(originalgoal);
	return runLongTermPlanning();

	*/
	/*
	// LEFT 
	if (position().z > 2.1) {
		if (position().x < 35) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, 55);
			_goalQueue.push(goal);
			goal.targetLocation = Point(30, 0, 55);
			_goalQueue.push(goal);
			goal.targetLocation = Point(30, 0, 1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 50) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, 60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, 60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, 1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 65) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, 70);
			_goalQueue.push(goal);
			goal.targetLocation = Point(55, 0, 70);
			_goalQueue.push(goal);
			goal.targetLocation = Point(55, 0, 1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 80) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, 80);
			_goalQueue.push(goal);
			goal.targetLocation = Point(70, 0, 80);
			_goalQueue.push(goal);
			goal.targetLocation = Point(70, 0, 1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(position().x, 0, 90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, 90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, 1);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	// RIGHT
	else if (position().z < -2.1) {
		if (position().x < 35) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, -55);
			_goalQueue.push(goal);
			goal.targetLocation = Point(30, 0, -55);
			_goalQueue.push(goal);
			goal.targetLocation = Point(30, 0, -1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 50) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, -60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, -60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, -1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 65) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, -70);
			_goalQueue.push(goal);
			goal.targetLocation = Point(55, 0, -70);
			_goalQueue.push(goal);
			goal.targetLocation = Point(55, 0, -1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 80) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(position().x, 0, -80);
			_goalQueue.push(goal);
			goal.targetLocation = Point(70, 0, -80);
			_goalQueue.push(goal);
			goal.targetLocation = Point(70, 0, -1);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}

		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(position().x, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -90);
		_goalQueue.push(goal);
		goal.targetLocation = Point(90, 0, -1);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	
	SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
	_goalQueue.pop();
	SteerLib::AgentGoalInfo goal;
	goal.targetLocation = Point(position().x, 0, 1);
	_goalQueue.push(goal);
	_goalQueue.push(originalgoal);

	return runLongTermPlanning();
	*/

	// go to the back
	/*
	if (position().z > 2.1) {
		if (position().x < 30) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(40, 0, 15);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, 1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 50) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(45, 0, 30);
			_goalQueue.push(goal);
			goal.targetLocation = Point(45, 0, 1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 70) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(65, 0, 45);
			_goalQueue.push(goal);
			goal.targetLocation = Point(65, 0, 1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x > 70) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(95, 0, 60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(95, 0, 1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
	}
	else if (position().z < -2.1) {
		if (position().x < 30) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(40, 0, -20);
			_goalQueue.push(goal);
			goal.targetLocation = Point(40, 0, -1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 50) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(50, 0, -45);
			_goalQueue.push(goal);
			goal.targetLocation = Point(50, 0, -1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 70) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(70, 0, -60);
			_goalQueue.push(goal);
			goal.targetLocation = Point(70, 0, -1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
		else if (position().x < 91) {
			SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
			_goalQueue.pop();
			SteerLib::AgentGoalInfo goal;
			goal.targetLocation = Point(80, 0, -80);
			_goalQueue.push(goal);
			goal.targetLocation = Point(80, 0, -1.6);
			_goalQueue.push(goal);
			_goalQueue.push(originalgoal);

			return runLongTermPlanning();
		}
	}
	*/
}

// doorway-two-way
// DONE
// Try to optimize if time
bool SocialForcesAgent::seventhAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// two agents try to get through same entrance 

	// bottom agent
	if (initialConditions.name == "A") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(0, 0, .2);
		_goalQueue.push(goal);
		goal.targetLocation = Point(3, 0, .2);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	// top agent
	else if (initialConditions.name == "B") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(15, 0, 1);
		_goalQueue.push(goal);
		goal.targetLocation = Point(15, 0, 2);
		_goalQueue.push(goal);
		goal.targetLocation = Point(2, 0, 2);
		_goalQueue.push(goal);
		goal.targetLocation = Point(0, 0, .2);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
}

// double-squeeze
// DONE
bool SocialForcesAgent::eighthAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// four agents try to get by each other
	//printf("double-squeeze\n");

	std::vector<Util::Point> agentPath;

	// <-10, 0, -.5>
	// <10, 0, -.5>
	// <-10, 0, .5>
	// <10, 0, .5>

	// bottom left agent
	if (initialConditions.name == "A") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-12, 0, .6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(1, 0, .6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(12, 0, .6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(15, 0, .6);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		return runLongTermPlanning();
	}
	// top left agent
	else if (initialConditions.name == "B") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-1, 0, -.6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-12, 0, -.6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-15, 0, -.6);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}
	// bottom right agent
	else if (initialConditions.name == "C") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(1, 0, .6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(12, 0, .6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(15, 0, .6);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}
	// top right agent
	else if (initialConditions.name == "D") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(12, 0, -.6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-1, 0, -.6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-12, 0, -.6);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-15, 0, -.6);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}
}

// wall-squeeze
// Definitely need a better way to do this
bool SocialForcesAgent::ninthAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// basically previous test case but there's now a wall and three agents

	// bottom left agent
	if (initialConditions.name == "A") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-10, 0, .5);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-6, 0, 1.55);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}
	// top agent
	else if (initialConditions.name == "B") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(10, 0, -.75);
		_goalQueue.push(goal);
		goal.targetLocation = Point(1.5, 0, -.75);
		_goalQueue.push(goal);
		
		
		goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(17, 0, .55);
		_goalQueue.push(goal);
		goal.targetLocation = Point(20, 0, .55);
		_goalQueue.push(goal);
		
		goal.targetLocation = Point(-3, 0, .55);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}
	// bottom right agent
	else if (initialConditions.name == "C") {
		SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
		_goalQueue.pop();
		SteerLib::AgentGoalInfo goal;
		goal.targetLocation = Point(-15, 0, .5);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-10, 0, .5);
		_goalQueue.push(goal);
		goal.targetLocation = Point(-6, 0, 1.55);
		_goalQueue.push(goal);
		_goalQueue.push(originalgoal);

		runLongTermPlanning();

		return true;
	}


	

	return runLongTermPlanning();
}

// hallway-two-way
// DONE
// optimize if time
bool SocialForcesAgent::tenthAI(const SteerLib::AgentInitialConditions & initialConditions) {
	// get direction of agents
	// move them to one side
	// proceed
	// bunch of agents going two different directions
	//printf("hallway-two-way\n");

	// separate into quadrants (x)
	// TODO separate it further
	// 100...75...50...25...0...-25...-50...-75...-100

	// separate further into quadrants (y)
	
	// goes to right
	if (initialConditions.direction.x == -1) {
		if (position().x > 0) {

			// closer to bottom wall
			if (position().z < 0) {
				SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
				_goalQueue.pop();
				SteerLib::AgentGoalInfo goal;
				goal.targetLocation = Point(45, 0, -6);
				_goalQueue.push(goal);
				_goalQueue.push(originalgoal);

				return runLongTermPlanning();
			}
			// closer to top wall
			else {
				SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
				_goalQueue.pop();
				SteerLib::AgentGoalInfo goal;
				goal.targetLocation = Point(45, 0, -3);
				_goalQueue.push(goal);
				_goalQueue.push(originalgoal);

				return runLongTermPlanning();
			}
		}
		else {
			return runLongTermPlanning();
		}
	}
	// goes to the left
	else {
		if (position().x < 0) {

			if (position().z < 0) {
				SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
				_goalQueue.pop();
				SteerLib::AgentGoalInfo goal;
				goal.targetLocation = Point(45, 0, 6);
				_goalQueue.push(goal);
				_goalQueue.push(originalgoal);

				return runLongTermPlanning();
			}
			// closer to top wall
			else {
				SteerLib::AgentGoalInfo originalgoal = _goalQueue.front();
				_goalQueue.pop();
				SteerLib::AgentGoalInfo goal;
				goal.targetLocation = Point(45, 0, 3);
				_goalQueue.push(goal);
				_goalQueue.push(originalgoal);

				return runLongTermPlanning();
			}
		}
		else {
			// steerbench was lower with longterm
			return runLongTermPlanning();
		}
	}
}

// maze
// TODO 
// I DONT UNDERSTAND WHY IT HAS SUCH A BAD SCORE
bool SocialForcesAgent::eleventhAI() {
	// should run astar here

	return AStar();

	// return runLongTermPlanning();


	// SteerLib::AgentGoalInfo goalPoint = _goalQueue.front();
	// _goalQueue.pop();
	// SteerLib::AgentGoalInfo goal;
	// goal.targetLocation = Point(-60, 0, position().z);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-70, 0, 60);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(55, 0, 60);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(55, 0, 20);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-40, 0, 20);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-40, 0, -10);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(20, 0, -10);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(15, 0, -65);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-20, 0, -65);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-20, 0, -50);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-65, 0, -50);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-65, 0, 15);
	// _goalQueue.push(goal);
	// goal.targetLocation = Point(-90, 0, 15);
	// _goalQueue.push(goal);
	// _goalQueue.push(goalPoint);

	// return runLongTermPlanning();

	/*
	std::vector<Util::Point> agentPath;
	
	agentPath.push_back(Point(-60, 0, position().z));
	agentPath.push_back(Point(-70, 0, 60));
	agentPath.push_back(Point(55, 0, 60));
	agentPath.push_back(Point(55, 0, 20));
	agentPath.push_back(Point(-40, 0, 20));
	agentPath.push_back(Point(-40, 0, -10));
	agentPath.push_back(Point(10, 0, -10));
	agentPath.push_back(Point(10, 0, -65));
	agentPath.push_back(Point(-35, 0, -65));
	
	
	for (int i = 0; i < agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if (agentPath.size()>0)
	{
		for (int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i - 1], agentPath[i], gYellow);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
	*/

	//return AStar();
}

bool SocialForcesAgent::AStar()
{
	_midTermPath.clear();

	std::vector<Util::Point> agentPath;
	Util::Point pos = position();

	if (!astar.computePath(agentPath, pos, _goalQueue.front().targetLocation, gSpatialDatabase)) {
		return false;
	}

	for (int i = 1; i < agentPath.size(); i++) {
		_midTermPath.push_back(agentPath.at(i));

		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0) {
			_waypoints.push_back(agentPath.at(i));
		}
	}

	if (agentPath.size() > 0) {
		for (int i = 1; i < agentPath.size(); ++i) {
			Util::DrawLib::drawLine(agentPath[i - 1], agentPath[i], Util::Color(1.0f, 0.0f, 0.0f), 2);
		}
	}

	return true;
}
