/*
 * simulator.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

#include "extlibs/hrvo/hrvo_simulator.h"

#include <stdexcept>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/goal.h"
#include "extlibs/hrvo/kd_tree.h"

HRVOSimulator::HRVOSimulator()
    : defaults_(nullptr),
      kdTree_(nullptr),
      globalTime_(0.0),
      timeStep_(0.0),
      reachedGoals_(false)
{
    kdTree_ = new KdTree(this);
}

HRVOSimulator::HRVOSimulator(const World &world, double time_step)
    : timeStep_(time_step),
    globalTime_(0.0),
    reachedGoals_(false),
    kdTree_(nullptr)
{
    // Set current robot states
    const std::vector<Robot>& friendly_robots = world.friendlyTeam().getAllRobots();
    const std::vector<Robot>& enemy_robots = world.enemyTeam().getAllRobots();
    for (const Robot& robot : friendly_robots)
    {
        Vector2 position(robot.position().x(), robot.position().y());
        Vector2 velocity(robot.velocity().x(), robot.velocity().y());
        addAgent(robot.id(),
                 position,
                 addGoal(position),
                 robot.robotConstants().robot_max_speed_m_per_s,
                 friendly_robots.size(),
                 ROBOT_MAX_RADIUS_METERS,
                 0.03f,
                 robot.robotConstants().robot_max_speed_m_per_s,
                 robot.robotConstants().robot_max_speed_m_per_s,
                 0.0f,
                 robot.robotConstants().robot_max_acceleration_m_per_s_2,
                 velocity);
    }

    for (const Robot& robot : enemy_robots)
    {
        // TODO: determine the behavior of the enemy team
        Vector2 position(robot.position().x(), robot.position().y());
        Vector2 velocity(robot.velocity().x(), robot.velocity().y());
        addAgent(robot.id() + friendly_robots.size() ,
                 position,
                 addGoal(position),
                 robot.robotConstants().robot_max_speed_m_per_s,
                 friendly_robots.size(),
                 ROBOT_MAX_RADIUS_METERS,
                 0.03f,
                 robot.robotConstants().robot_max_speed_m_per_s,
                 robot.robotConstants().robot_max_speed_m_per_s,
                 0.0f,
                 robot.robotConstants().robot_max_acceleration_m_per_s_2,
                 velocity);
    }
}

HRVOSimulator::~HRVOSimulator()
{
    delete defaults_;
    defaults_ = nullptr;

    delete kdTree_;
    kdTree_ = nullptr;

    for (Agent* agent : agents_)
    {
        delete agent;
        agent = nullptr;
    }

    for (Goal* goal : goals_)
    {
        delete goal;
        goal = nullptr;
    }
}

std::size_t HRVOSimulator::addAgent(const Vector2 &position, std::size_t goalNo)
{
    if (defaults_ == nullptr)
    {
        throw std::runtime_error("Agent defaults not set when adding agent.");
    }

    Agent *const agent = new Agent(this, position, goalNo);
    agents_.push_back(agent);

    return agents_.size() - 1;
}

std::size_t
HRVOSimulator::addAgent(unsigned int robot_id, const Vector2 &position, std::size_t goalNo, float neighborDist,
                        std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
                        float uncertaintyOffset, float maxAccel, const Vector2 &velocity)
{
    Agent *const agent = new Agent(this, robot_id, position, goalNo, neighborDist, maxNeighbors,
                                   radius, velocity, maxAccel, goalRadius, prefSpeed,
                                   maxSpeed, uncertaintyOffset);
    agents_.push_back(agent);

    return agents_.size() - 1;
}

std::size_t HRVOSimulator::addGoal(const Vector2 &position)
{
    Goal *const goal = new Goal(position);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    Goal *const goal = new Goal(positions);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions,
                                        const std::vector<float> &speedAtPosition)
{
    Goal *const goal = new Goal(positions, speedAtPosition);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

void HRVOSimulator::doStep()
{
    if (kdTree_ == nullptr)
    {
        throw std::runtime_error(
                "HRVO Simulation not initialized when attempting to do step.");
    }

    reachedGoals_ = true;

    // Maybe should call update using the new timeStep so the robots are at the most up to
    // date position, and then compute the next velocities

    // Update robots position s given previous velocities and the time step since last
    // frame NOTE: Vel at first iteration will always be zero. Do we want to skip update
    // if globalTime == 0?
    for (Agent* agent : agents_)
    {
        agent->update();
    }
    globalTime_ += timeStep_;

    kdTree_->build();

    // Find next robots velocities
    // NOTE: We do not update the robot positions here as we do not know how long the
    //       next time step will be.
    for (Agent* agent : agents_)
    {
        agent->computePreferredVelocity();
        agent->computeNeighbors();
        agent->computeNewVelocity();
    }
}

std::size_t HRVOSimulator::getAgentGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->goalNo_;
}

float HRVOSimulator::getAgentGoalRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->goalRadius_;
}

float HRVOSimulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents_[agentNo]->maxAccel_;
}

std::size_t HRVOSimulator::getAgentMaxNeighbors(std::size_t agentNo) const
{
    return agents_[agentNo]->maxNeighbors_;
}

float HRVOSimulator::getAgentMaxSpeed(std::size_t agentNo) const
{
    return agents_[agentNo]->maxSpeed_;
}

float HRVOSimulator::getAgentNeighborDist(std::size_t agentNo) const
{
    return agents_[agentNo]->neighborDist_;
}

Vector2 HRVOSimulator::getAgentPosition(std::size_t agentNo) const
{
    return agents_[agentNo]->position_;
}

float HRVOSimulator::getAgentPrefSpeed(std::size_t agentNo) const
{
    return agents_[agentNo]->prefSpeed_;
}

float HRVOSimulator::getAgentRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->radius_;
}

bool HRVOSimulator::getAgentReachedGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->reachedGoal_;
}

float HRVOSimulator::getAgentUncertaintyOffset(std::size_t agentNo) const
{
    return agents_[agentNo]->uncertaintyOffset_;
}

Vector2 HRVOSimulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->velocity_;
}

Vector2 HRVOSimulator::getGoalPosition(std::size_t goalNo) const
{
    return goals_[goalNo]->position_;
}

void HRVOSimulator::setAgentDefaults(float neighborDist, std::size_t maxNeighbors,
                                 float radius, float goalRadius, float prefSpeed,
                                 float maxSpeed, float uncertaintyOffset, float maxAccel,
                                 const Vector2 &velocity)
{
    if (defaults_ == nullptr)
    {
        defaults_ = new Agent(this);
    }

    defaults_->goalRadius_        = goalRadius;
    defaults_->maxAccel_          = maxAccel;
    defaults_->maxNeighbors_      = maxNeighbors;
    defaults_->maxSpeed_          = maxSpeed;
    defaults_->neighborDist_      = neighborDist;
    defaults_->newVelocity_       = velocity;
    defaults_->uncertaintyOffset_ = uncertaintyOffset;
    defaults_->prefSpeed_         = prefSpeed;
    defaults_->radius_            = radius;
    defaults_->velocity_          = velocity;
}

void HRVOSimulator::setAgentGoal(std::size_t agentNo, std::size_t goalNo)
{
    agents_[agentNo]->goalNo_ = goalNo;
}

void HRVOSimulator::setAgentGoalPosition(std::size_t agentNo, Vector2 position)
{
    goals_[agentNo]->position_ = position;
}

void HRVOSimulator::setAgentGoalRadius(std::size_t agentNo, float goalRadius)
{
    agents_[agentNo]->goalRadius_ = goalRadius;
}

void HRVOSimulator::setAgentMaxAccel(std::size_t agentNo, float maxAccel)
{
    agents_[agentNo]->maxAccel_ = maxAccel;
}

void HRVOSimulator::setAgentMaxNeighbors(std::size_t agentNo, std::size_t maxNeighbors)
{
    agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}

void HRVOSimulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed)
{
    agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void HRVOSimulator::setAgentNeighborDist(std::size_t agentNo, float neighborDist)
{
    agents_[agentNo]->neighborDist_ = neighborDist;
}

void HRVOSimulator::setAgentPosition(std::size_t agentNo, const Vector2 &position)
{
    agents_[agentNo]->position_ = position;
}

void HRVOSimulator::setAgentPrefSpeed(std::size_t agentNo, float prefSpeed)
{
    agents_[agentNo]->prefSpeed_ = prefSpeed;
}

void HRVOSimulator::setAgentRadius(std::size_t agentNo, float radius)
{
    agents_[agentNo]->radius_ = radius;
}

void HRVOSimulator::setAgentUncertaintyOffset(std::size_t agentNo, float uncertaintyOffset)
{
    agents_[agentNo]->uncertaintyOffset_ = uncertaintyOffset;
}

void HRVOSimulator::setAgentVelocity(std::size_t agentNo, const Vector2 &velocity)
{
    agents_[agentNo]->velocity_ = velocity;
}

Vector2 HRVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->prefVelocity_;
}
