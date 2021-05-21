#ifndef __AAP_H_
#define __AAP_H_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include <ros/ros.h>

namespace gazebo
{
  class GAZEBO_VISIBLE AutoActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
  public:
    AutoActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
  public:
    virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
  private:
    void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
  private:
    void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm (Edited by Bacchin Alberto).
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
  private:
    void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Helper function to compute a 2D perpendicula vector to a given one (Bacchin Alberto)
    /// \param[in] _pos The input vector
    /// \param[in] dir_pi_2 Direction of rotation: true= +pi/2, false= -pi/2
    /// \return The othogonal vector with the desidered verse
    ignition::math::Vector3d Perpendicular(ignition::math::Vector3d &_pos, bool dir_pi_2);

    /// \brief Helper function for an efficient intersection checking (Bacchin Alberto)
    /// \param[in] box The bounding box to be used, the z height will be ignored
    /// \param[in] origin Starting point from which the distances refer to
    /// \param[in] direction Direction of the interseption line
    /// \param[in] d_in_min Lower bound of the intersection segment
    /// \param[in] d_in_min Upper bound of the intersection segment
    /// \return An std::tuple containing a bool (true=collision detected) and a double which tells the distance of the nearest collision point on the direction line
    std::tuple<bool, double> checkIntersection(ignition::math::Box box,
                                               ignition::math::Vector3d origin, ignition::math::Vector3d direction, double d_in_min, double d_in_max);

    /// \brief Helper function for an efficient intersection checking (Bacchin Alberto)
    /// \param[in] A First point of the first line
    /// \param[in] B First point of the first line
    /// \param[in] C First point of the first line
    /// \param[in] D First point of the first line
    /// \return
    ignition::math::Vector2d Intersect(ignition::math::Vector2d A, ignition::math::Vector2d B,
                                       ignition::math::Vector2d C, ignition::math::Vector2d D);

    /// \brief Pointer to the parent actor.
  private:
    physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
  private:
    physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
  private:
    sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
  private:
    ignition::math::Vector3d velocity;
    double rot_velocity;

    /// \brief List of connections
  private:
    std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
  private:
    ignition::math::Vector3d target;

    /// \brief Current direction
  private:
    ignition::math::Vector3d direction;

    /// \brief Personalization of the room space where the actor moves (Bacchin Alberto)
  private:
    ignition::math::Vector2d room_x;
    /// \brief Personalization of the room space where the actor moves (Bacchin Alberto)
  private:
    ignition::math::Vector2d room_y;
    /// \brief Personalization of the room space where the actor moves (Bacchin Alberto)
  private:
    double room_z;

    /// \brief To broadcast velocity info (Bacchin Alberto)
  private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    /// \brief GridMap for obstacle avoidance
    //private:
    //grid_map::GridMap obs_map;

    /// \brief Target List
  private:
    std::vector<ignition::math::Vector3d> targets;

  private:
    int idx;

    /// \brief Target Tolerance
  private:
    double tolerance;

    /// \brief pose used to understand is the actor get stucked
  private:
    ignition::math::Vector3d check_pose;

    /// \brief distance used to understand is the actor get stucked
  private:
    int stuck_count = 0;

    /// \brief Target location weight (used for vector field)
  private:
    double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
  private:
    double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
  private:
    double animationFactor = 1.0;

    /// \brief Time of the last update.
  private:
    common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
  private:
    std::vector<std::string> ignoreModels;

    /// \brief List of other actor, used to solve the bug on actor bounding box
  private:
    std::vector<std::string> otherActors;

    /// \brief List of dynamic obstacles, used to deal with moving objects, namely apply a larger security box
  private:
    std::vector<std::string> dynObstacles;

    /// \brief Custom trajectory info.
  private:
    physics::TrajectoryInfoPtr trajectoryInfo;
  };
}

#endif
