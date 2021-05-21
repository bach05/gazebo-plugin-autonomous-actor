#include "autonomous_actor/AutonomousActorPlugin.hh"

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include <iostream>

#include <ros/ros.h>
#include <cmath>
#include <ctgmath>
#include <fstream>
#include <string>

#include "gazebo_person_detection/actor_vel.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AutoActorPlugin)

#define WALKING_ANIMATION "walking"
#define SPEED_BASE 0.8

std::ofstream myfile;

/////////////////////////////////////////////////
AutoActorPlugin::AutoActorPlugin()
{
}

/////////////////////////////////////////////////
void AutoActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&AutoActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Added by brucechanjianle
  // Read in multiple targets
  if (_sdf->HasElement("targets"))
  {
    // Obtain targets with element pointer
    sdf::ElementPtr local_targets = _sdf->GetElement("targets")->GetElement("target");

    // Extract target
    while (local_targets)
    {
      this->targets.push_back(local_targets->Get<ignition::math::Vector3d>());
      local_targets = local_targets->GetNextElement("target");
    }

    // Set index to zero
    this->idx = 0;
  }

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Read in the other obstacles to ignore (Bacchin Alberto)
  if (_sdf->HasElement("actor_awareness"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("actor_awareness")->GetElement("model");
    while (modelElem)
    {
      this->otherActors.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Moving obstacles will be treated with larger security bounds (Bacchin Alberto)
  if (_sdf->HasElement("dynamic_obstacles"))
  {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("dynamic_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->dynObstacles.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Added by brucechanjianle
  // Read in target tolerance
  if (_sdf->HasElement("target_tolerance"))
    this->tolerance = _sdf->Get<double>("target_tolerance");
  else
    this->tolerance = 0.3;

  // Added by Alberto Bacchin
  // Room personalization
  if (_sdf->HasElement("room_x"))
    this->room_x = _sdf->Get<ignition::math::Vector2d>("room_x");
  else
    this->room_x.Set(100, 100);

  if (_sdf->HasElement("room_y"))
    this->room_y = _sdf->Get<ignition::math::Vector2d>("room_y");
  else
    this->room_y.Set(100, 100);

  if (_sdf->HasElement("room_z"))
    this->room_z = _sdf->Get<double>("room_z");
  else
    this->room_z = 0;

  //Initialize the obstacle map
  ignition::math::Vector3d init_pose;
  if (_sdf->HasElement("init_pose"))
    init_pose = _sdf->Get<ignition::math::Vector3d>("init_pose");
  else
    init_pose = {0, 0, 0};

  check_pose = init_pose;

  //Publisher initialization
  pub = nh.advertise<gazebo_person_detection::actor_vel>("actor_velocities", 1, true);

  direction = (this->target - init_pose).Normalize();
}

/////////////////////////////////////////////////
void AutoActorPlugin::Reset()
{
  this->velocity = SPEED_BASE; //[m/s]
  //Add rotational velocity for gracefully rotations
  this->rot_velocity = this->velocity.Length() * 4.375; //[rad/s]
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, room_z);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::ChooseNewTarget()
{
  //ROS_INFO("+++++++ CHOOSE NEW TARGET START +++++++++");
  //Dilation of the bounding box of the model to improve the awareness
  ignition::math::Box security_box, Bbox;
  ignition::math::Vector3d scaling(0.3, 0.3, 0);
  ignition::math::Vector3d scaling_actor(0.5, 0.5, 1.2138);
  ignition::math::Vector3d targetProjection, other_actor_pose;

  ignition::math::Vector3d newTarget(this->target);

#ifdef DEBUG_
  // For debug
  gzdbg << "++++++++++++ NEW TARGET SELECTION ++++++++++++" << std::endl;
#endif

  //choose a target that is far enough from the previuos one
  while ((newTarget - this->target).Length() < 2.0)
  {

    //debug std::cout << "New target: " << newTarget << " too near to the previous one!" << std::endl;
#ifdef DEBUG_
    // For debug
    gzdbg << "New target: " << newTarget << " too near to the previous one!" << std::endl;
#endif

    newTarget.X(ignition::math::Rand::DblUniform(this->room_x[0], this->room_x[1]));
    newTarget.Y(ignition::math::Rand::DblUniform(this->room_y[0], this->room_y[1]));

    //debug std::cout << "NEW RANDOM TARGET: " << newTarget << std::endl;

    if (newTarget.X() == 2.0 || newTarget.Y() == 2.0)
    {

      std::cout << "ATTENTION: Not valid target for this training/test..." << std::endl;
      continue;
    }

#ifdef DEBUG_
    // For debug
    gzdbg << "NEW RANDOM TARGET: " << newTarget << std::endl;
#endif

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {

      physics::ModelPtr model = this->world->ModelByIndex(i);
      if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                    model->GetName()) == this->ignoreModels.end())
      {
        //Actors bounding box handling (the method BoundingBox() dose not work properly for actors)
        if (std::find(this->otherActors.begin(), this->otherActors.end(),
                      model->GetName()) == this->otherActors.end())
        {
          Bbox = model->BoundingBox();
        }
        else
        {
          other_actor_pose = model->WorldPose().Pos();
          //debug std::cout << ">>>> other actor pose=" << other_actor_pose << std::endl;
          Bbox = ignition::math::Box(other_actor_pose - scaling_actor, other_actor_pose + scaling_actor);
          //debug std::cout << ">>>> BBOX=" << Bbox << std::endl;
        }

        //expasion of the BB to avoid dangerous targets
        security_box = ignition::math::Box(Bbox.Min() - scaling, Bbox.Max() + scaling);
        targetProjection = ignition::math::Vector3d(newTarget.X(), newTarget.Y(), Bbox.Min().Z());

        //if there is a model too closed to the new target, I retry
        if (security_box.Contains(targetProjection))
        {

          //debug std::cout << ">>>> New target: " << newTarget << " too near to " << model->GetName() << std::endl;

          //debug std::cout << ">>>> security box =" << security_box << std::endl;

#ifdef DEBUG_
          // For debug
          gzdbg << ">>>> New target: " << newTarget << " too near to "
                << model->GetName() << std::endl;
#endif
          newTarget = this->target;
          break;
        }
      }
    }
  }

  //Update the speed to reach the new target with random noise
  this->velocity = SPEED_BASE + ignition::math::Rand::DblNormal(0, 0.1); //[m/s]
  this->rot_velocity = this->velocity.Length() * 4.375;                  //[rad/s]

  //all models are enough far, so I can set the new target
  this->target = newTarget;

  //ROS_INFO("SET RANDOM TARGET");

#ifdef DEBUG_
  // For debug
  std::string line;
  myfile.open("/home/alberto/vel_debug.txt", std::ios::app);
  myfile << "#### Set speed of " << this->actor->GetName() << " to: " << this->velocity << " [m/s] ####" << std::endl;
  myfile.close();
#endif
}

ignition::math::Vector3d AutoActorPlugin::Perpendicular(ignition::math::Vector3d &_pos, bool dir_pi_2) //Added by Bacchin Alberto
{

  ignition::math::Vector3d rot;
  rot.Z(0);
  _pos.Normalize();
  //rotation of pi/2 counter clockwise
  if (dir_pi_2)
  {
    rot.X(-_pos.Y());
    rot.Y(_pos.X());
  }
  else //rotation of pi/2 clockwise
  {
    rot.X(_pos.Y());
    rot.Y(-_pos.X());
  }

  return rot;
}

std::tuple<bool, double> AutoActorPlugin::checkIntersection(ignition::math::Box box, ignition::math::Vector3d origin, ignition::math::Vector3d direction, double d_in_min, double d_in_max)
{
  ignition::math::Vector3d min = box.Min(), max = box.Max();
  //Vertex of the 2D box
  ignition::math::Vector2d A(min.X(), min.Y()), B(max.X(), min.Y()), C(max.X(), max.Y()), D(min.X(), max.Y());
  //Direction points, arbitrary
  ignition::math::Vector2d dir1(origin.X(), origin.Y()), dir2(direction.X(), direction.Y()), origin2d(origin.X(), origin.Y());

  if (dir1 == dir2)
    dir2 = 2 * dir2;

  ignition::math::Line2d ab(A, B), bc(B, C), cd(C, D), da(D, A), dir(dir1, dir2);
  std::vector<ignition::math::Line2d> edges{ab, bc, cd, da};
  std::vector<ignition::math::Vector2d> intersections;
  ignition::math::Vector2d point;
  intersections.reserve(2);

  for (auto e : edges)
  {

    if (dir.Intersect(e, point))
      intersections.push_back(point);
  }

  if (intersections.empty())
    return std::make_tuple(false, 0);

  if (intersections.size() == 1)
    return std::make_tuple(true, (origin2d - intersections[0]).Length());

  double d1 = (origin2d - intersections[0]).Length();
  double d2 = (origin2d - intersections[1]).Length();
  double d_min, d_max;

  if (d1 > d2)
  {
    d_max = d1;
    d_min = d2;
  }
  else
  {
    d_max = d2;
    d_min = d1;
  }

  //check if no overlapping
  if (d_min > d_in_max || d_in_min > d_max)
  {
    return std::make_tuple(false, 0);
  }

  return std::make_tuple(true, std::max(d_min, d_in_min));
}

/*ignition::math::Vector2d AutoActorPlugin::Intersect(ignition::math::Vector2d A, ignition::math::Vector2d B,
                                                    ignition::math::Vector2d C, ignition::math::Vector2d D)
{

  ignition::math::Vector2d result;
  // Line AB represented as a1x + b1y = c1
  double a1 = B.Y() - A.Y();
  double b1 = A.X() - B.X();
  double c1 = B.Y() * A.X() - A.Y() * B.X();

  // Line CD represented as a2x + b2y = c2
  double a2 = D.Y() - C.Y();
  double b2 = C.X() - D.X();
  double c2 = D.Y() * C.X() - C.Y() * D.X();

  double determinant = a1 * b2 - a2 * b1;

  if (determinant == 0)
  {
    // The lines are parallel
    return result(ignition::math::NAN_D,ignition::math::NAN_D);
  }
  else
  {
    return result((b2 * c1 - b1 * c2) / determinant, (a1 * c2 - a2 * c1) / determinant);
  }
} */

/////////////////////////////////////////////////
void AutoActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos) //Updated by Bacchin Alberto
{
  std::tuple<bool, double> obs_intersection;
  ignition::math::Vector3d correction, other_actor_pose;
  ignition::math::Vector3d actor_pose_projection = this->actor->WorldPose().Pos();

  //Dilation of the bounding box of the model to improve the awareness
  ignition::math::Box security_box, security_box2, Bbox;
  ignition::math::Vector3d scaling(0.2, 0.2, 0);
  ignition::math::Vector3d scaling2(2, 2, 0);
  ignition::math::Vector3d scaling_actor(0.5, 0.5, 1.2138);

  //Parameters of the function that compute the correction to the trajectory
  double a = 2.0, b = 1.0;

  //Cycle over the models of the world
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);

    //Ignore models indicated in the .world file, like ground
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                  model->GetName()) == this->ignoreModels.end())
    {

      //Actors bounding box handling (the method BoundingBox() dose not work properly for actors)
      if (std::find(this->otherActors.begin(), this->otherActors.end(),
                    model->GetName()) == this->otherActors.end())
      {
        Bbox = model->BoundingBox();
      }

      else
      {
        other_actor_pose = model->WorldPose().Pos();
        Bbox = ignition::math::Box(other_actor_pose - scaling_actor, other_actor_pose + scaling_actor);
      }

      //Compute a security distance of scaling2 meters
      security_box2 = ignition::math::Box(Bbox.Min() - scaling2, Bbox.Max() + scaling2);
      //Project the actor pose on the 2D plane
      actor_pose_projection.Z(std::max(Bbox.Min().Z(), 0.0));

      //A collision is handled when the obstacle is closer then 2 meters
      if (security_box2.Contains(actor_pose_projection))
      {
        //projection in a 2D plane
        _pos.Z(Bbox.Min().Z());

        //Dynamic ostacles are treated with larger security bounds
        if (std::find(this->dynObstacles.begin(), this->dynObstacles.end(),
                      model->GetName()) == this->dynObstacles.end())
        {
          scaling.Set(0.2, 0.2, 0.0);
          b = 1.0;
        }
        else
        {
          scaling.Set(0.7, 0.7, 0.0);
          b = 2.0;
        }

        security_box = ignition::math::Box(Bbox.Min() - scaling, Bbox.Max() + scaling);

        //Check if there are any intersection in the direction of motion
        obs_intersection = security_box.IntersectDist(actor_pose_projection, direction, 0.01, 1.99);
        //obs_intersection = checkIntersection(security_box, actor_pose_projection, _pos, 0.01, 1.99); //handcrafted implementation

#ifdef DEBUG_
        // For debug
        gzdbg << "....................OBSTACLE HANDLING: " << model->GetName() << "................." << std::endl;
        gzdbg << "Actor world pose projection: " << actor_pose_projection << std::endl;
        gzdbg << "Collision box: " << security_box << std::endl;
        gzdbg << "Direction (to the target) pos: " << _pos << std::endl;
        gzdbg << "Current Direction: " << direction << std::endl;
        gzdbg << "Intersection between " << this->actor->GetName() << " and " << model->GetName() << " = " << std::get<0>(obs_intersection) << std::endl;
        gzdbg << "Distance between a" << this->actor->GetName() << " and " << model->GetName() << " = " << std::get<1>(obs_intersection) << std::endl;
        gzdbg << "......................................................" << std::endl;
#endif

        if (std::get<0>(obs_intersection))
        {

          //Correction have 2 components:

          //First: a force in the direction opposite to the actual motion
          correction = -_pos;
          correction.Normalize();

#ifdef DEBUG_
          // For debug
          gzdbg << "!!! POSSIBLE COLLISION, Correction... " << std::endl;
          gzdbg << "Correction offset: " << correction << std::endl;
          gzdbg << "Perpendicular to pose: " << Perpendicular(_pos, true) << std::endl;
          gzdbg << "cos(ang<correction_pos>): " << correction.Dot(_pos) << std::endl;
#endif

          //Second: a force in perpendicular direction to the motion, propotional to the cos(angle<_pos,correction>)
          correction = correction + Perpendicular(_pos, true) /** abs(correction.Dot(_pos))*/; //(in the actual implementation cos() weight is useless)

          correction.Normalize();

          //Correction strength depends on the distance from the obstacle
          _pos = _pos + (a * exp(-b * std::get<1>(obs_intersection))) * correction;
          _pos.Normalize();

          if (std::get<1>(obs_intersection) < 0.1 && std::get<1>(obs_intersection) > 0)
          {
            this->ChooseNewTarget();
          }

#ifdef DEBUG_
          // For debug
          //gzdbg << "!!! POSSIBLE COLLISION, Correction... " << std::endl;
          gzdbg << "Correction final offset: " << correction << std::endl;
          gzdbg << "Weight of correction: " << (a * exp(-b * std::get<1>(obs_intersection))) << std::endl;
          gzdbg << "New pos: " << _pos << std::endl;
#endif
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  gazebo_person_detection::actor_vel msg;

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

#ifdef DEBUG_
  // For debug
  gzdbg << "--------------------------------------------------------" << std::endl;
  gzdbg << "DISTANCE FROM TARGET: " << distance << std::endl;
#endif

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < this->tolerance)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

#ifdef DEBUG_
  // For debug
  gzdbg << "TARGET: " << target[0] << " " << target[1] << " " << target[2] << " " << std::endl;
  gzdbg << "INITIAL POSE: " << pose << std::endl;

#endif

#ifdef DEBUG_
  // For debug
  gzdbg << "DIRECTION TO THE TARGET: " << pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;

#endif

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize(); //* this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  direction = pos;

#ifdef DEBUG_
  // For debug
  gzdbg << "DIRECTION TO THE TARGET AFTER OA: " << pos[0] << " " << pos[1] << " " << pos[2] << " " << std::endl;

#endif

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

#ifdef DEBUG_
  // For debug
  gzdbg << "YAW TO THE TARGET: " << yaw << std::endl;

#endif

#ifdef DEBUG_
  // For debug
  gzdbg << "CURRENT POSE: " << pose.Pos()[0] << " " << pose.Pos()[1] << " " << pose.Pos()[2] << " " << pose.Rot().Euler()[0] << " " << pose.Rot().Euler()[1] << " " << pose.Rot().Euler()[2] << " " << std::endl;
  gzdbg << "***********" << std::endl;

#endif

  pose.Pos() = pose.Pos() + pos * this->velocity * dt;
  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * this->rot_velocity * dt);
#ifdef DEBUG_
  // For debug
  gzdbg << "UPDATE: " << pos * this->velocity * dt << std::endl;
  gzdbg << "NEW POSE: " << pose << std::endl;
  gzdbg << "***********" << std::endl;

#endif

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(this->room_x[0], std::min(this->room_x[1], pose.Pos().X())));
  pose.Pos().Y(std::max(this->room_y[0], std::min(this->room_y[1], pose.Pos().Y())));
  pose.Pos().Z(this->room_z);

#ifdef DEBUG_
  // For debug
  gzdbg << "BOUNDS x: " << this->room_x[0] << "-" << this->room_x[1] << "   y: " << this->room_y[0] << "-" << this->room_y[1] << std::endl;
  gzdbg << "NEW POSE AFTER CHECK: " << pose << std::endl;
  gzdbg << "***********" << std::endl;

#endif

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
                             this->actor->WorldPose().Pos())
                                .Length();

  //Try to unstuck the actor
  double check_dist = (check_pose -
                       this->actor->WorldPose().Pos())
                          .Length();

  if (check_dist > 0.1)
  {

    stuck_count = 0;
    check_pose = this->actor->WorldPose().Pos();
  }
  else
  {

    stuck_count++;
  }

  if (stuck_count > 250)
  {
    ROS_INFO("Timeout for %s!", this->actor->GetName().c_str());
    this->ChooseNewTarget();
    ROS_INFO("New target selected");
    stuck_count = 0;
  }

#ifdef DEBUG_
  // For debug
  gzdbg << "NEW POSE: " << pose.Pos() << std::endl;
  gzdbg << "CURRENT POSE: " << this->actor->WorldPose().Pos()[0] << " " << this->actor->WorldPose().Pos()[1] << " " << this->actor->WorldPose().Pos()[2] << " " << std::endl;
  gzdbg << "DIFF POSE: " << (pose.Pos() - this->actor->WorldPose().Pos())[0] << " " << (pose.Pos() - this->actor->WorldPose().Pos())[1] << " " << (pose.Pos() - this->actor->WorldPose().Pos())[2] << " " << std::endl;
  gzdbg << "TRAVELLED DISTANCE: " << distanceTraveled << std::endl;

#endif

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;

  msg.id = (int)(this->actor->GetName().back()) - '0';
  msg.twist.header.frame_id = "world";
  msg.twist.header.stamp = ros::Time::now();
  msg.twist.twist.linear.x = pos.X() * this->velocity.X();
  msg.twist.twist.linear.y = pos.Y() * this->velocity.Y();
  msg.twist.twist.angular.z = yaw.Radian() * this->rot_velocity;

  pub.publish(msg);

#ifdef DEBUG_
  // For debug
  gzdbg << "FINAL POSE: " << pose.Pos()[0] << " " << pose.Pos()[1] << " " << pose.Pos()[2] << " " << pose.Rot().Euler()[0] << " " << pose.Rot().Euler()[1] << " " << pose.Rot().Euler()[2] << " " << std::endl;
  gzdbg << "***********" << std::endl;

#endif
}
