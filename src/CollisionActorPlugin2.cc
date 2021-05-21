/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/math/Vector3.hh>

#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include "CollisionActorPlugin.hh"

#include <boost/shared_ptr.hpp>

using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(CollisionActorPlugin)

/////////////////////////////////////////////////
CollisionActorPlugin::CollisionActorPlugin()
{
}

/////////////////////////////////////////////////
void CollisionActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get a pointer to the actor
  auto actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);

  // Map of collision scaling factors
  std::map<std::string, ignition::math::Vector3d> scaling;
  std::map<std::string, ignition::math::Pose3d> offsets;

  // Read in the collision scaling factors, if present
  if (_sdf->HasElement("scaling"))
  {
    auto elem = _sdf->GetElement("scaling");
    while (elem)
    {
      if (!elem->HasAttribute("collision"))
      {
#ifdef DEBUG_
        gzdbg << "****** Skipping element without collision attribute" << std::endl;
#endif
        elem = elem->GetNextElement("scaling");
        continue;
      }
      auto name = elem->Get<std::string>("collision");

      if (elem->HasAttribute("scale"))
      {
        auto scale = elem->Get<ignition::math::Vector3d>("scale");
        scaling[name] = scale;
      }

      if (elem->HasAttribute("pose"))
      {
        auto pose = elem->Get<ignition::math::Pose3d>("pose");
        offsets[name] = pose;
      }
      elem = elem->GetNextElement("scaling");
    }
  }

  //for each link in the actor
  for (const auto &link : actor->GetLinks())
  {
#ifdef DEBUG_
    gzdbg << "***** Actor link " << link->GetName() << std::endl;
#endif
    // Init the links, which in turn enables collisions
    link->Init();

    if (scaling.empty())
      continue;
#ifdef DEBUG_
    gzdbg << "Collisions of link " << link->GetName() << ":" << std::endl;
#endif
    // Process all the collisions in all the links
    for (const auto &collision : link->GetCollisions())
    {
      auto name = collision->GetName();
#ifdef DEBUG_
      gzdbg << "> " << name << std::endl;
#endif

      //find a correspondence between collision link name and scaling name
      if (scaling.find(name) != scaling.end())
      {

#ifdef DEBUG_
        gzdbg << "Collision box original shape: " << collision->GetShape()->TypeStr() << std::endl;
#endif

        //auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(collision->GetShape());

        gazebo::physics::BoxShape box(collision);

        box.Init();

#ifdef DEBUG_
        gzdbg << "Init" << std::endl;
#endif

        box.SetSize(ignition::math::Vector3d(1, 1, 1));

#ifdef DEBUG_
        gzdbg << "Setsize" << std::endl;
#endif

        boost::shared_ptr<gazebo::physics::BoxShape> box_ptr(&box);

#ifdef DEBUG_
        gzdbg << "Create pointer to " << box_ptr->TypeStr() << std::endl;
#endif

        collision->SetShape(box_ptr);

        auto boxShape = collision->GetShape();

#ifdef DEBUG_
        gzdbg << "Collision box NEW shape: " << boxShape->TypeStr() << std::endl;
#endif

        // Make sure we have a box shape (not NULL ponter).
        if (boxShape)
        {
          boxShape->SetScale(scaling[name]); //scale the original box
#ifdef DEBUG_
          gzdbg << "Add box to " << name << "of size " << scaling[name] << std::endl;
#endif
        }
      }

      if (offsets.find(name) != offsets.end())
      {
        collision->SetInitialRelativePose(
            offsets[name] + collision->InitialRelativePose());
      }
    }
  }
}
