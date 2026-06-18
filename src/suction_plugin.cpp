#include "suction_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/math/AxisAlignedBox.hh>

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

SuctionPlugin::SuctionPlugin()
{
    std::cout << "======= SuctionPlugin constructed =======" << std::endl;
}

// Configuration
bool useSuctionRadius = false;

void SuctionPlugin::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
    this->modelEntity = _entity;
    std::cout << "======= Configure started. Entity ID: " << _entity << " =======" << std::endl;

    // Check if entity is valid and has Name component
    auto name = _ecm.Component<components::Name>(_entity);
    if (name)
    {
        std::cout << "Model name attached to plugin: " << name->Data() << std::endl;
    }
    else
    {
        std::cerr << "ERROR: Entity does not have Name component!" << std::endl;
    }

    // Read suction radius from SDF
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction radius set to: " << this->suctionRadius << std::endl;
    }
    else
    {
        std::cout << "Using default suction radius: " << this->suctionRadius << std::endl;
    }

    // Read filter name from SDF
    if (_sdf->HasElement("filter_name"))
    {
        this->filterName = _sdf->Get<std::string>("filter_name");
        std::cout << "Filter name set to: " << this->filterName << std::endl;
    }
    else
    {
        std::cout << "Using default filter name: " << this->filterName << std::endl;
    }

    // Read parent link name from SDF
    if (_sdf->HasElement("link_name"))
    {
        this->parentLinkName = _sdf->Get<std::string>("link_name");
        std::cout << "Parent link name set to: " << this->parentLinkName << std::endl;
    }

    // Read use_suction_radius from SDF
    if (_sdf->HasElement("use_suction_radius"))
    {
        this->useSuctionRadius = _sdf->Get<bool>("use_suction_radius");
        std::cout << "Use suction radius: " << (this->useSuctionRadius ? "TRUE" : "FALSE") << std::endl;
    }
    
    if (_sdf->HasElement("suction_force"))
    {
        this->suctionForce = _sdf->Get<double>("suction_force");
        std::cout << "Suction Force: " << this->suctionForce << std::endl;
    }
    
    if (_sdf->HasElement("suction_radius"))
    {
        this->suctionRadius = _sdf->Get<double>("suction_radius");
        std::cout << "Suction Radius: " << this->suctionRadius << std::endl;
    }

    if (_sdf->HasElement("suction_topic"))
    {
        this->suctionTopic = _sdf->Get<std::string>("suction_topic");
        std::cout << "Suction Topic: " << this->suctionTopic << std::endl;
    }

    // Create and advertise suction activation topic
    this->publisher = this->node.Advertise<msgs::Boolean>(this->suctionTopic);
    if (!this->publisher)
    {
        std::cerr << "ERROR: Failed to advertise topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Topic advertised successfully: " << this->suctionTopic << std::endl;
    }

    // Subscribe to suction activation topic
    if (!this->node.Subscribe(this->suctionTopic,
                              &SuctionPlugin::OnSuctionActivate,
                              this))
    {
        std::cerr << "ERROR: Failed to subscribe to topic: " << this->suctionTopic << std::endl;
    }
    else
    {
        std::cout << "Successfully subscribed to topic: " << this->suctionTopic << std::endl;
    }

    std::cout << "======= Configure completed =======" << std::endl;
}

void SuctionPlugin::PreUpdate(const UpdateInfo &_info,
                              EntityComponentManager &_ecm)
{
    // 0. Create AxisAlignedBox components for links that lacked them during PostUpdate
    for (const auto &link : this->pendingAabbCreations)
    {
        _ecm.CreateComponent(link, components::AxisAlignedBox());
    }
    this->pendingAabbCreations.clear();

    // 1. Apply pending wrench commands (computed in PostUpdate)
    for (const auto &cmd : this->pendingWrenchCommands)
    {
        // Force on target link (pull toward gripper)
        auto wrenchComp = _ecm.Component<components::ExternalWorldWrenchCmd>(cmd.targetLink);
        if (!wrenchComp)
        {
            components::ExternalWorldWrenchCmd wrenchCmd;
            msgs::Wrench wrenchMsg;
            msgs::Set(wrenchMsg.mutable_force(), cmd.force);
            wrenchCmd.Data() = wrenchMsg;
            _ecm.CreateComponent(cmd.targetLink, wrenchCmd);
        }
        else
        {
            msgs::Set(wrenchComp->Data().mutable_force(), cmd.force);
        }

        // Equal and opposite force on gripper link
        math::Vector3d gripperForce = -cmd.force;
        auto gripperWrenchComp = _ecm.Component<components::ExternalWorldWrenchCmd>(cmd.gripperLink);
        if (!gripperWrenchComp)
        {
            components::ExternalWorldWrenchCmd wrenchCmd;
            msgs::Wrench wrenchMsg;
            msgs::Set(wrenchMsg.mutable_force(), gripperForce);
            wrenchCmd.Data() = wrenchMsg;
            _ecm.CreateComponent(cmd.gripperLink, wrenchCmd);
        }
        else
        {
            msgs::Set(gripperWrenchComp->Data().mutable_force(), gripperForce);
        }
    }
    this->pendingWrenchCommands.clear();

    // 2. Handle Attachment (Joint Creation)
    if (this->targetEntityToAttach != kNullEntity)
    {
        if (this->attachedEntity == kNullEntity) // Double check
        {
             // Find gripper link using the shared helper
             Entity resolvedModel = kNullEntity;
             Entity gripperLink = this->FindGripperLink(_ecm, resolvedModel);

             // Find target link
             Entity targetLink = kNullEntity;
             auto targetLinks = _ecm.ChildrenByComponents(this->targetEntityToAttach, components::Link());
             if (!targetLinks.empty()) targetLink = targetLinks[0];

             if (gripperLink != kNullEntity && targetLink != kNullEntity)
             {
                 this->jointEntity = _ecm.CreateEntity();
                 components::DetachableJointInfo jointInfo;
                 jointInfo.parentLink = gripperLink;
                 jointInfo.childLink = targetLink;
                 jointInfo.jointType = "fixed";
                 _ecm.CreateComponent(this->jointEntity, components::DetachableJoint(jointInfo));
                 _ecm.CreateComponent(this->jointEntity, components::Name("suction_joint"));
                 
                 this->attachedEntity = this->targetEntityToAttach;
                 std::cout << "SUCCESS: Attached to " << this->attachedEntity << std::endl;
             }
             else
             {
                 std::cerr << "ERROR: Failed to find links for attachment in PreUpdate." << std::endl;
             }
        }
        this->targetEntityToAttach = kNullEntity; // Reset
    }

    // 3. Handle Detachment
    if (!this->suctionActive && this->attachedEntity != kNullEntity)
    {
        this->Detach(_ecm);
    }
}

void SuctionPlugin::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{   
    // CONTACT MODE
    // Sensing: Only look for targets if suction is active and we are NOT attached
    // AND we are in Contact Mode
    if (this->suctionActive && this->attachedEntity == kNullEntity && !this->useSuctionRadius)
    {
        this->FindTargetContact(_ecm);
    }

    // SUCTION MODE
    // 1. Logic for Suction (Sensing/Actuation)
    if (this->suctionActive && this->attachedEntity == kNullEntity)
    {
        if (this->useSuctionRadius)
        {
            this->FindTargetRadius(_ecm);
        }
        // Contact mode is handled above, which sets targetEntityToAttach
    }

    // Note: Attachment/Detachment joint creation and wrench application
    // are performed in PreUpdate, where the ECM is non-const.
}

Entity SuctionPlugin::FindGripperLink(const EntityComponentManager &_ecm,
                                      Entity &_resolvedModelEntity) const
{
    Entity gripperLink = kNullEntity;
    _resolvedModelEntity = this->modelEntity;

    if (this->parentLinkName.empty())
        return kNullEntity;

    // If modelEntity is the world entity (ID 1), resolve the actual model
    // that contains our parent link
    Entity searchModel = this->modelEntity;
    if (searchModel == 1)
    {
        _ecm.Each<components::Model, components::Name>(
            [&](const Entity &_modelEntity,
                const components::Model *,
                const components::Name *) -> bool
        {
            auto links = _ecm.ChildrenByComponents(_modelEntity, components::Link());
            for (const auto &link : links)
            {
                auto nameComp = _ecm.Component<components::Name>(link);
                if (nameComp && nameComp->Data() == this->parentLinkName)
                {
                    searchModel = _modelEntity;
                    _resolvedModelEntity = _modelEntity;
                    return false;
                }
            }
            return true;
        });
    }

    // Find the gripper link within the (possibly resolved) model
    auto links = _ecm.ChildrenByComponents(searchModel, components::Link());
    for (const auto &link : links)
    {
        auto nameComp = _ecm.Component<components::Name>(link);
        if (nameComp && nameComp->Data() == this->parentLinkName)
        {
            gripperLink = link;
            break;
        }
    }

    return gripperLink;
}

void SuctionPlugin::FindTargetRadius(const EntityComponentManager &_ecm)
{
    // Find the gripper link using the shared helper.
    // resolvedModel is set to the actual model containing the gripper link,
    // which may differ from this->modelEntity when the plugin is at world level.
    Entity resolvedModel = kNullEntity;
    Entity gripperLink = this->FindGripperLink(_ecm, resolvedModel);

    if (gripperLink == kNullEntity) return;

    // Calculate Gripper Link World Pose using the resolved model entity
    auto modelPose = _ecm.Component<components::Pose>(resolvedModel);
    if (!modelPose) return;
    
    math::Pose3d gripperWorldPose = modelPose->Data();
    auto linkPose = _ecm.Component<components::Pose>(gripperLink);
    if (linkPose) gripperWorldPose = modelPose->Data() * linkPose->Data();

    Entity nearestEntity = kNullEntity;
    double minDistance = std::numeric_limits<double>::max();

    // Clear pending wrench commands from previous iteration
    this->pendingWrenchCommands.clear();

    // Iterate all models to find candidates
    _ecm.Each<components::Name, components::Pose, components::Model>(
        [&](const Entity &_entity,
            const components::Name *_name,
            const components::Pose *_pose,
            const components::Model *) -> bool
        {
            if (_entity == resolvedModel) return true;
            if (_name->Data().find(this->filterName) == std::string::npos) return true;

            // Compute distance from gripper position to the nearest face of the
            // target model's link bounding boxes.
            const math::Vector3d &gripperPos = gripperWorldPose.Pos();
            const math::Pose3d &targetModelWorldPose = _pose->Data();
            Entity bestLink = kNullEntity;
            math::Vector3d bestSurfacePoint;
            double distance = std::numeric_limits<double>::max();

            // Iterate over the target model's links
            auto targetLinks = _ecm.ChildrenByComponents(_entity, components::Link());
            for (const auto &link : targetLinks)
            {
                // Get the AxisAlignedBox component of the link (stored in link's local frame)
                auto aabbComp = _ecm.Component<components::AxisAlignedBox>(link);
                if (!aabbComp)
                {
                    // Defer creation to PreUpdate (ECM is read-only here)
                    this->pendingAabbCreations.push_back(link);
                    continue;
                }

                const math::AxisAlignedBox &box = aabbComp->Data();

                // Compute the link's world pose (model world pose * link relative pose)
                auto linkRelativePoseComp = _ecm.Component<components::Pose>(link);
                math::Pose3d linkWorldPose = targetModelWorldPose;
                if (linkRelativePoseComp)
                {
                    linkWorldPose = targetModelWorldPose * linkRelativePoseComp->Data();
                }

                // Transform the AABB corners from link-local frame to world frame
                math::Vector3d worldMin = linkWorldPose.Rot() * box.Min() + linkWorldPose.Pos();
                math::Vector3d worldMax = linkWorldPose.Rot() * box.Max() + linkWorldPose.Pos();

                // Extract world-space box extents for runtime debug inspection
                double minX = worldMin.X();
                double maxX = worldMax.X();
                double minY = worldMin.Y();
                double maxY = worldMax.Y();
                double minZ = worldMin.Z();
                double maxZ = worldMax.Z();

                // Midpoints for face center calculation
                double midX = (minX + maxX) / 2.0;
                double midY = (minY + maxY) / 2.0;
                double midZ = (minZ + maxZ) / 2.0;

                // Compute center of each of the 6 faces of the AABB, find the closest
                // to the gripper position
                struct { double dist; math::Vector3d point; } bestFace;
                bestFace.dist = std::numeric_limits<double>::max();

                // X faces
                auto checkFace = [&](const math::Vector3d &faceCenter) {
                    double d = gripperPos.Distance(faceCenter);
                    if (d < bestFace.dist) {
                        bestFace.dist = d;
                        bestFace.point = faceCenter;
                    }
                };
                checkFace(math::Vector3d(minX, midY, midZ));
                checkFace(math::Vector3d(maxX, midY, midZ));
                checkFace(math::Vector3d(midX, minY, midZ));
                checkFace(math::Vector3d(midX, maxY, midZ));
                checkFace(math::Vector3d(midX, midY, minZ));
                checkFace(math::Vector3d(midX, midY, maxZ));

                double linkDist = bestFace.dist;
                math::Vector3d bestSurfacePointLocal = bestFace.point;
                if (linkDist < distance)
                {
                    distance = linkDist;
                    bestLink = link;
                    bestSurfacePoint = bestSurfacePointLocal;
                }
            }

            if (distance <= this->suctionRadius && bestLink != kNullEntity)
            {
                // Direction from the target surface point toward the gripper
                math::Vector3d direction = gripperPos - bestSurfacePoint;
                direction.Normalize();

                // Force pulling target toward gripper
                math::Vector3d force = direction * this->suctionForce;

                // Only log force application for newly captured targets (not already attached ones)
                if (this->attachedEntity != _entity)
                {
                    std::cout << "Applying suction force " << this->suctionForce << " to " << _name->Data() << std::endl;
                }

                // Store pending wrench command for application in PreUpdate
                this->pendingWrenchCommands.push_back(
                    {bestLink, gripperLink, force});

                // Check for attachment
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestEntity = _entity;
                }
            }
            return true;
        });

    // If close enough (e.g. < 0.02m), schedule attachment
    if (nearestEntity != kNullEntity && minDistance < 0.02) 
    {
        this->targetEntityToAttach = nearestEntity;
        std::cout << "FindTargetRadius: Target " << nearestEntity << " in range (" << minDistance << "). Scheduling attachment." << std::endl;
    }
}

void SuctionPlugin::FindTargetContact(const EntityComponentManager &_ecm)
{
    // Find the gripper link using the shared helper.
    // _resolvedModelEntity holds the actual model containing the gripper link,
    // which may differ from this->modelEntity when the plugin is at world level.
    Entity resolvedModelEntity = kNullEntity;
    Entity gripperLink = this->FindGripperLink(_ecm, resolvedModelEntity);

    if (gripperLink == kNullEntity) return;

    auto sensors = _ecm.ChildrenByComponents(gripperLink, components::Sensor());
    for (const auto &sensor : sensors)
    {
        auto contactData = _ecm.Component<components::ContactSensorData>(sensor);
        if (contactData)
        {
            for (const auto &contact : contactData->Data().contact())
            {
                Entity col1 = contact.collision1().id();
                Entity col2 = contact.collision2().id();
                
                auto GetModelFromCollision = [&](Entity _colEntity) -> Entity {
                    auto linkEntity = _ecm.ParentEntity(_colEntity);
                    if (linkEntity == kNullEntity) return kNullEntity;
                    return _ecm.ParentEntity(linkEntity);
                };

                Entity model1 = GetModelFromCollision(col1);
                Entity model2 = GetModelFromCollision(col2);
                
                Entity potentialTarget = kNullEntity;
                if (model1 == resolvedModelEntity && model2 != kNullEntity) potentialTarget = model2;
                else if (model2 == resolvedModelEntity && model1 != kNullEntity) potentialTarget = model1;
                
                if (potentialTarget != kNullEntity)
                {
                    auto nameComp = _ecm.Component<components::Name>(potentialTarget);
                    if (nameComp && nameComp->Data().find(this->filterName) != std::string::npos)
                    {
                        this->targetEntityToAttach = potentialTarget;
                        std::cout << "FindTargetContact: Match found " << potentialTarget << ". Scheduling attachment." << std::endl;
                        return; // Found one, that's enough
                    }
                }
            }
        }
    }
}

void SuctionPlugin::Detach(EntityComponentManager &_ecm)
{
    if (this->jointEntity != kNullEntity)
    {
        std::cout << "Detaching entity: " << this->attachedEntity << std::endl;
        _ecm.RequestRemoveEntity(this->jointEntity);
        this->jointEntity = kNullEntity;
        this->attachedEntity = kNullEntity;
    }
}

void SuctionPlugin::OnSuctionActivate(const msgs::Boolean &_msg)
{
    bool newState = _msg.data();
    std::cout << "======= ALERT: Received suction command: " << (newState ? "ACTIVATE" : "DEACTIVATE") << " =======" << std::endl;
    this->suctionActive = newState;
}

double SuctionPlugin::CalculateDistance(const math::Pose3d &_pose1,
                                        const math::Pose3d &_pose2)
{
    return _pose1.Pos().Distance(_pose2.Pos());
}

GZ_ADD_PLUGIN(SuctionPlugin,
              gz::sim::System,
              SuctionPlugin::ISystemConfigure,
              SuctionPlugin::ISystemPreUpdate,
              SuctionPlugin::ISystemPostUpdate)
