#ifndef SUCTION_PLUGIN_HPP_
#define SUCTION_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

namespace gz
{
    namespace sim
    {
        // Gazebo Harmonic uses versioned namespaces
        // Version 8 is for Harmonic
        namespace systems
        {

            class SuctionPlugin : public System,
                                  public ISystemConfigure,
                                  public ISystemPreUpdate,
                                  public ISystemPostUpdate
            {
            public:
                SuctionPlugin();
                ~SuctionPlugin() override = default;

                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;

                void PreUpdate(const UpdateInfo &_info,
                               EntityComponentManager &_ecm) override;

                void PostUpdate(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm) override;

            private:
            private:
                void OnSuctionActivate(const gz::msgs::Boolean &_msg);
                
                // Helper to find and attach to the nearest valid object
                void UpdateAttachment(gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr);
                
                // Helper to detach the currently attached object
                void Detach(gz::sim::EntityComponentManager &_ecm);

                double CalculateDistance(const gz::math::Pose3d &_pose1, const gz::math::Pose3d &_pose2);
                Entity modelEntity{kNullEntity};
                Entity attachedEntity{kNullEntity};
                Entity jointEntity{kNullEntity};
                Entity targetEntityToAttach{kNullEntity};
                
                double suctionRadius{0.1}; // Default 0.1m
                double suctionForce{20.0}; // Default 20N
                bool useSuctionRadius = false;
                bool enableJointCreationAtAttachment{true}; // Whether joint creation on attachment is allowed (on by default)
                std::string filterName;
                bool suctionActive{false};
                std::string parentLinkName{""}; // Name of the link to attach to
                
                void FindTargetRadius(const EntityComponentManager &_ecm);
                void FindTargetContact(const EntityComponentManager &_ecm);

                // Pending force commands computed in PostUpdate, applied in PreUpdate
                struct WrenchCommand {
                    Entity targetLink;
                    Entity gripperLink;
                    math::Vector3d force;
                };
                std::vector<WrenchCommand> pendingWrenchCommands;

                // Links that need an AxisAlignedBox component created (deferred to PreUpdate)
                std::vector<Entity> pendingAabbCreations;

                // Helper: resolve the actual model when plugin is attached at world level
                // (modelEntity == 1), then find and return the gripper link within that model.
                // _resolvedModelEntity is set to the model entity that contains the gripper link.
                Entity FindGripperLink(const EntityComponentManager &_ecm,
                                       Entity &_resolvedModelEntity) const;
                
                gz::transport::Node node;
                gz::transport::Node::Publisher publisher;
                std::string suctionTopic{"/suction/enable"};
            };

        } // namespace systems
    } // namespace sim
} // namespace gz

#endif // SUCTION_PLUGIN_HPP_