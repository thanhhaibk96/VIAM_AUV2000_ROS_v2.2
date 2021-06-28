#ifndef RUDDER_PLUGIN_HH
#define RUDDER_PLUGIN_HH

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>

#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <utils/CommandLong.h>
#include <utils/ThrustCommand.h>
#include <utils/KeyboardCommand.h>
#include <gazebo/physics/physics.hh>
#include "utils/mx28_stamped.h"
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <complex>

#include "math.h"
using namespace utils;
namespace gazebo
{
  // Foward declaration of AUVRUDDER class
  class AUVRudder;

  /// \brief Thruster class
  class Rudder
  {
    /// \brief Constructor
    /// \param[in] _parent Pointer to an SDF element to parse.
    public: explicit Rudder(AUVRudder *_parent);

    
    /// \brief  the upper limit angle in rad
    public: double upperLimit;

    /// \brief the lowwer limit angle in rad
    public: double lowerLimit;

    
    /// \brief Link where thrust force is applied.
    public: physics::LinkPtr link;

    /// \brief Current, most recent command.
    public: double currCmd;

    /// \brief Joint controlling the rudder.
    public: physics::JointPtr rudderJoint;

    /// \brief Plugin parent pointer - for accessing world, etc.
    protected: AUVRudder *plugin;
  };

  class AUVRudder : public ModelPlugin
  {
    /// \brief Constructor.
    public: AUVRudder() = default;

    /// \brief Destructor.
    public: virtual ~AUVRudder() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);
    private: physics::LinkPtr link;
    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \brief Convenience function for getting SDF parameters.
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    /// \param[in] _paramName The name of the element to parse.
    /// \param[in] _defaultVal The default value returned if the element
    /// does not exist.
    /// \return The value parsed.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   const double _defaultVal) const;

    /// \brief Takes ROS Drive commands and scales them by max thrust.
    /// \param[in] _cmd ROS drive command.
    /// \return Value scaled and saturated.

    private: void aniRudder(double delta_R);
    public: bool onSetRudderCallBack(utils::CommandLongRequest &_req,
                                     utils::CommandLongResponse &_res);
    public: void OnRudderCmdCallBack(const utils::mx28_stamped::ConstPtr &_msg);
    public: void OnRudderCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg);

    public: std::mutex mutex;

    /// \brief The ROS node handler used for communications.
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    public: physics::WorldPtr world;

    /// \brief Pointer to Gazebo parent model, retrieved when the model is
    /// loaded.
    private: physics::ModelPtr model;

    /// \brief Timeout for receiving Drive commands [s].
    private: double cmdTimeout;

    /// \brief Topic name for incoming ROS thruster commands.
    public: std::string cmdTopic;
    public: std::string pubTopic;
    /// \brief Subscription to thruster commands.
    public: ros::Subscriber cmdSub;
    public: ros::Publisher pubPosition;

    mx28_stamped rudder_msg;
    /// \brief Topic name for incoming ROS thruster commands from keyboard.
    public: std::string cmdKeyboardTopic;

    /// \brief Subscription to thruster commands from keyboard.
    public: ros::Subscriber cmdKeyboardSub;

    /// \brief Last time received a command via ROS topic.
    public: common::Time lastCmdTime;

    /// \brief Service name for incoming GCS commands.
    public: std::string setRudderService;

    /// \brief Subscription to GCS commands.
    public: ros::ServiceServer resSetRudder;

    /// \brief Propeller-locking status from GCS.
    public: bool rudderLocked = true;

    /// \brief Keyboard control status from keyboard.
    public: bool keyboardEnabled = true;

    /// \brief Vector of thruster instances
    private: std::vector<Rudder> rudder;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    // Rudder
    private: double paramXuudd;
    private: double paramXuvd;
    private: double paramXurd;
    private: double paramYuudd;
    private: double paramYuvd;
    private: double paramYurd;

  };
}

#endif // RUDDER_PLUGIN_HH
