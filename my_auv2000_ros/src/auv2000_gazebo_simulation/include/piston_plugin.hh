#ifndef PISTON_PLUGIN_HH
#define PISTON_PLUGIN_HH

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
#include "utils/motor_stamped.h"
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <complex>

#include "math.h"
using namespace utils;
namespace gazebo
{
  // Foward declaration of AUVPISTON class
  class AUVPiston;

  /// \brief  Piston class
  class Piston
  {
    /// \brief Constructor
    /// \param[in] _parent Pointer to an SDF element to parse.
    public: explicit Piston(AUVPiston *_parent);

    /// \brief  the upper limit angle in rad
    public: double upperLimit;

    /// \brief the lowwer limit angle in rad
    public: double lowerLimit;
    
    /// \brief Link where thrust force is applied.
    public: physics::LinkPtr link;

    /// \brief Current, most recent command.
    public: double currCmd;

    /// \brief Joint controlling the rudder.
    public: physics::JointPtr pistonJoint;

    /// \brief Plugin parent pointer - for accessing world, etc.
    protected: AUVPiston *plugin;
  };

  class AUVPiston : public ModelPlugin
  {
    /// \brief Constructor.
    public: AUVPiston() = default;

    /// \brief Destructor.
    public: virtual ~AUVPiston() = default;

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

    public: bool onSetPistonCallBack(utils::CommandLongRequest &_req,
                                     utils::CommandLongResponse &_res);
    public: void OnPistonCmdCallBack(const utils::motor_stamped::ConstPtr &_msg);
    public: void OnPistonCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg);

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

    motor_stamped piston_msg;
    /// \brief Topic name for incoming ROS thruster commands from keyboard.
    public: std::string cmdKeyboardTopic;

    /// \brief Subscription to thruster commands from keyboard.
    public: ros::Subscriber cmdKeyboardSub;

    /// \brief Last time received a command via ROS topic.
    public: common::Time lastCmdTime;

    /// \brief Service name for incoming GCS commands.
    public: std::string setPistonService;

    /// \brief Subscription to GCS commands.
    public: ros::ServiceServer resSetPiston;
    public:ros::Publisher pubPiston;
    /// \brief Piston-locking status from GCS.
    ///
    public: bool pistonLocked = true;
    public: double Piston_pos_set = 0.04;
    public: int cnt = 0 ;

    /// \brief Keyboard control status from keyboard.
    ///
    public: bool keyboardEnabled = true;

    /// \brief Vector of thruster instances
    private: std::vector<Piston> piston;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif // PISTON_PLUGIN_HH
