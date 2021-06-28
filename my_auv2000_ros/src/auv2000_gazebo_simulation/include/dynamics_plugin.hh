/*
 * Copyright (C) 2017  Brian Bingham
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

#ifndef AUV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_
#define AUV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>

#include <sdf/sdf.hh>

#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

namespace gazebo
{
  class AUVDynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AUVDynamicsPlugin();

    /// \brief Destructor.
    public: virtual ~AUVDynamicsPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback for Gazebo simulation engine.
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
    /// \brief Convenience function for calculating the area of circle segment
    /// \param[in] R Radius of circle
    /// \param[in] h Height of the chord line
    /// \return The area
    private: double CircleSegment(double R, double h);

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    private: physics::WorldPtr world;

    /// \brief Pointer to model link in gazebo,
    /// optionally specified by the bodyName parameter.
    /// The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Simulation time of the last update.
    private: common::Time prevUpdateTime;

    /// \brief Linear velocity from previous time step,
    /// for estimating acceleration.
    private: ignition::math::Vector3d prevLinVel;

    /// \brief Angular velocity from previous time step,
    /// for estimating acceleration.
    private: ignition::math::Vector3d prevAngVel;

    /// \brief Values to set via Plugin Parameters.
    /// Plugin Parameter: Added mass in surge, X_\dot{u}.
    /// //Added mass
    private: double paramXudot;
    private: double paramKpdot;
    private: double paramYvdot;
    private: double paramZwdot;
    private: double paramNvdot;
    private: double paramMwdot;
    private: double paramNrdot;
    private: double paramMqdot;
    private: double paramZqdot;
    private: double paramYrdot;
    // Hydrodynamic
    private: double paramXuu;
    private: double paramYvv;
    private: double paramZww;
    private: double paramNvv;
    private: double paramMww;
    private: double paramYrr;
    private: double paramZqq;
    private: double paramNrr;
    private: double paramMqq;
    private: double paramKpp;
    // Body Lift
    private: double paramYuvl;
    private: double paramZuwl;
    private: double paramNuvl;
    private: double paramMuwl;
    // Fin Lift
    private: double paramZuwf;
    private: double paramZuqf;
    private: double paramMuwf;
    private: double paramMuqf;
    // Prop
    private: double paramRprop;
    // Rudder
    private: double paramXuudd;
    private: double paramXuvd;
    private: double paramXurd;
    private: double paramYuudd;
    private: double paramYuvd;
    private: double paramYurd;
    // Bouyancy and Weigth
    private: double paramBuoy;

	
    // Center of mass
    private: ignition::math::Vector3d linearAccFiltered;
    private: ignition::math::Vector3d angularAccFiltered;
    /// \brief Water height [m].
    private: double waterLevel;

    /// \brief Water density [kg/m^3].
    private: double waterDensity;

    /// \brief Length discretization, i.e., "N"
    private: int paramLengthN;

    /// \brief Added mass matrix, 6x6.
    private: Eigen::MatrixXd Ma;

    /// \brief The name of the wave model
    protected: std::string waveModelName;


    /// \brief Convert vector to comply with the NED reference frame
  protected: ignition::math::Vector3d ToNED(ignition::math::Vector3d _vec);

  /// \brief Convert vector to comply with the NED reference frame
  protected: ignition::math::Vector3d FromNED(ignition::math::Vector3d _vec);
private: double sat(double x, double min, double max );
    // /// \brief Wave parameters.
    // private: int paramWaveN;

    // /// \brief Wave amplitude values for N components.
    // private: std::vector<float> paramWaveAmps;

    // /// \brief Wave period values for N components.
    // private: std::vector<float> paramWavePeriods;

    // /// \brief Wave direction values for N components.
    // private: std::vector<std::vector<float>> paramWaveDirections;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief The wave parameters.
  //  private: std::shared_ptr<const asv::WaveParameters> waveParams;
  };
}

#endif