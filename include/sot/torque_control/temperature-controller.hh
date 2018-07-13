/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_torque_control_temperature_controller_H__
#define __sot_torque_control_temperature_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sot_temperature_controller_EXPORTS)
#    define SOTTEMPERATURECONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTEMPERATURECONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTEMPERATURECONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/utils/ddpsolver.hh>

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/talos-common.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      //Eigen::VectorXd svdSolveWithDamping(const Eigen::JacobiSVD<Eigen::MatrixXd>& A, const Eigen::VectorXd &b, double damping);

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTEMPERATURECONTROLLER_EXPORT TemperatureController
	:public::dynamicgraph::Entity
      {
        typedef TemperatureController EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        TemperatureController( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(q,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(qDot,         dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(temp_motor,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(tau,          dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(temp_ambiant, dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(u,           dynamicgraph::Vector);  /// integral of dqDes
        // DEBUG SIGNALS
        //DECLARE_SIGNAL_OUT(dqDes,             dynamicgraph::Vector);  /// dqDes = J^+ * Kf * (fRef-f)


        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[TemperatureController-"+name+"] "+msg, t, file, line);
        }

      protected:
        Eigen::VectorXd   m_u;             /// desired joint positions

        /// robot geometric/inertial data
        //typedef Eigen::Matrix<double, 6*Hrp2_14::NBBODIES, Hrp2_14::NBDOF>  AllJacobian;


      }; // class TemperatureController

    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_temperature_controller_H__
