/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#include <sot/torque_control/temperature-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace Eigen;

#define PROFILE_DQ_DES_COMPUTATION "Temperature control computation"

/*#define REF_FORCE_SIGNALS m_fRightFootRefSIN << m_fLeftFootRefSIN << \
                          m_fRightHandRefSIN << m_fLeftHandRefSIN
#define FORCE_SIGNALS     m_fRightFootSIN << m_fLeftFootSIN << \
                          m_fRightHandSIN << m_fLeftHandSIN
#define GAIN_SIGNALS      m_KdSIN << m_KfSIN
#define STATE_SIGNALS     m_base6d_encodersSIN << m_jointsVelocitiesSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_FORCE_SIGNALS << \
                          FORCE_SIGNALS << GAIN_SIGNALS << m_controlledJointsSIN << m_dampingSIN

#define FORCE_ERROR_SIGNALS m_fRightFootErrorSOUT << m_fLeftFootErrorSOUT << m_fRightHandErrorSOUT << m_fLeftHandErrorSOUT
#define OUTPUT_SIGNALS      m_qDesSOUT << m_dqDesSOUT << FORCE_ERROR_SIGNALS*/

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef TemperatureController EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TemperatureController,
                                         "TemperatureController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      TemperatureController::TemperatureController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(q,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(qDot,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(temp_motor,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(tau,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(temp_ambiant,  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(qDes,         dynamicgraph::Vector, STATE_SIGNALS<<
                                                                FORCE_SIGNALS<<
                                                                REF_FORCE_SIGNALS<<
                                                                GAIN_SIGNALS<<
                                                                m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(dqDes,            dynamicgraph::Vector, m_uDesSOUT)
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        /*addCommand("getUseJacobianTranspose",
                   makeDirectGetter(*this,&m_useJacobianTranspose,
                                    docDirectGetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse","bool")));
        addCommand("setUseJacobianTranspose",
                   makeDirectSetter(*this, &m_useJacobianTranspose,
                                    docDirectSetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse",
                                                    "bool")));*/
        addCommand("init",
                   makeCommandVoid1(*this, &AdmittanceController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
      }

      void TemperatureController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_KdSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
        if(!m_KfSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kf is not plugged", MSG_TYPE_ERROR);
        if(!m_controlledJointsSIN.isPlugged())
          return SEND_MSG("Init failed: signal controlledJoints is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
        m_qDes.setZero(N_JOINTS);
        m_dqDes.setZero(N_JOINTS);
        m_q.setZero();
        m_dq.setZero();



        m_initSucceeded = true;
        m_firstIter = true;
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(u,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal qDes before initialization!");
          return s;
        }

        const Eigen::VectorXd& q =             m_base6d_encodersSIN(iter);
        if(m_firstIter)
        {
          // at the fist iteration store the joint positions as desired joint positions
          m_qDes = q.tail<N_JOINTS>();
          m_firstIter = false;
        }

        const Eigen::VectorXd& dqDes  =                m_dqDesSOUT(iter); // n
        const Eigen::VectorXd& qMask =       m_controlledJointsSIN(iter); // n

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
        {
          // if the joint is active integrate the desired velocity to get the new desired position
          if(qMask[i]!=0.0)
            m_qDes(i) += m_dt * dqDes(i);
          // if the joint is not active just check whether the desired joint position is too far
          // from the measured position, if this is the case it probably means that another
          // controller is moving that joint, so we update the desired joint position to the measured
          // position (@todo check this only when a joint switch from unactive to active)
          else if(fabs(q(6+i)-m_qDes(i))>DEFAULT_MAX_DELTA_Q)
          {
            m_qDes(i) = q(6+i);
//            SEND_MSG("Resetting qDes for joint "+JointUtil::get_name_from_id(i)+" because it was "+
//                     "too far from q,  q-qDes="+toString(q(6+i)-m_qDes(i)), MSG_TYPE_WARNING);
          }
        }

	s = m_qDes;
        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      /*void TemperatureController::display(std::ostream& os) const
      {
        os << "TemperatureController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void TemperatureController::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotTemperatureController:\n"
              << "\t -." << std::endl;
          Entity::commandLine(cmdLine, cmdArgs, os);
        }
        else
        {
          Entity::commandLine(cmdLine,cmdArgs,os);
        }
      }*/

      //**************************************************************************************************

    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph
