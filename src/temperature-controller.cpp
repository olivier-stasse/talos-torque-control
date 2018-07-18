/*
 * Copyright 2018, Florent Forget, LAAS-CNRS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the talos-torque-control project.
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
			  #define GAIN_SIGNALS      m_KdSIN << m_KfSIN */
#define STATE_SIGNALS     m_qSIN << m_qDotSIN << m_temp_motorSIN << m_tauSIN << m_temp_ambiantSIN
/*
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
            ,CONSTRUCT_SIGNAL_OUT(qDes,         dynamicgraph::Vector, STATE_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(dqDes,        dynamicgraph::Vector, STATE_SIGNALS)
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( STATE_SIGNALS );

        /* Commands. */
        /*addCommand("getUseJacobianTranspose",
                   makeDirectGetter(*this,&m_useJacobianTranspose,
                                    docDirectGetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse","bool")));
        addCommand("setUseJacobianTranspose",
                   makeDirectSetter(*this, &m_useJacobianTranspose,
                                    docDirectSetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse",
                                                    "bool")));*/
        addCommand("init",
                   makeCommandVoid1(*this, &TemperatureController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
      }

      void TemperatureController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_qSIN.isPlugged())
          return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);
        if(!m_qDotSIN.isPlugged())
          return SEND_MSG("Init failed: signal qDot is not plugged", MSG_TYPE_ERROR);
        if(!m_temp_motorSIN.isPlugged())
          return SEND_MSG("Init failed: signal temp_motor is not plugged", MSG_TYPE_ERROR);
        if(!m_tauSIN.isPlugged())
          return SEND_MSG("Init failed: signal tau is not plugged", MSG_TYPE_ERROR);
        if(!m_temp_ambiantSIN.isPlugged())
          return SEND_MSG("Init failed: signal temp_ambiant is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
	
        m_qDes.setZero(N_JOINTS);
      /*
        m_dqDesSOUT.setZero(N_JOINTS);
        m_qSIN.setZero();
        m_dqSIN.setZero();
	*/


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

        const Eigen::VectorXd& q =             m_qSIN(iter);
	const Eigen::VectorXd& qDes  =         m_qDesSOUT(iter); // n

        if(m_firstIter)
        {
          // at the fist iteration store the joint positions as desired joint positions
          m_qDes = q.tail<N_JOINTS>();
          m_firstIter = false;
        }

        const Eigen::VectorXd& dqDes  =                m_dqDesSOUT(iter); // n
        // const Eigen::VectorXd& qMask =       m_controlledJointsSIN(iter); // n

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
        {
          // if the joint is active integrate the desired velocity to get the new desired position
          //if(qMask[i]!=0.0)
	  m_qDes(i) = m_dt * dqDes(i) + qDes(i);
          // if the joint is not active just check whether the desired joint position is too far
          // from the measured position, if this is the case it probably means that another
          // controller is moving that joint, so we update the desired joint position to the measured
          // position (@todo check this only when a joint switch from unactive to active)
	  //else
          if(fabs(q(6+i)-qDes(i))>DEFAULT_MAX_DELTA_Q)
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
