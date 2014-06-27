// -*- C++ -*-
/*!
 * @file  Controller.cpp
 * @brief Controller to drive a robot
 * @date $Date$
 *
 * $Id$
 */

#include "PA10Controller.h"

// Module specification
// <rtc-template block="module_spec">
static const char* controller_spec[] =
  {
    "implementation_id", "PA10Controller",
    "type_name",         "PA10Controller",
    "description",       "Controller to drive a robot",
    "version",           "1.0.0",
    "vendor",            "devRT",
    "category",          "Controller",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

PA10Controller::PA10Controller(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_anglesIn("angles", m_angles),
    m_velsIn("vels", m_vels),
    m_torqueOut("torque", m_torque),
    // </rtc-template>
    dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

}

PA10Controller::~PA10Controller()
{
}


RTC::ReturnCode_t PA10Controller::onInitialize()
{
  // Set InPort buffers
  addInPort("angles", m_anglesIn);
  addInPort("vels", m_velsIn);
  
  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);

  m_qRef.data.length(9);
  m_qRef.data[0] = 0.0;
  m_qRef.data[1] = 0.8;
  m_qRef.data[2] = 0.0;
  m_qRef.data[3] = 0.8;
  m_qRef.data[4] = 0.0;
  m_qRef.data[5] = 0.8;
  m_qRef.data[6] = 1.57;
  m_qRef.data[7] = 0.0;
  m_qRef.data[8] = 0.0;

  m_torque.data.length(9);
  for (size_t i = 0; i < m_torque.data.length(); i++){
    m_torque.data[i] = 0;
  }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PA10Controller::onExecute(RTC::UniqueId ec_id)
{
  if (m_anglesIn.isNew()) m_anglesIn.read();
  if (m_velsIn.isNew()) m_velsIn.read();
  if (!m_angles.data.length()) return RTC::RTC_OK;

#define P 800.0
#define D -100.0
#define MaxTau 200.0
  for (size_t i = 0; i < m_qRef.data.length(); i++){
    double t = P*(m_qRef.data[i] - m_angles.data[i])+D*m_vels.data[i];
    t = std::min( MaxTau, t);
    t = std::max(-MaxTau, t);
    m_torque.data[i] = t;
  }

  m_torqueOut.write(); 

  return RTC::RTC_OK;
}

extern "C"
{
 
  void PA10ControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(controller_spec);
    printf("PA10ControllerInit called");
    manager->registerFactory(profile,
                             RTC::Create<PA10Controller>,
                             RTC::Delete<PA10Controller>);
  }
  
};

