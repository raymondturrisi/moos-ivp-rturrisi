/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include "BHV_ZigLeg.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include <cstdlib>
#include <iterator>
#include <string>
#include "ZAIC_PEAK.h"
using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) : IvPBehavior(domain)
{
  // Provide a default behavior name
  //
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("WPT_INDEX", "NAV_HEADING");
  addInfoVars("NAV_X, NAV_Y");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if ((param == "foo") && isNumber(val))
  {
    // Set local member variables here
    return (true);
  }
  else if (param == "zig_duration")
  {
    m_zig_duration = double_val;
    return (true);
  }
  else if (param == "zig_angle")
  {
    m_zig_angle = double_val;
    return (true);
  }
  // If not handled above, then just return false;
  return (false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ZigLeg::onSetParamComplete() {}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart() {}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState() {}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState() {}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus() {}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState() {}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState() {}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction *BHV_ZigLeg::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = nullptr;
  bool ok = false;
  string wpt_idx = IvPBehavior::getBufferStringVal("WPT_INDEX", ok);
  if (!ok) {
    postWMessage("No waypoint posted yet.");
    return 0;
  }
  double osx = IvPBehavior::getBufferDoubleVal("NAV_X");
  double osy = IvPBehavior::getBufferDoubleVal("NAV_Y");
  double osh = IvPBehavior::getBufferDoubleVal("NAV_HEADING");
  double current_time = getBufferCurrTime();

  if (0 != wpt_idx.compare(m_old_wpt_idx))
  {
    m_old_wpt_idx = wpt_idx;
    m_zig_init_time = current_time;
    m_zig_boot_time = current_time + 5;
    m_boot = false;
  }
  if ((current_time >= m_zig_boot_time) && !m_boot) {
    m_init_heading = osh;
    m_boot = true;
  } else if (m_boot && ((m_zig_init_time + m_zig_duration) > current_time))
  {
    /*
      If the waypoint idx is the same, for the duration we skew the desired heading
    */
    ZAIC_PEAK crs_zaic(m_domain, "course");
    crs_zaic.setSummit(m_init_heading + m_zig_angle);
    crs_zaic.setPeakWidth(0);
    crs_zaic.setBaseWidth(180.0);
    crs_zaic.setSummitDelta(0);
    crs_zaic.setValueWrap(true);
    ipf = crs_zaic.extractIvPFunction();
  }
  if (ipf)
    ipf->setPWT(m_priority_wt);

  return (ipf);
}
