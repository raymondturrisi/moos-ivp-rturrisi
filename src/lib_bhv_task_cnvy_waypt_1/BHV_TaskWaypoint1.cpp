/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskWaypoint1.cpp                                */
/*    DATE: Dec 5th 2018                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <cmath>
#include "BHV_TaskWaypoint.h"
#include "MBUtils.h"
#include "MacroUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskWaypoint::BHV_TaskWaypoint(IvPDomain domain) :
  IvPTaskBehavior(domain)
{
  // Initialize state variables
  m_ptx = 0;
  m_pty = 0;

  m_ptx_set = false;
  m_pty_set = false;

  m_consider_contacts = false;
}


//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskWaypoint::onHelmStart()
{
  //if(m_update_var == "")
  //  return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskWaypoint::setParam(string param, string value) 
{
  if(IvPTaskBehavior::setParam(param, value))
    return(true);
  
  param = tolower(param);
  if((param == "waypt_x") && isNumber(value)) {
    m_ptx = atof(value.c_str());
    m_ptx_set = true;
    return(true);
  }
  else if((param == "waypt_y") && isNumber(value)) {
    m_pty = atof(value.c_str());
    m_pty_set = true;
    return(true);
  }
  else if(param == "waypt") {
    string xstr = biteStringX(value, ',');
    string ystr = value;
    if(isNumber(xstr) && isNumber(ystr)) {
      m_ptx = atof(xstr.c_str());
      m_ptx_set = true;
      m_pty = atof(ystr.c_str());
      m_pty_set = true;
      return(true);
    }
  }
  else if(param == "consider_contacts")
    return(setBooleanOnString(m_consider_contacts, value));
  
  return(false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskWaypoint::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: alerted : noroster : roster : bidding :
//            bidwon : bidlost : abstain

IvPFunction *BHV_TaskWaypoint::onRunState()
{
  IvPTaskBehavior::onGeneralRunState();
  return(0);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()

double BHV_TaskWaypoint::getTaskBid()
{
  //TODO: Place feedforward odometry here
  double dist = hypot(m_osx - m_ptx, m_osy - m_pty);
  return(dist);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskWaypoint::applyFlagMacros(vector<VarDataPair> flags)
{
  string ptx_str = doubleToStringX(m_ptx, 2);
  string pty_str = doubleToStringX(m_pty, 2);

  for(unsigned int i=0; i<flags.size(); i++) {
    if(flags[i].is_string()) {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "PTX", ptx_str);
      sdata = macroExpand(sdata, "PTY", pty_str);      
      flags[i].set_sdata(sdata, true);
    }
  }

  return(flags);
}





