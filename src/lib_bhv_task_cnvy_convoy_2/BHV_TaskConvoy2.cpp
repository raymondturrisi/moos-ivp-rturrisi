/*****************************************************************/
/*    NAME: Raymond Turrisi                                      */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskConvoy2.cpp                                  */
/*    CIRC: November 2023                                        */
/*    CMTS: Shallow bidding function which incorporates heading  */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include <cmath>
#include "BHV_TaskConvoy2.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "AngleUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskConvoy2::BHV_TaskConvoy2(IvPDomain domain) : IvPTaskBehavior(domain)
{
}


//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskConvoy2::onHelmStart()
{
  if(m_update_var == "")
    return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam

bool BHV_TaskConvoy2::setParam(string param, string param_val) 
{
  if(IvPTaskBehavior::setParam(param, param_val))
    return(true);

  return(false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskConvoy2::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: spawned : noroster : roster : bidding

IvPFunction *BHV_TaskConvoy2::onRunState()
{
  IvPTaskBehavior::onGeneralRunState();
  return(0);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()

double BHV_TaskConvoy2::getTaskBid()
{
  //Get the angle between ownship and the target
  double ang_to_target = atan2(m_cny - m_osy, m_cnx - m_osx);

  //Identify the angle between our current heading and the target - is between -pi and pi
  double phi = fmod(m_osh - ang_to_target + M_PI, 2*M_PI);

  //Compute the distance to the target
  double dist = hypot(m_osx - m_cnx, m_osy - m_cny);

  //The bid is the distance scaled by how complementary the angles are. In the two cases, the distance is scaled [dist*0.5, dist*1] on whether or not the agent is facing that direction, or away from, respectively
  double bid = dist*(3/4 - (1/4)*cos(phi));

  return(bid);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskConvoy2::applyFlagMacros(vector<VarDataPair> flags)
{
  for(unsigned int i=0; i<flags.size(); i++) {
    string var = flags[i].get_var();
    if(flags[i].is_string()) {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "CONTACT", tolower(m_contact));
      flags[i].set_sdata(sdata, true);
    }
  }

  return(flags);
}





