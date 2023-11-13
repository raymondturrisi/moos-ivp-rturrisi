/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskConvoy.cpp                                   */
/*    DATE: Dec 19th 2020                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include <cmath>
#include "BHV_TaskConvoy1.h"
#include "MBUtils.h"
#include "MacroUtils.h"
#include "AngleUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskConvoy::BHV_TaskConvoy(IvPDomain domain) : IvPTaskBehavior(domain)
{
}


//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskConvoy::onHelmStart()
{
  if(m_update_var == "")
    return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam

bool BHV_TaskConvoy::setParam(string param, string param_val) 
{
  if(IvPTaskBehavior::setParam(param, param_val))
    return(true);

  return(false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskConvoy::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: spawned : noroster : roster : bidding

IvPFunction *BHV_TaskConvoy::onRunState()
{
  IvPTaskBehavior::onGeneralRunState();
  return(0);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()

double BHV_TaskConvoy::getTaskBid()
{
  //TODO: Place feedforward dynamic model here
  double dist = hypot(m_osx - m_cnx, m_osy - m_cny);
  return(dist);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskConvoy::applyFlagMacros(vector<VarDataPair> flags)
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





