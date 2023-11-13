/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoySpdPolicy.cpp                                  */
/*    DATE: June 29th, 2021, broken out to separate class        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "ConvoySpdPolicy.h"
#include "MBUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

ConvoySpdPolicy::ConvoySpdPolicy()
{
  m_full_stop_convoy_rng = -1;
  m_slower_convoy_rng    = -1;
  m_ideal_convoy_rng     = -1;
  m_faster_convoy_rng    = -1;
  m_full_lag_convoy_rng  = -1;

  m_max_compression = 0.9;
  m_lag_speed_delta = -1;

  m_correction_mode = "unset";
}

//-----------------------------------------------------------
// Procedure: setFullStopConvoyRng()

void ConvoySpdPolicy::setFullStopConvoyRng(double dval)
{
  m_full_stop_convoy_rng = dval;
}

//-----------------------------------------------------------
// Procedure: setSlowerConvoyRng()

void ConvoySpdPolicy::setSlowerConvoyRng(double dval)
{
  m_slower_convoy_rng = dval;
}

//-----------------------------------------------------------
// Procedure: setIdealConvoyRng()

void ConvoySpdPolicy::setIdealConvoyRng(double dval)
{
  m_ideal_convoy_rng = dval;
}

//-----------------------------------------------------------
// Procedure: setFasterConvoyRng()

void ConvoySpdPolicy::setFasterConvoyRng(double dval)
{
  m_faster_convoy_rng = dval;
}

//-----------------------------------------------------------
// Procedure: setFullLagConvoyRng()

void ConvoySpdPolicy::setFullLagConvoyRng(double dval)
{
  m_full_lag_convoy_rng = dval;
}

//-----------------------------------------------------------
// Procedure: setMaxCompression()
//      Note: The max_compression is the greatest allowable
//            compression. By default it is 0.9. It can be
//            made smaller, all the way down to zero which
//            would make this policy uncompressible. It can
//            not be made more compressible than 0.9.
//      Note: If the given value is outside the allowable
//            range, it is mererly clipped.

void ConvoySpdPolicy::setMaxCompression(double dval)
{
  m_max_compression = dval;
  
  if(m_max_compression < 0)
    m_max_compression = 0;
  if(m_max_compression > 0.9)
    m_max_compression = 0.9;
}

//-----------------------------------------------------------
// Procedure: setLagSpeedDelta()

void ConvoySpdPolicy::setLagSpeedDelta(double dval)
{
  m_lag_speed_delta = dval;
}

//-----------------------------------------------------------
// Procedure: setParam()

bool ConvoySpdPolicy::setParam(string param, string value) 
{
  bool handled = false;
  if(param == "full_stop_convoy_range")
    handled = setPosDoubleOnString(m_full_stop_convoy_rng, value);
  else if(param == "slower_convoy_range")
    handled = setPosDoubleOnString(m_slower_convoy_rng, value);
  else if(param == "ideal_convoy_range")
    handled = setPosDoubleOnString(m_ideal_convoy_rng, value);
  else if(param == "faster_convoy_range")
    handled = setPosDoubleOnString(m_faster_convoy_rng, value);
  else if(param == "full_lag_convoy_range")
    handled = setPosDoubleOnString(m_full_lag_convoy_rng, value);

  else if(param == "lag_speed_delta")
    handled = setPosDoubleOnString(m_lag_speed_delta, value);
  else if(param == "policy_name")
    handled = setNonWhiteVarOnString(m_policy_name, value);

  return(handled);
}

//-----------------------------------------------------------
// Procedure: status()

string ConvoySpdPolicy::status() 
{
  // If ideal convoy range not set, make it the midpoint between
  // faster and slower ranges.
  if(m_ideal_convoy_rng == -1) {
    double span = m_faster_convoy_rng - m_slower_convoy_rng;
    if(span >= 0)
      m_ideal_convoy_rng = m_slower_convoy_rng + (span/2);
  }

  if(m_full_stop_convoy_rng <= 0)
    return("full_stop_convoy_rng not set or invalid");
  if(m_slower_convoy_rng < m_full_stop_convoy_rng) 
    return("slower_convoy_rng not set or invalid");
  if(m_ideal_convoy_rng < m_slower_convoy_rng) 
    return("ideal_convoy_rng not set or invalid");
  if(m_faster_convoy_rng < m_ideal_convoy_rng) 
    return("faster_convoy_rng not set or invalid");
  if(m_full_lag_convoy_rng < m_faster_convoy_rng)
    return("full_lag_convoy_rng not set or invalid");
  
  return("ok");
}

//-----------------------------------------------------------
// Procedure: updateSetSpeed()
//     Notes: contact_rng: Straight-line (as the crow flies)
//                         distance to the contact
//            convoy_rng:  Length of tail + distance of ownship
//                         to the aft marker.
//            lead_speed:  Speed of the moving contact.
//
//   Purpose: Decide speed based on the policy and input params
// 
//   cn    fstop     slower       ideal      faster     full_lag
//   o       o          o           o           o          o
//      #1       #2           #3          #4         #5      #6


double ConvoySpdPolicy::getSpdFromPolicy(double lead_speed,
					 double contact_rng, 
					 double convoy_rng)
{
  // Case 1 Full stop now
  if(contact_rng <= m_full_stop_convoy_rng) {
    m_correction_mode = "full_stop";
    return(0);
  }

  // Case 2 Slower proportionally
  if(convoy_rng <= m_slower_convoy_rng) {
    m_correction_mode = "close";
    double span = m_slower_convoy_rng - m_full_stop_convoy_rng;
    if(span <= 0)
      return(0);

    double pct = (convoy_rng - m_full_stop_convoy_rng) / span;
    return(pct * lead_speed);
  }

  // Case 3 and 4 Match contact speed
  if(convoy_rng <= m_ideal_convoy_rng) {
    m_correction_mode = "ideal_close";
    return(lead_speed);
  }

  // Case 3 and 4 Match contact speed
  if(convoy_rng <= m_faster_convoy_rng) {
    m_correction_mode = "ideal_far";
    return(lead_speed);
  }

  // Case 5 Speed up proportionally
  if(convoy_rng <= m_full_lag_convoy_rng) {
    m_correction_mode = "far";
    double span = m_full_lag_convoy_rng - m_faster_convoy_rng;
    if(span <= 0)
      return(lead_speed);
    else {
      double pct = (convoy_rng - m_faster_convoy_rng) / span;
      return(lead_speed + (pct * m_lag_speed_delta));
    }
  }

  // Case 6 Speed up as much as allowable
  m_correction_mode = "full_lag";
  double full_lag_speed = lead_speed + m_lag_speed_delta;
  return(full_lag_speed);      
}

//-----------------------------------------------------------
// Procedure: compress()

bool ConvoySpdPolicy::compress(double pct)
{
  if((pct < 0) || (pct > m_max_compression))
    return(false);
  if(status() != "ok")
    return(false);

  double slower_span = m_slower_convoy_rng - m_full_stop_convoy_rng;
  double idealA_span = m_ideal_convoy_rng - m_slower_convoy_rng;
  double idealB_span = m_faster_convoy_rng - m_ideal_convoy_rng;
  double faster_span = m_full_lag_convoy_rng - m_faster_convoy_rng;
  
  double new_slower_span = (1-pct) * slower_span;  
  double new_idealA_span = (1-pct) * idealA_span;  
  double new_idealB_span = (1-pct) * idealB_span;  
  double new_faster_span = (1-pct) * faster_span;  

  m_slower_convoy_rng = m_full_stop_convoy_rng + new_slower_span;
  m_ideal_convoy_rng  = m_slower_convoy_rng + new_idealA_span;
  m_faster_convoy_rng = m_ideal_convoy_rng + new_idealB_span;
  m_full_lag_convoy_rng = m_faster_convoy_rng + new_faster_span;

  return(true);
}


//-----------------------------------------------------------
// Procedure: getTerse()
//   Example: [5]    10--------20--------30      [50]   {2.0}

string ConvoySpdPolicy::getTerse() const
{
  string str = "[" + doubleToStringX(m_full_stop_convoy_rng,2) + "]     ";;
  str += doubleToStringX(m_slower_convoy_rng,2);
  str += "----------";
  str += "(" + doubleToStringX(m_ideal_convoy_rng,2) + ")";
  str += "----------";
  str += doubleToStringX(m_faster_convoy_rng,2) + "      ";
  str += "[" + doubleToStringX(m_full_lag_convoy_rng,2) + "]    ";
  str += "{" + doubleToStringX(m_lag_speed_delta,2) + "}";

  return(str);
}


//-----------------------------------------------------------
// Procedure: atIdealConvoyRng()

bool ConvoySpdPolicy::atIdealConvoyRng(double convoy_rng)
{
  if(convoy_rng > m_faster_convoy_rng)
    return(false);
  if(convoy_rng < m_slower_convoy_rng)
    return(false);

  return(true);
}


//-----------------------------------------------------------
// Procedure: getSpec()

string ConvoySpdPolicy::getSpec() const
{
  string spec = "full_stop_rng=" + doubleToStringX(m_full_stop_convoy_rng,2);

  if(m_vname != "")
    spec += ",vname=" + m_vname;
  
  spec += ",slower_rng=" + doubleToStringX(m_slower_convoy_rng,2);
  spec += ",ideal_rng=" + doubleToStringX(m_ideal_convoy_rng,2);
  spec += ",faster_rng=" + doubleToStringX(m_faster_convoy_rng,2);
  spec += ",full_lag_rng=" + doubleToStringX(m_full_lag_convoy_rng,2);
  spec += ",lag_spd_delta=" + doubleToStringX(m_lag_speed_delta,2);
  spec += ",max_compress=" + doubleToStringX(m_max_compression,2);

  if(m_policy_name != "")
    spec += ",name=" + m_policy_name;
  
  return(spec);
}

//-----------------------------------------------------------
// Procedure: string2ConvoySpdPolicy()

ConvoySpdPolicy string2ConvoySpdPolicy(string msg)
{
  ConvoySpdPolicy new_policy;

  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    double dval = atof(value.c_str());
    
    if(param == "vname")
      new_policy.setVName(value);
    else if(param == "full_stop_rng")
      new_policy.setFullStopConvoyRng(dval);
    else if(param == "slower_rng")
      new_policy.setSlowerConvoyRng(dval);
    else if(param == "ideal_rng")
      new_policy.setIdealConvoyRng(dval);
    else if(param == "faster_rng")
      new_policy.setFasterConvoyRng(dval);
    else if(param == "full_lag_rng")
      new_policy.setFullLagConvoyRng(dval);
    else if(param == "lag_spd_delta")
      new_policy.setLagSpeedDelta(dval);
    else if(param == "max_compress")
      new_policy.setMaxCompression(dval);
    else if(param == "name")
      new_policy.setPolicyName(value);
  }

  return(new_policy);
}


