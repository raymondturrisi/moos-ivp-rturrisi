/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyStatRecap.cpp                                  */
/*    DATE: July 17th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "ConvoyStatRecap.h"
#include "MBUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ConvoyStatRecap::ConvoyStatRecap()
{
  m_ideal_rng = -1;
  m_compression = 0;
  m_index = 0;
  m_idle = false;
}

//---------------------------------------------------------------
// Procedure: getStringValue()

string ConvoyStatRecap::getStringValue(string key) const
{
  key = tolower(key);
  if(key == "ideal_rng")
    return(doubleToStringX(m_ideal_rng, 2));
  else if(key == "compression")
    return(doubleToStringX(m_compression, 2));
  else if(key == "idle")
    return(boolToString(m_idle));
  else
    return("");  
}

//------------------------------------------------------------
// Procedure: getSpec()

string ConvoyStatRecap::getSpec() const
{
  if(m_follower == "")
    return("");
  
  string str = "follower=" + m_follower;

  if(isSetLeader())
    str += ",leader=" + m_leader;
  if(isSetIdealRng())
    str += ",ideal_rng=" + doubleToString(m_ideal_rng,2);
  if(m_compression > 0)
    str += ",compression=" + doubleToString(m_compression,2);
  if(m_index > 0)
    str += ",index=" + uintToString(m_index);
  if(m_idle)
    str += ",idle=true";
  
  return(str);
}

//---------------------------------------------------------
// Procedure: string2ConvoyStatRecap()
//   Example: follower=henry,leader=abe,ideal_rng=40,
//            compression=0.4,index=23

ConvoyStatRecap string2ConvoyStatRecap(string msg)
{
  ConvoyStatRecap new_recap;

  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    double dval = atof(value.c_str());
    
    if(param == "follower")
      new_recap.setFollower(value);
    else if(param == "leader")
      new_recap.setLeader(value);
    else if(param == "ideal_rng")
      new_recap.setIdealRng(dval);
    else if(param == "compression")
      new_recap.setCompression(dval);
    else if(param == "idle")
      new_recap.setIdle(tolower(value) == "true");
    else if(param == "index")
      new_recap.setIndex((unsigned int)(dval));
  }

  return(new_recap);
}


