/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyMarker.cpp                                     */
/*    DATE: June 25th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cmath>
#include <cstdlib>
#include "ConvoyMarker.h"
#include "MBUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

ConvoyMarker::ConvoyMarker()
{
  m_x = 0;
  m_y = 0;
  m_id  = 0;
  m_set = false;
  m_utc = 0;
}

//-----------------------------------------------------------
// Procedure: Constructor

ConvoyMarker::ConvoyMarker(double x, double y, unsigned int id)
{
  m_x = x;
  m_y = y;
  m_id  = id;
  m_set = true;
  m_utc = 0;
}

//-----------------------------------------------------------
// Procedure: distToMarker()

double ConvoyMarker::distToMarker(double x, double y) 
{
  return(hypot(m_x-x, m_y-y));
}


//-----------------------------------------------------------
// Procedure: getSpec()

string ConvoyMarker::getSpec(string vname) const 
{
  string spec;
  spec =  "x="  + doubleToStringX(m_x, 2);
  spec += ",y=" + doubleToStringX(m_y, 2);
  spec += ",id=" + uintToString(m_id);

  if(vname != "")
    spec += ",vname=" + vname;
  else if(m_vname != "")
    spec += ",vname=" + m_vname;
  
  if(m_utc > 0)
    spec += ",time=" + doubleToString(m_utc,3);
  
  return(spec);
}


//-----------------------------------------------------------
// Procedure: string2ConvoyMarker()

ConvoyMarker string2ConvoyMarker(string str)
{
  ConvoyMarker marker;

  double x = 0;
  double y = 0;
  bool x_set = false;
  bool y_set = false;
  
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    if((param == "x") && isNumber(value)) {
      x = atof(value.c_str());
      x_set = true;
    }
    else if((param == "y") && isNumber(value)) {
      y = atof(value.c_str());
      y_set = true;
    }
    else if((param == "id") && isNumber(value)) {
      int int_id = atoi(value.c_str());
      unsigned int uint_id = 0;
      if(int_id >= 0)
	uint_id = (unsigned int)(int_id);
      marker.setID(uint_id);
    }
    else if(param == "vname")
      marker.setVName(value);
    else if((param == "utc") && isNumber(value)) {
      double utc = atof(value.c_str());
      if(utc > 0)
	marker.setUTC(utc);
    }
  }	 

  if(x_set && y_set)
    marker.setXY(x,y);
  
  if(marker.valid())
    return(marker);

  ConvoyMarker null_marker;
  return(null_marker);
}


