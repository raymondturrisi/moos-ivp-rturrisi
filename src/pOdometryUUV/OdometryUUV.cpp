/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "OdometryUUV.h"
#include <cmath>
using namespace std;

//---------------------------------------------------------
// Constructor()

OdometryUUV::OdometryUUV()
{
  m_first_reading = false;
  m_current_x = 0;
  m_current_y = 0;
  m_previous_x = 0;
  m_previous_y = 0;
  m_delta_x = 0;
  m_delta_y = 0;
  m_total_distance = 0;
  m_depth_threshold = 0;
}

//---------------------------------------------------------
// Destructor

OdometryUUV::~OdometryUUV()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool OdometryUUV::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_X") 
     {
      if(m_current_depth >= m_depth_threshold) {
        m_previous_x = m_current_x;
        m_current_x = msg.GetDouble();
        if ((m_previous_x_depth > m_depth_threshold) && (m_previous_y_depth > m_depth_threshold)) {
          m_delta_x += m_current_x-m_previous_x;
        }
      }
     }
     else if (key == "NAV_Y")
     {
      if(m_current_depth >= m_depth_threshold) {
        m_previous_y = m_current_y;
        m_current_y = msg.GetDouble();
        if ((m_previous_x_depth > m_depth_threshold) && (m_previous_y_depth > m_depth_threshold)) {
          m_delta_y += m_current_y-m_previous_y;
        }
      }
     }
     else if (key == "NAV_DEPTH")
     {
        double depth = msg.GetDouble();
        m_previous_depth = m_current_depth;
        m_previous_y_depth = m_current_depth;
        m_previous_x_depth = m_current_depth;
        m_current_depth = msg.GetDouble();
     }
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
  if (((m_current_x != m_previous_x) || 
      (m_current_y != m_previous_y) || 
      (m_current_depth != m_previous_depth)) &&
      (m_first_reading == false)) {
        m_first_reading = true;
      }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool OdometryUUV::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool OdometryUUV::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  if (m_first_reading) {
    m_total_distance += sqrt(pow(m_delta_x, 2.0)+pow(m_delta_y, 2.0));
    Notify("ODOMETRY_DIST", m_total_distance);
    Notify("ODOMETRY_DIST_AT_DEPTH", m_current_depth - m_depth_threshold);
    m_delta_x = 0;
    m_delta_y = 0;
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool OdometryUUV::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = true;
    if(param == "depth_thresh") {
      m_depth_threshold = atof(value.c_str());
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  Notify("ODOMETRY_DIST", 0.0);
  Notify("ODOMETRY_DIST_AT_DEPTH", 0.0);
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void OdometryUUV::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_DEPTH", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool OdometryUUV::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




