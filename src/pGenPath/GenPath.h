/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.h                                          */
/*    DATE: March 9th, 2023                             */
/************************************************************/

#ifndef GenPath_HEADER
#define GenPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>
#include <string>
#include "XYPoint.h"
#include "XYSegList.h"
#include <set>
#include <iostream>
#include <fstream>
#include <chrono>
#include <time.h>

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();
   std::vector<XYPoint> sort_points(XYPoint init_point);
   void post_pulse();
   void post_beam(bool captured, XYPoint missed);

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
 std::vector<XYPoint> m_points;
 std::vector<XYPoint> m_points_to_revisit;
 XYPoint m_last_missed_point;
 bool m_tour_complete = false;
 int m_missed_points = 0;
 bool m_all_points_received = false;
 double m_latest_capture_distance;

 double m_current_x, m_current_y;
 bool m_captured_x = false, m_captured_y = false;
 bool m_successful_capture = false;
 double m_visit_radius;

 XYPoint m_latest_captured_point;

 std::string m_vname;
 std::set<std::string> m_ids;
 uint64_t m_ticks_since_update = 0;
 FILE *fptr;
 char buffer [120];

 int m_state;

 int m_active_mode = 0;
 int m_monitor_mode = 1;
 int m_idle_mode = 2;
};

#endif 
