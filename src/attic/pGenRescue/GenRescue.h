/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.h                                          */
/*    DATE: March 9th, 2023                             */
/************************************************************/

#ifndef GenRescue_HEADER
#define GenRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include <_types/_uint32_t.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <time.h>
#include <vector>

class GenRescue : public AppCastingMOOSApp {
public:
  GenRescue();
  ~GenRescue();
  std::vector<XYPoint> sort_points(XYPoint init_point);
  void post_pulse(double x, double y);
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
  bool m_all_points_received = true;
  double m_latest_capture_distance;
  int32_t m_unique_ids = 0;
  double m_current_x, m_current_y;
  bool m_captured_x = false, m_captured_y = false;
  bool m_successful_capture = false;
  double m_visit_radius;
  bool m_new_point = false;
  XYPoint m_latest_captured_point;

  std::string m_vname;
  std::set<std::string> m_ids;
  uint64_t m_ticks_since_update = 0;
  FILE *fptr;
  char buffer[120];

  int m_state;
  uint32_t m_capture_idx = 0;
  int m_active_mode = 0;
  int m_monitor_mode = 1;
  int m_idle_mode = 2;
};

#endif
