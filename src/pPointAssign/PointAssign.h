/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                          */
/*    DATE: March 9th, 2023                             */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>
#include <string>
#include "XYPoint.h"
#include <map>

class PointAssign : public AppCastingMOOSApp
{
 public:
   PointAssign();
   ~PointAssign();
  
 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();
   
 protected:
   void registerVariables();
   void postViewPoint(XYPoint point, std::string color);

 private: // Configuration variables
   std::vector<std::string> m_vehicles;
   
 private: // State variables

  std::vector<std::string> m_points_raw;
  std::vector<XYPoint> m_points;
  uint64_t m_num_points = 0;
  std::map<std::string, std::string> m_v_ready;
  std::map<std::string, std::vector<XYPoint>> m_points_to_send;
  std::map<std::string, std::string> m_colors;
  bool m_all_points = false;
  bool m_assign_by_region = false;
};

#endif 
