/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: OdometryUUV.h                                   */
/*    CIRCA: Spring 2023                                    */
/************************************************************/

#ifndef OdometryUUV_HEADER
#define OdometryUUV_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class OdometryUUV : public AppCastingMOOSApp
{
 public:
   OdometryUUV();
   ~OdometryUUV();

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
 bool m_first_reading;
 double m_current_x;
 double m_current_y;
 double m_current_depth;
 double m_previous_x;
 double m_previous_x_depth;
 double m_previous_y;
 double m_previous_y_depth;
 double m_previous_depth;
 double m_total_distance;
 double m_delta_x;
 double m_delta_y;
 double m_depth_threshold;
};

#endif 
