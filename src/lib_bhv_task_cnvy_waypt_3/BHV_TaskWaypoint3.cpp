/*****************************************************************/
/*    NAME: Raymond Turrisi                                      */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskWaypoint3.cpp                                */
/*    CIRC: November 2023                                        */
/*    CMTS: Dubins Turn Task Waypoint Metric                     */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <cmath>
#include "BHV_TaskWaypoint3.h"
#include "MBUtils.h"
#include "MacroUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_TaskWaypoint3::BHV_TaskWaypoint3(IvPDomain domain) :
  IvPTaskBehavior(domain)
{
  // Initialize state variables
  m_ptx = 0;
  m_pty = 0;
  m_turn_radius = 3;

  m_ptx_set = false;
  m_pty_set = false;
  m_turn_radius_set = false;

  m_consider_contacts = false;
}


//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskWaypoint3::onHelmStart()
{
  //if(m_update_var == "")
  //  return;

  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskWaypoint3::setParam(string param, string value) 
{
  if(IvPTaskBehavior::setParam(param, value))
    return(true);
  
  param = tolower(param);
  if((param == "waypt_x") && isNumber(value)) {
    m_ptx = atof(value.c_str());
    m_ptx_set = true;
    return(true);
  }
  else if((param == "waypt_y") && isNumber(value)) {
    m_pty = atof(value.c_str());
    m_pty_set = true;
    return(true);
  }
  else if((param == "turn_radius") && isNumber(value)) {
    m_turn_radius = atof(value.c_str());
    m_turn_radius_set = true;
    return(true);
  }
  else if(param == "waypt") {
    string xstr = biteStringX(value, ',');
    string ystr = value;
    if(isNumber(xstr) && isNumber(ystr)) {
      m_ptx = atof(xstr.c_str());
      m_ptx_set = true;
      m_pty = atof(ystr.c_str());
      m_pty_set = true;
      return(true);
    }
  }
  else if(param == "consider_contacts")
    return(setBooleanOnString(m_consider_contacts, value));
  
  return(false);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskWaypoint3::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
  return;
}

//-----------------------------------------------------------
// Procedure: onRunState()
//    States: alerted : noroster : roster : bidding :
//            bidwon : bidlost : abstain

IvPFunction *BHV_TaskWaypoint3::onRunState()
{
  IvPTaskBehavior::onGeneralRunState();
  return(0);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()

double pmod(double a, double b) {
  double r = fmod(a,b);
  if (r < 0) {
    r+=b;
  }
  return r;
};

double BHV_TaskWaypoint3::getTaskBid()
{
  // TODO: Place dubins turn predicted odometry here
  XYSegList predicted_trajectory;

  // Get the differences between the state and the desired point
  double dx = m_ptx - m_osx; // meters
  double dy = m_pty - m_osy; // meters

  double heading = pmod(450-m_osh, 360)*M_PI/180;
  // Angle difference between heading and heading to target
  double psi = pmod(atan2(dy, dx) - heading + M_PI, 2 * M_PI) - M_PI; // radians

  // Determine if we are turning left or right
  double ang_to_cor = heading; // radians
  double cw = -1;             // a logical representation as a double - are we turning 'clockwise'
  if (psi < 0)
  {
    ang_to_cor -= M_PI / 2;
    cw = -1;
  }
  else
  {
    ang_to_cor += M_PI / 2;
    cw = 1;
  }

  // Place a temporary turning radius
  double current_x = m_osx;
  double current_y = m_osy;
  double current_heading = heading;

  double center_x = current_x + m_turn_radius * cos(ang_to_cor);
  double center_y = current_y + m_turn_radius * sin(ang_to_cor);

  // If the point is within the radius of our center of rotation, we need to scoot forward and update our center of rotation, adding this to our odometer
  predicted_trajectory.add_vertex(current_x, current_y);

  double step_size = 0.5; // meters
  while (hypot(m_pty - center_y, m_ptx - center_x) < m_turn_radius)
  {

    current_x += step_size * cos(current_heading);
    current_y += step_size * sin(current_heading);

    center_x = current_x + m_turn_radius * cos(ang_to_cor);
    center_y = current_y + m_turn_radius * sin(ang_to_cor);
  }

  // If we did have to move forward at all, then just add the new point to the predicted trajectory
  if (hypot(current_y - m_osy, current_x - m_osx) > step_size / 2)
  {
    predicted_trajectory.add_vertex(current_x, current_y);
  }

  int steps = 1;

  double ang_step_size = 5*M_PI/180; //k degrees to radians (5 degree step size for evaluation)
  while (true)
  {
    double nang = ang_to_cor + M_PI + (cw * steps * ang_step_size);
    double next_point_x = center_x + m_turn_radius * cos(nang);
    double next_point_y = center_y + m_turn_radius * sin(nang);

    //Vector v - current point to target
    double dv_x = m_ptx - current_x;
    double dv_y = m_pty - current_y;

    //Vector u - current point to the next proposed point
    double du_y = next_point_y - current_y;
    double du_x = next_point_x - current_x;

    double ang_to_trg = atan2(dv_y, dv_x);
    double ang_to_np = atan2(du_y, du_x);

    //Angle between the vectors, off of the vector to to target
    double phi = ang_to_trg - ang_to_np;

    phi = pmod(phi + M_PI, 2 * M_PI) - M_PI;
    
    //If the angle between these vectors is interior to the circle, i.e. it passes the circle, then we need to step to this next point
    //Otherwise, if we have a straight shot to the target, then we completed the dubins turn

    //Or If we have done this over for over 1000 steps, we messed up and the algorithm is faulty
    if (cw*phi+ang_step_size < 0 || steps > 1000) {
      break;
    }

    current_x = next_point_x;
    current_y = next_point_y;
    predicted_trajectory.add_vertex(current_x,current_y);

    steps++;
  }

  // Once we have our center of rotation placed, we zip around the turning radius, discretely, determining whether or not we have a trajectory which does not intersect with the circle
  predicted_trajectory.add_vertex(m_ptx,m_pty);
  predicted_trajectory.set_color("edge","red");
  predicted_trajectory.set_duration(10);
  predicted_trajectory.set_label(m_us_name+"_dubins_wpt");
  postMessage("VIEW_SEGLIST", predicted_trajectory.get_spec());
  double dist = predicted_trajectory.length();
  return(dist);
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskWaypoint3::applyFlagMacros(vector<VarDataPair> flags)
{
  string ptx_str = doubleToStringX(m_ptx, 2);
  string pty_str = doubleToStringX(m_pty, 2);

  for(unsigned int i=0; i<flags.size(); i++) {
    if(flags[i].is_string()) {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "PTX", ptx_str);
      sdata = macroExpand(sdata, "PTY", pty_str);      
      flags[i].set_sdata(sdata, true);
    }
  }

  return(flags);
}





