/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.cpp                                     */
/*    DATE: March 9th, 2023                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenPath.h"
#include <algorithm>
#include "XYRangePulse.h"
#include "XYCommsPulse.h"

using namespace std;

double rel_dist(XYPoint p1, XYPoint p2)
{
  return sqrt(pow(p1.x() - p2.x(), 2.0) + pow(p1.y() - p2.y(), 2.0));
}

//---------------------------------------------------------
// Constructor()

GenPath::GenPath()
{
}

//---------------------------------------------------------
// Destructor

GenPath::~GenPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif


    //If we receive a new visit point, reset the timer from the last update, and collect the XYPoint in the points buffer
    if (key == "VISIT_POINT")
    {
      m_ticks_since_update = 0;
      string sval = msg.GetString();
      vector<string> contents = parseString(sval, ',');
      double x, y;
      string unique_id;
      for (auto idx = contents.begin(); idx != contents.end(); idx++)
      {
        string param = biteStringX(*idx, '=');
        string value = *idx;
        if (tolower(param) == "x")
        {
          x = stod(value);
        }
        else if (tolower(param) == "y")
        {
          y = stod(value);
        }
        else if (tolower(param) == "label")
        {
          unique_id = value;
        }
      }
      if (m_ids.count(unique_id) == 0)
      {
        m_points.push_back(XYPoint(x, y, unique_id));
        m_ids.insert(unique_id);
      }
    }
    //If the waypoint behavior claims to have captured a waypoint (by either direct capture or slip condition), we save this latest capture point
    else if (key == "HITPTS")
    {
      string sval = msg.GetString();
      vector<string> contents = parseString(sval, ',');
      double x, y;
      string unique_id;
      for (auto idx = contents.begin(); idx != contents.end(); idx++)
      {
        string param = biteStringX(*idx, '=');
        string value = *idx;
        if (tolower(param) == "x")
        {
          x = stod(value);
        }
        else if (tolower(param) == "y")
        {
          y = stod(value);
        }
      }
      //Here we also reset our condition for if it is a successful capture by the genpath criteria, to evaluate later in the iterate loop
      m_latest_captured_point = XYPoint(x, y, "CAPTURED");
      m_successful_capture = false;
    }
    else if (key == "DELIVERED")
    {
      //If all pPointAssign says that all points have been received, then we update our state manager to acknowledge this
      string sval = msg.GetString();
      if (sval == "true")
      {
        m_all_points_received = true;
      }
    }

    //If the waypoint behavior says that it completes a tour of the first round of points, we update our state manager
    else if (key == "TOUR_COMPLETE")
    {
      string sval = msg.GetString();
      if (sval == "true")
      {
        m_tour_complete = true;
      }
    }

    //Read the NAV messages and track the vehicles current state from uSimMarine
    else if (key == "NAV_X")
    {
      m_current_x = msg.GetDouble();
      m_captured_x = true;
    }
    else if (key == "NAV_Y")
    {
      m_current_y = msg.GetDouble();
      m_captured_y = true;
    }

    //No clue!
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenPath::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second


/**
 * @brief Releases a pulse surrounding the vehicle
 * 
 */
void GenPath::post_pulse()
{
  XYRangePulse my_pulse(m_current_x, m_current_y);
  my_pulse.set_label("Replanning");
  my_pulse.set_duration(6);
  my_pulse.set_edge_size(1);
  my_pulse.set_rad(50);
  my_pulse.set_fill(0.9);
  my_pulse.set_color("edge", "white");
  my_pulse.set_color("fill", "white");
  string str = my_pulse.get_spec();
  m_Comms.Notify("VIEW_RANGE_PULSE", str);
}


/**
 * @brief Draws a beam from the vehicle to a point its either captured or missed
 * 
 * @param captured 
 * @param target Target to trace the beam to
 */
void GenPath::post_beam(bool captured, XYPoint target)
{
  XYCommsPulse my_pulse(m_current_x, m_current_y, target.x(), target.y());
  my_pulse.set_duration(30);
  my_pulse.set_beam_width(10);
  my_pulse.set_fill(0.5);
  if(captured) {
    my_pulse.set_color("fill", "green");
    my_pulse.set_label("CAPTURED_POINT");
  } else {
    my_pulse.set_color("fill", "red");
    my_pulse.set_label("MISSED_POINT");
  }
  string str = my_pulse.get_spec();
  m_Comms.Notify("VIEW_COMMS_PULSE", str);
}

/**
 * @brief Sorts the points for the traveling salesman problem - returns a list of consecutively 
 * greedy close points to one another using the euclidean distance, i.e. if init is 7 and points are 9 10 4 2, it'll return
 * 7 9 10 4 2
 * 
 * @param init_point 
 * @return std::vector<XYPoint> Collection of XYPoints
 */

std::vector<XYPoint> GenPath::sort_points(XYPoint init_point)
{
  // Could be more efficient, but no problem on my machine! 

  // Instantiate a sequence to be returned
  std::vector<XYPoint> seq;
  uint16_t min_idx = -1;
  double min_dist = 999999999.0;

  // Find the first closest point in the set of points w/rt the initial point
  for (uint16_t i = 0; i < m_points.size(); i++)
  {
    double dist = rel_dist(init_point, m_points[i]);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }

  // Add these two points to the sequence
  seq.push_back(XYPoint(m_points[min_idx]));

  // Delete this from the set of points
  m_points.erase(m_points.begin() + min_idx);

  // Iterate through and consume the vector while creating a copy
  while (m_points.size() > 0)
  {
    // for each point, find the next closest point compared to the last minimum index
    uint16_t min_idx = -1;
    double min_dist = 999999999.0;
    for (uint16_t j = 0; j < m_points.size(); j++)
    {
      double dist = rel_dist(seq[seq.size() - 1], m_points[j]);
      // if we found a new minimum, update the minimum index
      if (dist < min_dist)
      {
        min_dist = dist;
        min_idx = j;
      }
    }

    // We found this next minimum
    seq.push_back(XYPoint(m_points[min_idx]));

    // Remove this from the source sequence - this what I would suspect to be inefficient, and we could probably swap indices, but am not
    m_points.erase(m_points.begin() + min_idx);
  }
  return seq;
};

bool GenPath::Iterate()
{
  /*
    On an iterate, we cycle between three states
      Active - Starts on active, and if sufficient conditions for posting the sorted sequence are met, we post this update, transition to monitor mode

      Monitor - while in monitor mode, when the waypoint behavior posts an update claiming it has captured a point by either capture radii or slip radii, we compare the point to 
        GenPaths visit point conditions, and determine whether or not it should be put back into the revisit queue

      Idle - If all points have been visited by our criteria, then we simply go to an idle mode
  */
  AppCastingMOOSApp::Iterate();

  XYPoint current_point(m_current_x, m_current_y);


  //ACTIVE MODE
  if (m_state == m_active_mode && m_points.size() != 0 && (m_ticks_since_update >= 50) && m_all_points_received && (m_captured_x && m_captured_y))
  {
    m_missed_points = 0;

    XYSegList m_seglist;

    m_points = sort_points(current_point);

    post_pulse();

    for (auto element : m_points)
    {
      m_seglist.add_vertex(XYPoint(element));
    }

    std::string color;
    if (m_vname.compare("HENRY"))
    {
      color = "yellow";
    }
    else
    {
      color = "blue";
    }
    m_seglist.set_edge_color(color);
    std::string update_str = "points = " + m_seglist.get_spec();

    Notify("UPDATES_WPTS", update_str);

    m_state = m_monitor_mode;
    m_points.clear();
  }
  //MONITOR MODE
  else if (m_state == m_monitor_mode && (m_captured_x && m_captured_y))
  {
    // Capture condition

    // Get the distance between the current point and the latest captured point
    m_latest_capture_distance = rel_dist(current_point, m_latest_captured_point);


    // If the capture distance is greater than the visit radius, and it is a new point, then push it to the queue
    if (!m_successful_capture && m_latest_capture_distance >= m_visit_radius && rel_dist(m_last_missed_point, m_latest_captured_point) > 1)
    {
      m_points_to_revisit.push_back(m_latest_captured_point);
      m_last_missed_point = XYPoint(m_latest_captured_point);

      post_beam(false, m_latest_captured_point);

      m_missed_points++;
    }
    else
    {
      post_beam(true, m_latest_captured_point);
      m_successful_capture = true;
    }

    //While monitoring, if the waypoint behavior claims that it completes the first queue, but by our criteria some points have been missed, then reconstruct our points queue and transition to active mode
    if (m_tour_complete && m_points_to_revisit.size() > 0)
    {
      for (auto point_it = m_points_to_revisit.rbegin(); m_points_to_revisit.size() > 0; point_it++)
      {
        m_points.push_back(XYPoint(*point_it));
        m_points_to_revisit.pop_back();
      }
      m_state = m_active_mode;
      m_tour_complete = false;
    }
    //Otherwise, if tour is complete but we have no points to revisit, we tell the vessel to return
    else if (m_tour_complete)
    {
      m_state = m_idle_mode; 
      Notify("RETURN", "true");
    }
  }
  //IDLE MODE
  else
  {
    //Nothing to do
  }

  m_ticks_since_update++;
  //fclose(fptr);
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;

  // DEFAULT VALUES

  m_vname = "NONAME";
  m_visit_radius = 10;
  m_current_x = -10;
  m_current_y = -10;
  m_tour_complete = false;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = true;
    if (param == "vehicle")
    {
      m_vname = value;
      handled = true;
    }
    else if (param == "visit_radius")
    {
      m_visit_radius = stod(value);
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_state = m_active_mode;

  registerVariables();
  //Notify shoreside that the vehicle has been brought online and is ready to receive points from pPointAssign
  Notify("READY", "true");

  //For debugging, prepare a file buffer
  time_t rawtime;
  struct tm *timeinfo;
  memset(buffer, '\0', 120);
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 120, "_%F_%T_data.txt", timeinfo);
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("DELIVERED", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("HITPTS", 0);
  Register("TOUR_COMPLETE", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenPath::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(1);

  actab << m_vname << '\n';
  actab.addHeaderLines();
  actab << "Visit Radius: " + to_string(m_visit_radius) + "\n";
  actab << "Number of points in queue:" + std::to_string(m_points.size()) + "\n";
  actab << "Nav_X/Y Received: " + std::to_string(m_captured_x & m_captured_y);
  actab << "Points Visited: N/A\n";
  actab << "Latest Capture Distance:" + std::to_string(m_latest_capture_distance) + "\n";
  actab << "Missed Points:" + std::to_string(m_missed_points) << '\n';
  m_msgs << actab.getFormattedString();

  return (true);
}
