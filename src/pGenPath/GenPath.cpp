/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.cpp                                        */
/*    DATE: March 9th, 2023                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenPath.h"
#include <algorithm>

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
    else if (key == "DELIVERED")
    {
      string sval = msg.GetString();
      if (sval == "true")
      {
        m_all_points_received = true;
      }
    }
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

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  fptr = fopen((m_vname + std::string(buffer)).c_str(), "a");
  fprintf(fptr, "On iterate - ticks since update %llu\n", m_ticks_since_update);
  if (active && m_ticks_since_update > 100 && m_points.size() != 0 && m_all_points_received && (m_captured_x && m_captured_y))
  {
    std::vector<XYPoint> seq;
    XYSegList m_seglist;
    XYPoint current_point(m_current_x, m_current_y);

    // find the first closest point to the current location
    uint16_t min_idx = -1;

    double min_dist = 999999999.0;
    fprintf(fptr, "Num points: %lu\n", m_points.size());
    for (uint16_t i = 0; i < m_points.size(); i++)
    {
      fprintf(fptr, "i: %d\n", i);
      double dist = rel_dist(current_point, m_points[i]);
      fprintf(fptr, "Init dist: %s -> %s = %s\n", current_point.get_spec().c_str(), m_points[i].get_spec().c_str(), std::to_string(dist).c_str());
      if (dist < min_dist)
      {
        fprintf(fptr, "dist < min dist: %f %f\n", dist, min_dist);
        min_dist = dist;
        min_idx = i;
      }
    }

    // add these two points to the seglist
    seq.push_back(XYPoint(current_point));
    seq.push_back(XYPoint(m_points[min_idx]));

    m_seglist.add_vertex(XYPoint(current_point));
    m_seglist.add_vertex(XYPoint(m_points[min_idx]));
    fprintf(fptr, "First two points in seglist: %s\n", m_seglist.get_spec().c_str());
    m_points.erase(m_points.begin() + min_idx);

    // update the most recent minimum variable/index
    while (m_points.size() > 0)
    {
      // for each point, find the next closest point compared to the last minimum index
      uint16_t min_idx = -1;
      double min_dist = 999999999.0;
      for (uint16_t j = 0; j < m_points.size(); j++)
      {

        double dist = rel_dist(seq[seq.size() - 1], m_points[j]);
        fprintf(fptr, "j %d: %s -> %s = %s\n", j, seq[seq.size() - 1].get_spec().c_str(), m_points[j].get_spec().c_str(), std::to_string(dist).c_str());
        // if we found a new minimum, update the minimum index
        if (dist < min_dist)
        {
          min_dist = dist;
          min_idx = j;
        }
      }

      fprintf(fptr, "Min idx: %d - %s\n", min_idx, m_points[min_idx].get_spec().c_str());

      // take this next found minimum, add it to the seglist
      seq.push_back(XYPoint(m_points[min_idx]));

      m_seglist.add_vertex(XYPoint(m_points[min_idx]));

      fprintf(fptr, "Seglist: %s\n", m_seglist.get_spec().c_str());

      // erase the point which is two-points old from the list
      m_points.erase(m_points.begin() + min_idx);
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
    fprintf(fptr, "Final Seglist: %s\n", update_str.c_str());

    Notify("UPDATES_WPTS", update_str);

    fclose(fptr);
    active = false;
  }
  AppCastingMOOSApp::PostReport();
  m_ticks_since_update++;
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

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  Notify("READY", "true");
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
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenPath::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(1);

  /*
    Visit Radius
    Total Points Received
    NAV_X/Y Received

    Tour Status
    Points Visited
    Points Unvisited
  */
  actab << m_vname << '\n';
  actab.addHeaderLines();
  actab << "Visit Radius: N/A \n";
  actab << "Number of points in queue:" + std::to_string(m_points.size()) + "\n";
  actab << "Nav_X/Y Received: " + std::to_string(m_captured_x & m_captured_y);
  actab << "Points Visited: N/A\n";
  actab << "Points Unvisited: N/A\n";
  m_msgs << actab.getFormattedString();

  return (true);
}
