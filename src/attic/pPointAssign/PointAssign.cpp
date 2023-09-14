/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                 */
/*    DATE: March 9th, 2023                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "PointAssign.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PointAssign::PointAssign()
{
}

//---------------------------------------------------------
// Destructor

PointAssign::~PointAssign()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
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
      string sval = msg.GetString();
      if (sval == "firstpoint" || sval == "lastpoint")
      {
        if (sval == "lastpoint")
        {
          m_all_points = true;
        }
        return true;
      }
      m_points_raw.push_back(sval);
      vector<string> contents = parseString(sval, ',');
      double x = 0, y = 0;
      string unique_id = "-1";
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
        else if (tolower(param) == "unique_id")
        {
          unique_id = value;
        }
      }

      if (m_assign_by_region) {
        //for only two vehicles
        if(x < 88) {
          // Cycle over the vehicles and the points that they will receive, and then add the point to their queue
          m_points_to_send[m_vehicles[1]].push_back(XYPoint(x, y, unique_id));
          m_num_points++;
        } else {
          m_points_to_send[m_vehicles[0]].push_back(XYPoint(x, y, unique_id));
          m_num_points++;
        }
      } else {
        // Cycle over the vehicles and the points that they will receive, and then add the point to their queue
        m_points_to_send[m_vehicles[m_num_points % m_vehicles.size()]].push_back(XYPoint(x, y, unique_id));
        m_num_points++;
      }
      
    }

    else if (key.find("READY") != std::string::npos)
    {
      for (auto v : m_vehicles)
      {
        if (key == "READY_" + v)
        {
          m_v_ready[v] = msg.GetString();
        }
      }
    }
    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool PointAssign::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

void PointAssign::postViewPoint(XYPoint point, string color)
{

  point.set_color("vertex", color); // yellow is handy on dark screen
  point.set_param("vertex_size", "10");
  point.set_label_color("white");
  string spec = point.get_spec(); // gets the string representation of a point
  Notify("VIEW_POINT", spec);
}

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // For each vehicle
  for (auto v : m_vehicles)
  {
    // Check if the vehicle is ready to receive its points
    if (m_v_ready.count(v) && m_v_ready[v] == "true")
    {
      // If its ready to receive its points, then we push a single point from its queue to the DB, and give it time to process future points while we work on the other vehicles
      if (m_points_to_send[v].size() > 0)
      {
        // Append the name to the visit point message
        string new_key = "VISIT_POINT_" + v;
        // Notify the MOOSDB
        Notify(new_key, m_points_to_send[v].back().get_spec());
        // Add a special message to render it in pMarineViewer
        postViewPoint(m_points_to_send[v].back(), m_colors[v]);
        // Pop this message from its queue and move onto a next message which can be processed
        m_points_to_send[v].pop_back();
      } else if (m_points_to_send[v].size() == 0 && m_all_points == true)
      {
        Notify("DELIVERED_" + v, "true");
      }
    }
    
    Notify("POINTS_"+v, m_points_to_send[v].size());
    Notify("ALL_POINTS", std::to_string(m_all_points));
  }
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PointAssign::OnStartUp()
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

    bool handled = false;
    if (param == "vehicles")
    {
      if ((line[0] == '{' || line[0] == '(' || line[0] == '[') &&
          (line.back() == '}' || line.back() == ')' || line.back() == ']'))
      {
        string line_clean = line.substr(1, line.length() - 2);
        m_vehicles = parseString(line_clean, ',');
        handled = true;
      }
      else
      {
        reportConfigWarning("Vehicles must be in a list, i.e. Vehicles = [HENRY, GILDA, ...]. Got: \n\t" + orig + "\n");
        handled = false;
      }
    }
    else if (param == "colors")
    {
      if (m_vehicles.size() == 0)
      {
        reportConfigWarning("Colors must be defined before vehicles\n");
        handled = false;
      }
      else
      {
        if ((line[0] == '{' || line[0] == '(' || line[0] == '[') &&
            (line.back() == '}' || line.back() == ')' || line.back() == ']'))
        {
          string line_clean = line.substr(1, line.length() - 2);
          std::vector<std::string> colors = parseString(line_clean, ',');
          for (int i = 0; i < m_vehicles.size(); i++)
          {
            m_colors[m_vehicles[i]] = colors[i];
          }
          handled = true;
        }
        else
        {
          reportConfigWarning("Colors must be in a list, i.e. Colors = [yellow, green, ...]. Got: \n\t" + orig + "\n");
          handled = false;
        }
      }
    }
    else if (param == "assign_by_region")
    {
      handled = true;
      if(tolower(value) == "true") {
        m_assign_by_region = true;
      } else if (tolower(value) == "false") {
        m_assign_by_region = false;
      } else {
        reportConfigWarning("Assign by region is not a clear boolean expression, i.e. assign_by_region = true. Got: \n\t" + orig + "\n");
        handled = false;
      }
      
    } 

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  Notify("UTS_HOLD_POINTS", "false");
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  for (auto v : m_vehicles)
  {
    Register("READY_" + v, 0);
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool PointAssign::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << m_vehicles[0] + " | " + m_vehicles[1] + "\n";
  actab.addHeaderLines();

  actab << std::to_string(m_points_to_send[m_vehicles[0]].size()) << std::to_string(m_points_to_send[m_vehicles[0]].size()) << "\n";
  actab << std::to_string(m_all_points) << std::to_string(m_all_points) << '\n';

  m_msgs << actab.getFormattedString();

  return (true);
}
