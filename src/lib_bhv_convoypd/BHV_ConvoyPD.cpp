/************************************************************/
/*    NAME: Raymond Turrisi                                 */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ConvoyPD.cpp                                */
/*    CIRC: November 2023                                   */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include <cmath>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BuildUtils.h"
#include "BHV_ConvoyPD.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include <sstream>
#include "NodeRecord.h"
#include "NodeMessage.h" // In the lib_ufield library

using namespace std;

/*
  This application is to serve as the foundation for the synchronization convoying control law.
  This one, will be slightly simpler, containing most of the necessary abstraction layers, for which
  a more abstract control law can be build on top of.

  This behavior will be concerned with/maintain information on all other agents in the convoy.
  For now, we assume the agents are all on the same team. This convoy policy will contain the
  following features

  - The leader sets a trajectory which all agents will follow, 'dropping' points which the leader
      maintains in its queue
  - When its follower acquires the point, it is removed from the leaders queue and placed in that
      agent's queue, and this happens up til the last agent
  - The last agent does not maintain a queue of points which its follower must acquire
  - A queue is a collection of points (x,y) which have encoded information, containing state information
      on the leader agent. Some examples may be: the leader agents heading, the leader agents turning
      rate, the leader agents velocity at the time of the turn and the time in which the point was placed
  - A queue are the points which the current agent needs to acquire, and when a point is acquired it is passed
      to its follower, which it then adds to its queue
  - The distance between all the points in the queue + an agents distance to its next point (+ the distance of
      the last point to the agent its following) provides the total distance between an agent in the trajectory
  - This queue and the distance serves as the nominal speed control law, where some deviation from the ideal
      follow range encourages a speed up or slow down
  - The error is squared so we have a convex function, rather than a V about the ideal speed (U)

                \         /
                 \       /
                  \     /
  0                \   /                  VMAX
  *------------------+-----|-------------*
                     |     |
                Ideal speed|
                           |
      Required catchup speed

  0                                      M
  *-----------------|------X-------------*
  Ideal Follow Range       |
                          Actual follow range
                    Err = Actual - ideal

  Positive error, encourages increased thrust

  The D term is to prevent overshoot and may not even be necessary
  An I term may be added to this controller, but won't be added to the synchronization controller


  The algorithm starts with knowing who the leader is, and who they are following, per the task behavior
  for who won the bid after an ordering. With this, the behavior then constructs an ordering of all the
  agents in the convoy. This is not immediately necessary for this PD speed policy algorithm, but we use it
  for analysis. We identify who all the contacts are, from the contact manager. Specifically, CONTACTS_LIST.
  We have to assume, that by the time this behavior is running, we have a full contact lists.

  Once we have a full ordering of all the agents,

  Mail
    FOLLOWER : Redundant to FOLLOW_BROADCAST - a direct message from an agent who is following us
    CONTACTS_LIST : Redundant and used to check if we received all the agents we're expecting
    FOLLOW_BROADCAST : Received from every agent which is in the convoy - need to assemble a register for all agents in the convoy
    IS_LEADER : Received from task_waypoint
    NEW_POINT : (x,y):t,u,theta,theta_dot

  Parameters
    desired_speed
    point_update_distance - How often the leader seeds a point

    kp_spd
    kd_spd
    ki_spd - optional

    kp_hdg
    kd_hdg
    ki_hdg - optional


  States
    Is leader
      For convoy PD, this behavior is passive, and simply sends points to its follower. It seeds points based on its odometry since last reported point

    Is follower
      Follower Types
        Is mid-follower
          Receives points from who its following, and while its capturing points, we are also sending them to the agent following us
        Is tail-follower
          Receives points from who its following, is responsible for closing the point, and providing the time in which a point is closed as an evaluation metric
            This agent also reports the nominal convoy speed

      Provides a heading policy w/rt to the vehicles current position to the next point in its queue
        If the queue is empty, they just follow the next agent
      Provides a speed policy based on the deviation in convoy length from the agent its following, and the desired speed set by the trajectory point
        The agent reports this deviation to all other agents
      When managing the queue, it monitors a capture radius and a slip condition - a slip condition is when the point is in the slip radius but behind the agent - the point is dropped

    NOTE/TODO:
      [ ] Need to make it so that if all bids aren't received, the task times out and they by default lose the bid
*/

//---------------------------------------------------------------
// Constructor

BHV_ConvoyPD::BHV_ConvoyPD(IvPDomain domain) : IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "convoy_pd");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for

  m_desired_speed = 1.2;       // meters per second
  m_point_update_distance = 1; // meters
  m_ideal_follow_range = 5;    // meters

  m_is_leader = false;
  m_is_midship = false;
  m_is_tail = false;
  m_contact = "NULL";
  m_follower = "NULL";
  m_contact_list_str = "";
  m_has_broadcast_contact = false;

  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_osx_prv = 0;
  m_osy_prv = 0;
  m_osh_prv = 0;
  m_interval_odo = 0;                           // meters
  m_eps = 0.01;                                 // Meters - sufficient errors
  m_posted_points = 0;                          // counter
  m_max_tail_length = 3 * m_ideal_follow_range; // maximum tail length to be left behind - we have this so the leaders trajectory will mostly be followed

  m_ownship = XYPoint(m_osx, m_osy);
  m_target = XYPoint(m_osx, m_osy);

  m_cpq = ConvoyPointQueue(m_ownship, m_target);

  m_nav_x_k = "NAV_X";
  m_nav_y_k = "NAV_Y";
  m_nav_h_k = "NAV_HEADING";
  m_nav_spd_k = "NAV_SPEED";
  m_leader_k = "LEADER";
  m_contact_k = "CONTACT";
  m_agent_info_k = "AGENT_INFO";
  m_lead_point_k = "NEW_LEAD_POINT";
  m_contact_list_k = "CONTACTS_LIST";
  m_task_state_k = "TASK_STATE";
  m_whotowho_k = "A_FOLLOWING_B";
  m_updates_var_k = "CONVOY_UPDATES";

  string infovars = m_nav_h_k +
                    "," + m_nav_y_k +
                    "," + m_nav_h_k +
                    "," + m_nav_spd_k +
                    "," + m_leader_k +
                    "," + m_contact_k +
                    "," + m_agent_info_k +
                    "," + m_lead_point_k +
                    "," + m_contact_list_k +
                    "," + m_task_state_k +
                    "," + m_whotowho_k +
                    "," + m_updates_var_k;

  addInfoVars(infovars);
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ConvoyPD::setParam(string param, string val)
{

  param = tolower(param);

  if ((param == "is_leader") && isBoolean(val))
  {
    return setBooleanOnString(m_is_leader, val);
  }
  else if (param == "desired_speed" && isNumber(val))
  {
    m_desired_speed = stod(val);
    return true;
  }
  else if (param == "type")
  {
    m_type_assignment = val;
    return true;
  }
  else if (param == "point_update_distance" && isNumber(val))
  {
    m_point_update_distance = stod(val);
    return true;
  }
  else if (param == "kp_spd" && isNumber(val))
  {
    kp_spd = stod(val);
    return true;
  }
  else if (param == "kd_spd" && isNumber(val))
  {
    kd_spd = stod(val);
    return true;
  }
  else if (param == "ki_spd" && isNumber(val))
  {
    ki_spd = stod(val);
    return true;
  }
  else if (param == "kp_hdg" && isNumber(val))
  {
    kp_hdg = stod(val);
    return true;
  }
  else if (param == "kd_hdg" && isNumber(val))
  {
    kd_hdg = stod(val);
    return true;
  }
  else if (param == "ki_hdg" && isNumber(val))
  {
    ki_hdg = stod(val);
    return true;
  }
  else if (param == "updates")
  {
    m_update_var = val;
    return true;
  }

  // If not handled above, then just return false;
  return (false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ConvoyPD::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ConvoyPD::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ConvoyPD::onIdleState()
{
  updateMessages();
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ConvoyPD::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ConvoyPD::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ConvoyPD::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ConvoyPD::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateMessages()
//   Purpose: Invoked when idle and when running to keep variables up to date
double pmod(double a, double b)
{
  double r = fmod(a, b);
  if (r < 0)
  {
    r += b;
  }
  return r;
};

void BHV_ConvoyPD::updateMessages()
{
  std::string fname = "garb_" + m_us_name + ".txt";
  FILE *file;
  file = fopen(fname.c_str(), "a");

  //TODO: Add a UpdateOwnshipState function
  m_latest_buffer_time = getBufferCurrTime();
  m_osx = getBufferDoubleVal(m_nav_x_k);
  m_osy = getBufferDoubleVal(m_nav_y_k);
  m_osh = getBufferDoubleVal(m_nav_h_k);
  double dt = (m_latest_buffer_time - m_osh_tprv);

  // Change in heading +- 180 degrees always
  double dh = pmod(m_osh - m_osh_prv + 180, 360) - 180;

  // Incorporate an exponential moving average filter
  double m_osh_dot_instant = dh / dt;
  double alpha = 0.1;
  // m_osh_dot = m_osh*alpha+(1-alpha)*m_osh_prv;
  double ff = exp(-alpha * dt);
  m_osh_dot = m_osh_dot * ff + m_osh_dot_instant * (1 - ff);
  m_osh_prv = m_osh;
  m_osh_tprv = m_latest_buffer_time;

  m_speed = getBufferDoubleVal(m_nav_spd_k);

  m_ownship.set_vx(m_osx);
  m_ownship.set_vy(m_osy);

  //TODO: This should be moved ot the behavior part
  if (!m_is_leader && m_cpq.m_points.size() > 0)
  {
    XYPoint np = m_cpq.m_points.front().p;
    double dist = hypot(np.get_vy() - m_osy, np.get_vx() - m_osx);

    // Captured
    if (dist < 3)
    {

      ConvoyPoint prv_cp = m_cpq.dequeue();
      fprintf(file, "Points in queue:\n %s\n", m_cpq.repr("\n").c_str());
      XYPoint prv_point = prv_cp.p;
      prv_point.set_active(false);
      postRepeatableMessage("VIEW_POINT", prv_point.get_spec());
      //!m_is_tail
      if (true)
      {
        NodeMessage node_message;
        node_message.setSourceNode(m_us_name);
        node_message.setDestNode(m_follower);
        node_message.setVarName(m_lead_point_k);
        node_message.setStringVal(prv_cp.repr());
        postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
      }
    }
  }

  if (getBufferVarUpdated(m_leader_k))
    m_is_leader = tolower(getBufferStringVal(m_leader_k)) == "true" ? true : false;

  //TODO: Add a handleUpdateContactList
  if (getBufferVarUpdated(m_contact_list_k))
  {
    
    m_contact_list_str = getBufferStringVal(m_contact_list_k);
    fprintf(file,"New contact\n\t%s\n",m_contact_list_str.c_str());
    m_contact_list = parseString(m_contact_list_str, ",");
  }

  // TODO: Need to check for updates first but it wasn't working - have tried several times
  if (getBufferVarUpdated(m_task_state_k) &&!m_has_broadcast_contact)
  //if (true)
  {
    // fprintf(file,"New buffer val for task state\n");

    m_task_state = tolower(getBufferStringVal(m_task_state_k));

    // If we won the convoy bid, we know who we are trailing
    size_t idx1 = m_task_state.find("id=follow_");
    size_t idx2 = m_task_state.find("bidwon");
    if ((idx1 != std::string::npos) && (idx2 != std::string::npos) && !m_has_broadcast_contact && !m_is_leader)
    {
      // size_t idx1 = m_task_state.find("id=follow_");
      // size_t idx2 = m_task_state.find(",", idx1 + 10);
      fprintf(file,"Broadcasting contact from task state\n");
      m_contact = m_task_state.substr(idx1 + 10);
      m_contact = biteString(m_contact, ',');
      m_contact_info.name = m_contact;
      m_follower_to_leader_mapping[m_us_name] = m_contact;
      NodeMessage node_message;
      node_message.setSourceNode(m_us_name);
      node_message.setDestNode("all");
      node_message.setVarName(m_whotowho_k);
      node_message.setStringVal(tolower(m_us_name) + "_following_" + tolower(m_contact));
      postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
      m_has_broadcast_contact = true;
    }
  }

  if (getBufferVarUpdated(m_lead_point_k))
  {

    std::string msg = getBufferStringVal(m_lead_point_k);
    m_cpq.add_point(ConvoyPoint(msg));
    XYPoint cp = m_cpq.m_points.back().p;

    postRepeatableMessage("VIEW_POINT", cp.get_spec());
  }

  //TODO: We need to revisit this part
  if (true)
  {
    // post our own state to all ships
    NodeMessage node_message;
    NodeRecord nr;

    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");
    node_message.setVarName(string("AGENT_INFO_") + m_us_name);

    nr.setX(m_osx);
    nr.setY(m_osy);
    nr.setHeading(m_osh);
    nr.setSpeed(m_speed);
    nr.setName(m_us_name);
    nr.setTimeStamp(getBufferCurrTime());

    node_message.setStringVal(nr.getSpec());

    postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
    fprintf(file,"Posting state to all ships\n");
  }

  //TODO: Need to obtain some of the other agent states
  if (m_has_broadcast_contact)
  {
    // bool ok = true;
    // if(ok) m_contact_info.x = getBufferDoubleVal(toupper(m_contact)+"_NAV_X", ok);
    // if(ok) m_contact_info.y = getBufferDoubleVal(toupper(m_contact)+"_NAV_Y", ok);
    // if(ok) m_contact_info.h = getBufferDoubleVal(toupper(m_contact)+"_NAV_HEADING", ok);
    // if(ok) m_contact_info.speed = getBufferDoubleVal(toupper(m_contact)+"_NAV_SPEED",   ok);
    // if(ok) m_contact_info.utc = getBufferDoubleVal(toupper(m_contact)+"_NAV_UTC", ok);
    m_target.set_vx(m_contact_info.x);
    m_target.set_vy(m_contact_info.y);
  }

  if (m_is_leader)
  {
    m_contact = "*";
    m_follower_to_leader_mapping[m_us_name] = "*";
    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");
    node_message.setVarName(m_whotowho_k);
    node_message.setStringVal(tolower(m_us_name) + "_following_" + "*");
    postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
  }

  // bool new_agent_ordering = ;
  if (getBufferVarUpdated(m_whotowho_k))
  {
    std::string atob_msg = getBufferStringVal(m_whotowho_k);
    string agent_a = biteString(atob_msg, '_');
    biteString(atob_msg, '_');
    string agent_b = atob_msg;
    m_follower_to_leader_mapping[agent_a] = agent_b;
    m_has_broadcast_leadership = false;
  }

  std::map<string, string>::iterator it = m_follower_to_leader_mapping.begin();
  for (; it != m_follower_to_leader_mapping.end(); ++it)
  {
    //fprintf(file, "%s to %s\n", it->first.c_str(), it->second.c_str());
  }

  //TODO: Add an handleOrderingUpdate function
  if (m_follower_to_leader_mapping.size() > m_contact_list.size())
  {
    // Contacts list is n-1 agents since it does not include the ownship

    // When we have a mapping of all known agents, we generate our ordering, and share it to all other nodes

    std::map<string, string>::iterator it = m_follower_to_leader_mapping.begin();

    for (; it != m_follower_to_leader_mapping.end(); ++it)
    {
      m_leader_to_follower_mapping[it->second] = it->first;
    }

    std::string ahead = "*";
    std::string tail = "*";
    std::string ordering_str;

    for (int i = 0; i < m_leader_to_follower_mapping.size(); i++)
    {
      tail = m_leader_to_follower_mapping[ahead];
      m_ordering_vector.push_back(tail);
      ahead = tail;
      if (m_us_name == tail)
        m_place_in_convoy = i;

      ordering_str += tail;
      if (i != m_leader_to_follower_mapping.size() - 1)
      {
        ordering_str += ",";
      }
    }

    if (m_place_in_convoy + 1 <= m_leader_to_follower_mapping.size())
    {
      m_follower = m_leader_to_follower_mapping[m_us_name];

      // fprintf(file,"%s - #%d followed by: %s\n", m_us_name.c_str(), m_place_in_convoy, m_follower.c_str());
      // if(m_follower == "") {
      //   fprintf(file,"tail agent\n");
      // }
    }

    if (tail == m_us_name)
    {
      m_is_tail = true;
      fprintf(file,"is tail\n");
    }

    else if (!m_is_leader && !m_is_tail)
    {
      m_is_midship = true;
      fprintf(file,"is midship\n");
    }

    fprintf(file, "ordering: %s\n", ordering_str.c_str());
    
    postMessage("ORDERING", ordering_str);
  }

  //TODO:   Add a handleUpdateVar function
  bool has_updates_var = false;
  string updates_msg;
  has_updates_var = getBufferVarUpdated(m_updates_var_k);
  if (has_updates_var)
  {
    updates_msg = getBufferStringVal(m_updates_var_k);
  }

  if (updates_msg != m_updates_buffer)
  {

    m_updates_buffer = updates_msg;
    // UPDATES_VAR=gains='kp1,kd1,ki1',gains2='kp2,kd2,ki2',
    vector<string> updates = parseQuotedString(updates_msg, ',');

    vector<string>::iterator it = updates.begin();

    for (; it != updates.end(); ++it)
    {
      // a segment may equal gains='kp1,kd1,ki1'
      string segment = *it;
      // gains
      string param = biteString(segment, '=');
      //'kp1,kd1,ki1';
      string value;
      if (param == m_des_spd_k)
      {
        value = biteString(segment, '=');
        m_desired_speed = stod(value);
      }
      else if (param == m_point_update_dist_k)
      {
        value = biteString(segment, '=');
        m_point_update_distance = stod(value);
      }
    }
  }
  fclose(file);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction *BHV_ConvoyPD::onRunState()
{
  // Part 1: Build the IvP function
  updateMessages();
  IvPFunction *ipf = 0;

  // Identify if we are a leader, a mid-follower, or the tail follower

  // If we are the leader, for this controller, we don't do anything special
  // We do send points to the person following us
  // TODO: An agent is responsible for telling the agent their following who they are
  // If we are the leader, for this controller, we are only passive, without affecting the behavior function

  // If we are a mid-follower, we use a PD controller for maintaining our desired speed
  // TODO: We head to the point next in our queue

  // Coupled ZAIC Function for heading and speed
  // TODO: The speed is centered around the nominal speed, and the error from our ideal follow range
  //   is scaled by a gain to determine our current cruising speed
  std::string fname = "garb_" + m_us_name + ".txt";
  FILE *file;
  file = fopen(fname.c_str(), "a");
  fprintf(file,"On runs state: %s\n", m_us_name.c_str());
  if (m_is_leader)
  {
    if ((abs(m_osx - m_osx_prv) > m_eps) &&
        (abs(m_osy - m_osy_prv) > m_eps))
    {
      double dx = m_osx - m_osx_prv;
      double dy = m_osy - m_osy_prv;
      double dh = m_osh - m_osh_prv;

      m_osx_prv = m_osx;
      m_osy_prv = m_osy;
      m_osh_prv = m_osh;
      m_interval_odo += sqrt(dy * dy + dx * dx);
    }
    if (m_interval_odo >= m_point_update_distance)
    {

      XYPoint cp(m_osx, m_osy);
      // cp.set_duration(20);
      cp.set_vertex_color("red");
      cp.set_vertex_size(10);
      cp.set_active(true);
      cp.set_label(to_string(m_posted_points));
      cp.set_label_color("invisible");
      cp.set_id(to_string(m_posted_points));
      // postRepeatableMessage("VIEW_POINT", cp.get_spec());
      // m_previous_points.push_back(cp);
      ConvoyPoint cpp(cp);

      cpp.set_st(getBufferCurrTime());
      cpp.set_lh(m_osh);
      cpp.set_lhr(m_osh_dot);

      // m_cpq.add_point(cpp);
      ConvoyPoint cp_alt(cpp.repr()); // temporary

      m_interval_odo = 0;
      double dist_between = m_cpq.get_dist_to_target();

      NodeMessage node_message;
      node_message.setSourceNode(m_us_name);
      node_message.setDestNode(m_follower);
      node_message.setVarName(m_lead_point_k);
      node_message.setStringVal(cpp.repr());
      postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
      if (m_cpq.m_points.size() > 20)
      {
        fprintf(file, "distance: %0.2f\n", dist_between);
        // ConvoyPoint prv_cp = m_cpq.dequeue();
        // XYPoint prv_point = prv_cp.p;
        // prv_point.set_active(false);
        // postRepeatableMessage("VIEW_POINT", prv_point.get_spec());
      }
      m_posted_points++;
    }
  }
  else
  {

    // do general follower stuff here

    if (m_is_midship)
    {
    }
    else if (m_is_tail)
    {
    }
  }
  

  ZAIC_PEAK spd_zaic(m_domain, "speed");
  double m_desired_speed = 1.5; // TODO: Change this to a parameter
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);
  if (spd_zaic.stateOK() == false)
  {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return (0);
  }

  XYPoint np;
  if (m_cpq.m_points.size() > 0)
  {
    np = m_cpq.m_points.front().p;
  }
  else
  {
    np = XYPoint(m_osx, m_osy);
  }

  double rel_ang_to_wpt = relAng(m_osx, m_osy, np.get_vx(), np.get_vy());
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);
  crs_zaic.setValueWrap(true);
  if (crs_zaic.stateOK() == false)
  {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return (0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  // The ZAIC speed function will have a desired nominal speed, shifted by some error. It will be a sharp peak.

  // The ZAIC heading function will provide the desired heading to the next waypoint in the datastructure

  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if (m_cpq.m_points.size() && !m_is_leader)
  {
    if (ivp_function)
      ivp_function->setPWT(100);
  }
  else
  {
    if (ivp_function)
      ivp_function->setPWT(0);
  }
  fclose(file);
  return (ivp_function);
}
