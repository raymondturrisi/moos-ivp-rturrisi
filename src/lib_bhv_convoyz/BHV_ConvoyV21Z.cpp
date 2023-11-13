/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_ConvoyV21X.cpp                                   */
/*    DATE: Oct 5th, 2021                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "BHV_ConvoyV21Z.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_SPD.h"
#include "OF_Coupler.h"
#include "MacroUtils.h"
#include "VarDataPairUtils.h"
#include "ConvoyRecap.h"
#include "ConvoyStatRecap.h"
#include "NodeRecord.h"
#include "NodeMessage.h" // In the lib_ufield library

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_ConvoyV21X::BHV_ConvoyV21X(IvPDomain gdomain) : IvPContactBehavior(gdomain)
{
  this->setParam("descriptor", "convoy");

  m_domain = subDomain(m_domain, "course,speed");

  // ====================================================
  // Initialize State variables
  // ====================================================
  m_wptx = 0;
  m_wpty = 0;
  m_wpt_set = false;
  m_set_speed = 0;

  m_cnv_avg_2sec = 0;
  m_cnv_avg_5sec = 0;

  m_convoy_range = 1;

  m_post_recap_verbose = false;

  // ====================================================
  // Initialize State variables (Metrics)
  // ====================================================
  m_tail_range = -1;
  m_tail_angle = -1;
  m_marker_bng = -1;
  m_alignment = -1;

  m_reached_markers = 0;
  m_dropped_markers = 0;

  m_recap_index = 0;
  m_stat_recap_index = 0;

  // ====================================================
  // Initialize Config variables
  // ====================================================
  m_capture_radius = 5; // meters
  m_slip_radius = 20;   // meters
  m_compression = 0;    // [0,1]
  m_patience = 50;      // [1,99]
  m_aft_patience = false;

  m_hint_marker_color = "dodger_blue";
  m_hint_marker_label_color = "off";
  m_hint_marker_size = 8;

  m_active_convoying = false;

  m_holding_policy = "zero";
  m_is_leader = false;

  m_has_announced_contact = false;

  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, HIT_MARKER", "LEADER");
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_ConvoyV21X::setParam(string param, string param_val)
{
  if (IvPContactBehavior::setParam(param, param_val))
    return (true);

  bool handled = false;
  if ((param == "nm_radius") || (param == "slip_radius"))
    handled = setNonNegDoubleOnString(m_slip_radius, param_val);
  else if ((param == "radius") || (param == "capture_radius"))
    handled = setNonNegDoubleOnString(m_capture_radius, param_val);

  else if (param == "tail_length_max")
    handled = m_marker_tail.setMaxTailLength(param_val);
  else if (param == "inter_mark_range")
    handled = m_marker_tail.setInterMarkRange(param_val);

  else if ((param == "full_stop_convoy_range") ||
           (param == "slower_convoy_range") ||
           (param == "ideal_convoy_range") ||
           (param == "faster_convoy_range") ||
           (param == "full_lag_convoy_range") ||
           (param == "lag_speed_delta"))
    handled = m_spd_policy_base.setParam(param, param_val);

  else if (param == "visual_hints")
    return (handleParamVisualHints(param_val));
  else if (param == "compression")
    return (handleParamCompression(param_val));
  else if (param == "aft_patience")
    return (setBooleanOnString(m_aft_patience, param_val));
  else if (param == "holding_policy")
    return (handleConfigHoldingPolicy(param_val));
  else if (param == "is_leader")
  {
    m_is_leader = tolower(param_val) == "true" ? true : false;
    return true;
  }

  else if (param == "marker_flag")
    return (addVarDataPairOnString(m_marker_flags, param_val));
  else if (param == "convoy_flag")
    return (addVarDataPairOnString(m_convoy_flags, param_val));

  else if (param == "post_recap_verbose")
    handled = setBooleanOnString(m_post_recap_verbose, param_val);

  else if (param == "active_convoying")
    handled = setBooleanOnString(m_active_convoying, param_val);

  else if ((param == "patience") && isNumber(param_val))
  {
    double dval = atof(param_val.c_str());
    if ((dval < 1) || (dval > 99))
      return (false);
    m_patience = dval;
    return (true);
  }

  return (handled);
}

//-----------------------------------------------------------
// Procedure: handleConfigHoldingPolicy()
//   Options: (1) zero (default)
//                When in holding or captured state, an IPF
//                will be created with desired hdg of zero
//            (2) curr_hdg (probably should be default)
//                When in holding or captured state, an IPF
//                will be created with desired hdg matching
//                the vehicle's current heading
//            (3) setpt_hdg
//                When in captured state, an IPF will be
//                created with desired hdg matching the
//                vehicle's current hdg. When in holding
//                state, an IPF will be created with desired
//                hdg set to the heading toward the set pt.
//            (4) off
//                When in the capture or holding state, no
//                IPF will be created (behavior inactive)

bool BHV_ConvoyV21X::handleConfigHoldingPolicy(string policy)
{
  if ((policy != "zero") && (policy != "curr_hdg") &&
      (policy != "off") && (policy != "setpt_hdg"))
    return (false);

  m_holding_policy = policy;

  return (true);
}

//-----------------------------------------------------------
// Procedure: checkParamCollective()
//      Note: Invoked after all params have been set, either upon
//            mission startup, or after a full string up dynamic
//            updates.
//            Will check relationships between parameter settings.
//   Returns: Empty string if all is ok.
//            Error message otherwise.

string BHV_ConvoyV21X::checkParamCollective()
{
  string status = m_spd_policy_base.status();
  if (status != "ok")
    return ("base spd_policy: " + status);

  if (m_slip_radius < m_capture_radius)
    return ("slip_radius smaller than capture_radius");

  if (m_marker_tail.getInterMarkRange() >= m_marker_tail.getMaxTailLength())
    return ("tail_length_max should be greater than inter_mark_range");

  return ("");
}

//-----------------------------------------------------------
// Procedure: onSetParamComplete()
//      Note: If compression is non-zero, it is applied here
//            to the base speed policy.

void BHV_ConvoyV21X::onSetParamComplete()
{
  postEventMessage("onSetParamComplete");
  // Prevailing speed policy is initially a copy of base policy
  m_spd_policy = m_spd_policy_base;
  // Then the current compression is applied
  m_spd_policy.compress(m_compression);
  m_spd_policy.setVName(tolower(m_us_name));
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_ConvoyV21X::onRunState()
{
  std::string is_leader = getBufferStringVal("LEADER");
  m_is_leader = tolower(is_leader) == "true" ? true : false;
  if (m_is_leader)
  {
    postMessage("IS_LEADER_AND_AWARE", to_string(getBufferCurrTime()));
  }

  if (!updatePlatformInfo())
    return (0);
  m_marker_tail.setPosCN(m_cnx, m_cny);

  string spd_policy_status = m_spd_policy.status();
  if (spd_policy_status != "ok")
  {
    postWMessage("spd_policy error: " + spd_policy_status);
    return (0);
  }

  bool tail_modified = handleMarkerUpdates();
  if (!m_has_announced_contact)
  {
    NodeMessage node_message;
    node_message.setSourceNode(m_us_name);
    node_message.setDestNode("all");
    node_message.setVarName("ORDERING");
    node_message.setStringVal(tolower(m_us_name) + "_following_" + tolower(m_contact));
    postMessage("NODE_MESSAGE_LOCAL", node_message.getSpec());
    m_has_announced_contact = true;
  }

// Sanity check. Should never have an empty marker tail,
// unless inter marker gap bigger than max tail length.
#if 0
      if(m_marker_tail.empty()) {
        postWMessage("Empty Marker Tail");
        return(0);
      }
#endif

  handleNewContactSpd(m_cnv);
  m_set_speed = m_spd_policy.getSpdFromPolicy(m_cnv_avg_2sec,
                                              m_contact_range,
                                              m_convoy_range);

  updateMetrics();
  postFlags(m_convoy_flags);
  postRecap(tail_modified);
  postSpdPolicy();

  // Generate the IvP function
  setCurrentMarker();

  if (m_aft_patience)
  {
    if (!m_marker_tail.aftMarkerClosest(m_osx, m_osy))
      return (0);
  }

  IvPFunction *ipf = buildOF();

  if (m_is_leader)
  {
    ipf->setPWT(0);
  }
  return (ipf);
}

//-----------------------------------------------------------
// Procedure: handleMarkerUpdates()

bool BHV_ConvoyV21X::handleMarkerUpdates()
{
  // Part 1: handle possible drop of aft marker
  bool new_aft_marker = false;
  bool dropped_marker = checkDropAftMarker();
  if (dropped_marker)
  {
    postFlags(m_marker_flags);
    new_aft_marker = true;
  }

  // If marker_tail is empty, then any newly added marker will be a
  // *new* aft marker.
  if (m_marker_tail.empty())
    new_aft_marker = true;

  bool marker_added = m_marker_tail.handleNewContactPos(m_cnx, m_cny, m_cnh);
  // cout << "passive marker_added:" << boolToString(marker_added) << endl;;

  if (m_active_convoying)
  {
    m_marker_tail.setPosCN(m_cnx, m_cny);
    bool ok;
    vector<string> msgs = getBufferStringVector("HIT_MARKER", ok);
    // cout << "HM_SIZE:" << msgs.size() << endl;
    postEventMessage("HM_SIZE:" + uintToString(msgs.size()));
    postRepeatableMessage("HM_SIZE", uintToString(msgs.size()));

    for (unsigned int i = 0; i < msgs.size(); i++)
    {
      // cout << "HIT_MARKER=" << msgs[i] << endl;
      ConvoyMarker new_marker = string2ConvoyMarker(msgs[i]);
      if (!new_marker.valid())
        postWMessage("Invalid marker rcvd: " + msgs[i]);
      else
      {
        // cout << "new_marker.getVName(): " << new_marker.getVName() << endl;
        // cout << "contact: " << m_contact << endl;
        if (tolower(new_marker.getVName()) == tolower(m_contact))
        {
          // cout << "Matching Marker    yessir !!!!" << endl;
          m_marker_tail.handleNewActiveMarker(new_marker);

          list<ConvoyMarker> cleared_markers = m_marker_tail.getClearedMarkers();
          list<ConvoyMarker>::iterator p;
          for (p = cleared_markers.begin(); p != cleared_markers.end(); p++)
            eraseMarker(*p);

          marker_added = true;

          postEventMessage("hit_marker received:" + msgs[i]);
        }
      }
    }
  }

  if (marker_added)
  {
    ConvoyMarker marker = m_marker_tail.getLeadMarker();
    drawMarker(marker);
  }

  if (new_aft_marker)
  {
    ConvoyMarker aft_marker = m_marker_tail.getAftMarker();
    drawMarker(aft_marker, m_hint_marker_size * 2);
  }

  // Return true if the tail has changed
  return (dropped_marker || marker_added);
}

//-----------------------------------------------------------
// Procedure: setCurrentMarker()

void BHV_ConvoyV21X::setCurrentMarker()
{
  if (m_marker_tail.size() != 0)
  {
    ConvoyMarker marker = m_marker_tail.getAftMarker();
    m_wptx = marker.getX();
    m_wpty = marker.getY();
  }
  else
  {
    m_wptx = m_cnx;
    m_wpty = m_cny;
  }

  string msg = "x=" + doubleToStringX(m_wptx, 1);
  msg += "y=" + doubleToStringX(m_wpty, 1);
  postRepeatableMessage("CONVOY_WPT", msg);

  m_wpt_set = true;
}

//-----------------------------------------------------------
// Procedure: checkDropAftMarker()

bool BHV_ConvoyV21X::checkDropAftMarker()
{
  if (m_marker_tail.empty())
    return (false);

  ConvoyMarker aft_marker = m_marker_tail.getAftMarker();

  double dist = m_marker_tail.distToAftMarker(m_osx, m_osy);

  bool marker_dropped = false;

  // Case 1: Simplest check is the capture radius
  postRepeatableMessage("TAIL_SIZE", m_marker_tail.size());
  if (dist < m_capture_radius)
  {
    m_marker_tail.dropAftMarker();
    postRepeatableMessage("DROP_REASON", "one");
    marker_dropped = true;
  }

  postRepeatableMessage("DROP_DIST", dist);

  // Case 2: If inside the slip radius, check if ownship crossed line
  // perpendicular to line from aft marker to near-aft marker.
  if (!marker_dropped && (dist < m_slip_radius))
  {
    // Calculate the next point on the marker list.
    //
    //               wpt             nextpt         |
    //                 o----------------o           |
    //                  \ angle                     |
    //                   \                          |
    //                    \                         |
    //                     o--->                    |
    //                  ownship                     |

    // If only one marker, next "marker" will be contact position
    double next_x = m_cnx;
    double next_y = m_cny;

    if (m_marker_tail.size() > 1)
    {
      ConvoyMarker marker = m_marker_tail.getNearAftMarker();
      double next_x = marker.getX();
      double next_y = marker.getY();
    }
    double angle = angleFromThreePoints(m_wptx, m_wpty, m_osx, m_osy,
                                        next_x, next_y);

    string amsg = "nx=" + doubleToStringX(next_x, 1);
    amsg += "ny=" + doubleToStringX(next_y, 1);
    amsg += "ang=" + doubleToStringX(angle, 1);
    postRepeatableMessage("DROP_ANG", amsg);
    if (angle < 90)
    {
      m_marker_tail.dropAftMarker();
      postRepeatableMessage("DROP_REASON", "two");
      marker_dropped = true;
    }
  }

  // Case 3: If aft marker was not captured, check if the aft marker
  // should be dropped based on exceeding the max tail length.
  if (!marker_dropped)
  {
    string msg;
    marker_dropped = m_marker_tail.checkDropAftMarker(msg);
    if (msg != "")
      postRepeatableMessage("DROP_REASON", msg);
  }

  // Last step: If marker was dropped, count it and erase it.
  if (marker_dropped)
  {
    postRepeatableMessage("DROP_REASON", "three");

    aft_marker.setUTC(getBufferCurrTime());
    aft_marker.setVName(m_us_name);
    string msg = aft_marker.getSpec();
    postXMessage("HIT_MARKER", msg);
    m_dropped_markers++;
    m_wpt_set = false;
    eraseMarker(aft_marker);
  }

  return (marker_dropped);
}

//-----------------------------------------------------------
// Procedure: handleNewContactSpd()

void BHV_ConvoyV21X::handleNewContactSpd(double cnv)
{
  double curr_time = getBufferCurrTime();
  m_cn_spd_value.push_front(cnv);
  m_cn_spd_tstamp.push_front(curr_time);

  // Part 1A: Ensure behavior spd queue is no older than 5 secs.
  bool done = false;
  while (!done && !m_cn_spd_tstamp.empty())
  {
    if ((curr_time - m_cn_spd_tstamp.back()) > 5)
    {
      m_cn_spd_value.pop_back();
      m_cn_spd_tstamp.pop_back();
    }
    else
      done = true;
  }

  // Part 1B: Get the 5 second average
  double total_spd = 0;
  unsigned int cnt = 0;
  list<double>::iterator p;
  for (p = m_cn_spd_value.begin(); p != m_cn_spd_value.end(); p++)
  {
    double this_spd = *p;
    total_spd += this_spd;
    cnt++;
  }
  if (cnt > 0)
    m_cnv_avg_5sec = total_spd / (double)(cnt);

  // Part 2A: Build a copy of the spd queue only over 2 secs
  list<double> cn_spd_value = m_cn_spd_value;
  list<double> cn_spd_tstamp = m_cn_spd_tstamp;
  done = false;
  while (!done && !cn_spd_tstamp.empty())
  {
    if ((curr_time - cn_spd_tstamp.back()) > 2)
    {
      cn_spd_value.pop_back();
      cn_spd_tstamp.pop_back();
    }
    else
      done = true;
  }

  // Part 2B: Get the 2 second average
  total_spd = 0;
  cnt = 0;
  for (p = cn_spd_value.begin(); p != cn_spd_value.end(); p++)
  {
    double this_spd = *p;
    total_spd += this_spd;
    cnt++;
  }
  if (cnt > 0)
    m_cnv_avg_2sec = total_spd / (double)(cnt);
}

//-----------------------------------------------------------
// Procedure: updateMetrics()

void BHV_ConvoyV21X::updateMetrics()
{
  // Part 1: Update direct raw metrics
  m_tail_range = m_marker_tail.distToAftMarker(m_osx, m_osy);
  m_tail_angle = m_marker_tail.tailAngle(m_osx, m_osy);
  m_marker_bng = m_marker_tail.markerBearing(m_osx, m_osy, m_osh);
  m_alignment = m_tail_angle + m_marker_bng;

  if (m_marker_tail.size() == 0)
    m_convoy_range = m_contact_range;
  else
    m_convoy_range = m_tail_range + m_marker_tail.getMarkerTailLen();

  m_range_delta = m_convoy_range - m_spd_policy.getIdealConvoyRng();
  if (m_range_delta < 0)
    m_range_delta *= -1;

  m_track_error = m_marker_tail.getTrackError(m_osx, m_osy);
}

//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_ConvoyV21X::buildOF()
{
  IvPFunction *ipf = 0;

  // ======================================================
  // Part 0: Determine if we are holding and waiting for marker tail
  // to evolve before moving.
  // ======================================================
  bool holding = false;
  if (m_aft_patience && !m_marker_tail.aftMarkerClosest(m_osx, m_osy))
    holding = true;

  // ======================================================
  // Part 1: Build the Speed ZAIC
  // ======================================================
  if (holding)
    m_set_speed = 0;

  ZAIC_SPD spd_zaic(m_domain, "speed");
  spd_zaic.setMedSpeed(m_set_speed);

  string mode = m_spd_policy.getCorrectionMode();
  if (mode == "close")
    spd_zaic.setMinSpdUtil(50);
  else if (mode == "ideal_close")
    spd_zaic.setMinSpdUtil(25);
  else if (mode == "ideal_far")
    spd_zaic.setMaxSpdUtil(25);
  else if (mode == "far")
    spd_zaic.setMaxSpdUtil(50);
  else if (mode == "full_lag")
    spd_zaic.setMaxSpdUtil(75);
  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  if (!spd_ipf)
  {
    cout << "bld: 2: returning null IPF" << mode << endl;
    postWMessage("Failure on the SPD ZAIC via ZAIC_SPD utility");
    return (0);
  }

  // ======================================================
  // Part 2: Build the Course ZAIC
  // ======================================================
  // ======================================================
  double set_hdg = relAng(m_osx, m_osy, m_wptx, m_wpty);
  if (holding)
  {
    if (m_holding_policy == "zero")
      set_hdg = 0;
    else if (m_holding_policy == "curr_hdg")
      set_hdg = m_osh;
    else if (m_holding_policy == "off")
      return (0);
    // If holding_policy is setpt_hdg, this is as set_hdg
  }

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setValueWrap(true);
  crs_zaic.setParams(set_hdg, 0, 180, 50, 0, 100);
  int ix = crs_zaic.addComponent();
  crs_zaic.setParams(m_osh, 30, 180, 5, 0, 20, ix);

  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);
  if (!crs_ipf)
  {
    cout << "bld: Failed crs IPF" << endl;
    postWMessage("Failure on the CRS ZAIC");
  }
  // ======================================================
  // Part 3: Couple the two 1D functions into a 2D function
  // ======================================================
  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, m_patience, (100 - m_patience));
  if (!ipf)
  {
    cout << "bld: failed coupler" << endl;
    postWMessage("Failure on the CRS_SPD COUPLER");
  }

  return (ipf);
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState()

void BHV_ConvoyV21X::onRunToIdleState()
{
  clearMarkerTail();
}

//-----------------------------------------------------------
// Procedure: clearMarkerTail()

void BHV_ConvoyV21X::clearMarkerTail()
{
  // Part 1: Visuals: Get all markers so each can be erased.
  list<ConvoyMarker> markers = m_marker_tail.getMarkers();

  list<ConvoyMarker>::iterator p;
  for (p = markers.begin(); p != markers.end(); p++)
  {
    ConvoyMarker marker = *p;
    eraseMarker(marker);
  }

  // Part 2: Clear the markers from from the marker_tail
  m_marker_tail.clear();
}

//-----------------------------------------------------------
// Procedure: drawMarker()

void BHV_ConvoyV21X::drawMarker(ConvoyMarker marker, int vsize,
                                string color)
{
  if (!marker.valid())
    return;

  double mx = marker.getX();
  double my = marker.getY();
  unsigned int id = marker.getID();

  XYPoint point(mx, my);
  string label = tolower(m_us_name) + "_";
  label += tolower(m_contact) + "_" + uintToString(id);

  if (vsize < 1)
    point.set_vertex_size(m_hint_marker_size);
  else
    point.set_vertex_size(vsize);

  point.set_label(label);
  point.set_color("label", m_hint_marker_label_color);

  if (isColor(color))
    point.set_color("vertex", color);
  else
    point.set_color("vertex", m_hint_marker_color);

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: eraseMarker()

void BHV_ConvoyV21X::eraseMarker(ConvoyMarker marker)
{
  if (!marker.valid())
    return;

  double mx = marker.getX();
  double my = marker.getY();
  unsigned int id = marker.getID();

  XYPoint point(mx, my);
  string label = tolower(m_us_name) + "_";
  label += tolower(m_contact) + "_" + uintToString(id);

  point.set_label(label);
  point.set_active(false);

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: handleParamVisualHints()

bool BHV_ConvoyV21X::handleParamVisualHints(string hints)
{
  vector<string> svector = parseStringQ(hints, ',');

  for (unsigned int i = 0; i < svector.size(); i++)
  {
    string hint = svector[i];
    string param = tolower(biteStringX(hint, '='));
    string value = hint;

    if (param == "marker_color")
      return (setColorOnString(m_hint_marker_color, value));
    else if (param == "marker_label_color")
      return (setColorOnString(m_hint_marker_label_color, value));
    else if (param == "marker_size")
      return (setNonNegDoubleOnString(m_hint_marker_size, value));
    else
      return (false);
  }
  return (true);
}

//-----------------------------------------------------------
// Procedure: handleParamCompression()

bool BHV_ConvoyV21X::handleParamCompression(string sval)
{
  double dval = atof(sval.c_str());
  if ((dval < 0) || (dval > 1))
    return (false);

  m_compression = dval;

  return (true);
}

//-----------------------------------------------------------
// Procedure: postRecap()
//      Note: Tail_changed info is passed in to allow non-verbose
//            mode to hold back recaps to only occure when a marker
//            is dropped.
//      Note: The static recap will only be posted by the helm when
//            it changes. Helm duplication filter will handle this.
//      Note: Both publications can be silenced with the generic
//            post_mapping = CONVOY_RECAP,silent parameter, defined
//            at the IvPBehavior superclass level.

void BHV_ConvoyV21X::postRecap(bool tail_changed)
{
  if (commsPolicy() != "open")
    return;

  //========================================================
  // Part 1: Build recap of items not likely to change often
  //========================================================
  ConvoyStatRecap stat_recap;
  stat_recap.setLeader(tolower(m_contact));
  stat_recap.setFollower(tolower(m_us_name));
  stat_recap.setIdealRng(m_spd_policy.getIdealConvoyRng());
  stat_recap.setCompression(m_compression);
  stat_recap.setIndex(m_stat_recap_index);

  postMessage("CONVOY_STAT_RECAP", stat_recap.getSpec());

  //========================================================
  // Part 2: Build recap of items likely to change often
  //========================================================
  if (!m_post_recap_verbose)
    return;

  ConvoyRecap recap;
  recap.setVName(tolower(m_us_name));
  recap.setConvoyRng(m_convoy_range);
  recap.setConvoyRngDelta(m_range_delta);
  recap.setTailRng(m_tail_range);
  recap.setTailAng(m_tail_angle);
  recap.setMarkerBng(m_marker_bng);
  recap.setTrackErr(m_track_error);
  recap.setAlignment(m_alignment);
  recap.setSetSpd(m_set_speed);
  recap.setAvg2(m_cnv_avg_2sec);
  recap.setAvg5(m_cnv_avg_5sec);
  recap.setCorrMode(m_spd_policy.getCorrectionMode());
  recap.setTailCnt(m_marker_tail.size());
  recap.setIndex(m_recap_index);
  recap.setTimeUTC(getBufferCurrTime());

  if (!m_marker_tail.empty())
  {
    ConvoyMarker marker = m_marker_tail.getAftMarker();
    recap.setMarkerX(marker.getX());
    recap.setMarkerY(marker.getY());
    recap.setMarkerID(marker.getID());
  }

  m_recap_index++;
  postMessage("CONVOY_RECAP", recap.getSpec());
}

//-----------------------------------------------------------
// Procedure: postSpdPolicy()

void BHV_ConvoyV21X::postSpdPolicy()
{
  string str_spd_policy = m_spd_policy.getSpec();
  postMessage("CONVOY_SPD_POLICY", str_spd_policy);
}

//-----------------------------------------------------------
// Procedure: expandMacros()

string BHV_ConvoyV21X::expandMacros(string sdata)
{
  // =======================================================
  // First expand the macros defined at the superclass level
  // =======================================================
  sdata = IvPContactBehavior::expandMacros(sdata);

  double ideal_convoy_rng = m_spd_policy.getIdealConvoyRng();
  string correction_mode = m_spd_policy.getCorrectionMode();

  sdata = macroExpand(sdata, "LEADER", tolower(m_contact));
  sdata = macroExpand(sdata, "CONVOY_RNG", m_convoy_range);
  sdata = macroExpand(sdata, "RNG_DELTA", m_range_delta);
  sdata = macroExpand(sdata, "IDEAL_RNG", ideal_convoy_rng);
  sdata = macroExpand(sdata, "COMP", m_compression);

  sdata = macroExpand(sdata, "TAIL_RNG", m_tail_range);
  sdata = macroExpand(sdata, "TAIL_ANG", m_tail_angle);
  sdata = macroExpand(sdata, "TRK_ERR", m_track_error);
  sdata = macroExpand(sdata, "MARKER_BNG", m_marker_bng);
  sdata = macroExpand(sdata, "ALIGNMENT", m_alignment);

  sdata = macroExpand(sdata, "SET_SPD", m_set_speed);
  sdata = macroExpand(sdata, "CMODE", correction_mode);
  sdata = macroExpand(sdata, "AVG_SPD2", m_cnv_avg_2sec);
  sdata = macroExpand(sdata, "AVG_SPD5", m_cnv_avg_5sec);

  // marker count / tail size
  // num dropped, num reached

  return (sdata);
}
