/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoyEngine.cpp                                 */
/*    DATE: July 22nd, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "EvalConvoyEngine.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

EvalConvoyEngine::EvalConvoyEngine()
{
  m_recap_var = "CONVOY_RECAP";
  m_stat_recap_var = "CONVOY_STAT_RECAP";
  m_spd_policy_var = "CONVOY_SPD_POLICY";
  
  m_on_tail_thresh = 10;
  m_alignment_thresh = 60;
  m_tracking_thresh = 3;

  m_recap_rcvd = 0;
  m_stat_recap_rcvd = 0;
  m_spd_policy_rcvd = 0;

  m_on_tail = false;
  m_aligned = false;
  m_tethered = false;
  m_fastened = false;
  m_tracking = false;

  m_prev_time = 0;
  m_time_on_tail  = 0;
  m_time_aligned  = 0;
  m_time_tethered = 0;
  m_time_fastened = 0;
  m_time_tracking = 0;
  
  m_pct_time_on_tail  = 0;
  m_pct_time_aligned  = 0;
  m_pct_time_tethered = 0;
  m_pct_time_fastened = 0;
  m_pct_time_tracking = 0;

  m_rng_switches = 0;
  m_rng_switch_thresh = 0;
  
  m_attained_on_tail = false;
  m_attained_aligned = false;
  m_attained_tethered = false;
  m_attained_fastened = false;
  m_attained_tracking = false;

  m_time_attained_on_tail  = 0;
  m_time_attained_aligned  = 0;
  m_time_attained_tethered = 0;
  m_time_attained_tracking = 0;

  m_track_err_snap = 0.1;
  
  // Internal State variables
  m_curr_time = 0;
  m_prev_time = 0;
  m_tstamp_first_recap = 0;

  m_tstamp_attained_on_tail  = 0;
  m_tstamp_attained_aligned  = 0;
  m_tstamp_attained_tethered = 0;
  m_tstamp_attained_tracking = 0;
}


//---------------------------------------------------------
// Procedure: setCurrTime()

void EvalConvoyEngine::setCurrTime(double curr_time)
{
  m_prev_time = m_curr_time;
  m_curr_time = curr_time;
}

//---------------------------------------------------------
// Procedure: updateMetrics()

void EvalConvoyEngine::updateMetrics()
{
  updateCoreMetrics();
  updateAttainMetrics();
  updateTimeMetrics();
  updateSwitchMetrics();
  updateTrackErrMetrics();
}

//---------------------------------------------------------
// Procedure: setParam()

bool EvalConvoyEngine::setParam(string param, string value)
{
  bool handled = false;
  if(param == "on_tail_thresh") 
    handled = setPosDoubleOnString(m_on_tail_thresh, value);
  else if(param == "alignment_thresh") 
    handled = setPosDoubleOnString(m_alignment_thresh, value);
  else if(param == "tracking_thresh") 
    handled = setPosDoubleOnString(m_tracking_thresh, value);
  else if(param == "rng_switch_thresh") 
    handled = setNonNegDoubleOnString(m_rng_switch_thresh, value);
  else if(param == "recap_var") 
    handled = setNonWhiteVarOnString(m_recap_var, value);
  else if(param == "stat_recap_var") 
    handled = setNonWhiteVarOnString(m_stat_recap_var, value);
  else if(param == "spd_policy_var") 
    handled = setNonWhiteVarOnString(m_stat_recap_var, value);
  else if(param == "track_err_snap") 
    handled = setPosDoubleOnString(m_track_err_snap, value);
  
  return(handled);
}

//---------------------------------------------------------
// Procedure: handleStatRecap()
//   Example: follower=henry,leader=abe,ideal_rng=40,compression=0.4

bool EvalConvoyEngine::handleStatRecap(string recap_str)
{
  m_stat_recap = string2ConvoyStatRecap(recap_str);
  m_stat_recap_rcvd++;

  handleStatRecapAlly(recap_str);
  return(true);
}

//---------------------------------------------------------
// Procedure: handleRecap()
//   Example: convoy_rng=17.4,vname=deb,cname=cal,rng_delta=-7.5,
//            tail_rng=4.5,tail_ang=3.44,mark_bng=46.41,trk_err=0.38,
//            almnt=49.84,set_spd=0.661,cnv_avg2=0.905,cnv_avg5=0.867,
//            cmode=close,utc=29778372305,mx=30.6,my=-11.8,mid=0,
//            tail_cnt=6,index=390

bool EvalConvoyEngine::handleRecap(string recap_str)
{
  if(m_tstamp_first_recap == 0)
    m_tstamp_first_recap = m_curr_time;
  
  m_recap = string2ConvoyRecap(recap_str);
  m_recap_rcvd++;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleSpdPolicy()
//   Example: full_stop_rng=2,slower_rng=23,ideal_rng=25,faster_rng=27,
//            full_lag_rng=40,lag_spd_delta=2,max_compress=0.9

bool EvalConvoyEngine::handleSpdPolicy(string policy_str)
{
  m_spd_policy = string2ConvoySpdPolicy(policy_str);
  m_spd_policy_rcvd++;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleStatRecapAlly()
//   Example: follower=henry,leader=abe,ideal_rng=40,compression=0.4

bool EvalConvoyEngine::handleStatRecapAlly(string stat_recap)
{
  if(stat_recap == "")
    return(false);
  
  string follower = tokStringParse(stat_recap, "follower", ',', '=');
  string leader   = tokStringParse(stat_recap, "leader", ',', '=');
  string idle     = tokStringParse(stat_recap, "idle", ',', '=');

  if((leader == "") || (follower == ""))
    return(false);

  if(idle != "true") {
    string pairing = follower + "," + leader;
    m_order_detector.addPairing(pairing);
  }
  else {
    m_order_detector.removeVehicle(follower);
    m_order_detector.removeVehicle(leader);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: getBool()

bool EvalConvoyEngine::getBool(string str) const
{
  if(str == "on_tail")
    return(m_on_tail);
  else if(str == "aligned")
    return(m_aligned);
  else if(str == "tethered")
    return(m_tethered);
  else if(str == "fastened")
    return(m_fastened);
  else if(str == "tracking")
    return(m_tracking);

  else if(str == "attained_on_tail")
    return(m_attained_on_tail);
  else if(str == "attained_aligned")
    return(m_attained_aligned);
  else if(str == "attained_tethered")
    return(m_attained_tethered);
  else if(str == "attained_fastened")
    return(m_attained_fastened);
  else if(str == "attained_tracking")
    return(m_attained_tracking);

  return(false);
}

//---------------------------------------------------------
// Procedure: getDouble()

double EvalConvoyEngine::getDouble(string str) const
{
  if(str == "time_on_tail")
    return(m_time_on_tail);
  else if(str == "time_aligned")
    return(m_time_aligned);
  else if(str == "time_tethered")
    return(m_time_tethered);
  else if(str == "time_fastened")
    return(m_time_fastened);
  else if(str == "time_tracking")
    return(m_time_tracking);

  else if(str == "pct_time_on_tail")
    return(m_pct_time_on_tail);
  else if(str == "pct_time_aligned")
    return(m_pct_time_aligned);
  else if(str == "pct_time_tethered")
    return(m_pct_time_tethered);
  else if(str == "pct_time_fastened")
    return(m_pct_time_fastened);
  else if(str == "pct_time_tracking")
    return(m_pct_time_tracking);

  else if(str == "time_attained_on_tail")
    return(m_attained_on_tail);
  else if(str == "time_attained_aligned")
    return(m_attained_aligned);
  else if(str == "time_attained_tethered")
    return(m_attained_tethered);
  else if(str == "time_attained_fastened")
    return(m_attained_fastened);
  else if(str == "time_attained_tracking")
    return(m_attained_tracking);

  else if(str == "on_tail_thresh")
    return(m_on_tail_thresh);
  else if(str == "alignment_thresh")
    return(m_alignment_thresh);
  else if(str == "tracking_thresh")
    return(m_tracking_thresh);
  else if(str == "rng_switch_thresh")
    return(m_rng_switch_thresh);

  else if(str == "convoy_rng")
    return(m_recap.getConvoyRng());
  else if(str == "tail_rng")
    return(m_recap.getTailRng());
  else if(str == "tail_ang")
    return(m_recap.getTailAng());
  else if(str == "marker_bng")
    return(m_recap.getMarkerBng());
  else if(str == "track_err")
    return(m_recap.getTrackErr());
  else if(str == "convoy_rng_delta")
    return(m_recap.getConvoyRngDelta());

  else if(str == "ideal_range")
    return(m_stat_recap.getIdealRange());
  else if(str == "track_err_snap")
    return(m_track_err_snap);
  
  return(0);
}


//---------------------------------------------------------
// Procedure: getUInt()

unsigned int EvalConvoyEngine::getUInt(string str) const
{
  if(str == "recap_rcvd")
    return(m_recap_rcvd);
  else if(str == "stat_recap_rcvd")
    return(m_stat_recap_rcvd);
  else if(str == "spd_policy_rcvd")
    return(m_spd_policy_rcvd);
  else if(str == "rng_switches")
    return(m_rng_switches);

  return(0);
}

//---------------------------------------------------------
// Procedure: getStrBool()

string EvalConvoyEngine::getStrBool(string str) const
{
  bool val = getBool(str);
  return(boolToString(val));
}
    
//---------------------------------------------------------
// Procedure: getStrDouble()

string EvalConvoyEngine::getStrDouble(string str, int res) const
{
  if(res < 0)
    res = 0;

  double val = getDouble(str);
  return(doubleToString(val, res));
}

//---------------------------------------------------------
// Procedure: getStrDouble()

string EvalConvoyEngine::getStrUInt(string str) const
{
  unsigned int val = getUInt(str);
  return(uintToString(val));
}
    
//---------------------------------------------------------
// Procedure: getStrString()

string EvalConvoyEngine::getStrString(string str) const
{
  if(str == "recap_var")
    return(m_recap_var);
  if(str == "stat_recap_var")
    return(m_stat_recap_var);
  if(str == "spd_policy_var")
    return(m_spd_policy_var);
  
  return("");
}
    
//---------------------------------------------------------
// Procedure: updateCoreMetrics()
//   Metrics: (1) on_tail
//            (2) aligned
//            (3) tethered
//            (4) fastened

void EvalConvoyEngine::updateCoreMetrics()
{
  // Delete this comment if wanted:
  // Bail on evaluating these booleans unless the first
  // recap has been recieved.  Otherwise the initial values
  // for these variables will be evaluated until a recap
  // has been recieved, and this results in unexpected
  // skewing of the metrics.
  if(m_tstamp_first_recap == 0)
    return;
  
  double tail_rng = m_recap.getTailRng();
  double tail_ang = m_recap.getTailAng();
  double mark_bng = m_recap.getMarkerBng();
  double alignment = m_recap.getAlignment();
  double track_err = m_recap.getTrackErr();

  m_on_tail  = (tail_rng <= m_on_tail_thresh);
  m_aligned  = (alignment <= m_alignment_thresh);
  m_tethered = (m_on_tail && m_aligned);
  m_tracking = (m_on_tail && ( track_err <= m_tracking_thresh ) );  // Not sure about this..
                                                                    // but there is an error
                                                                    // in logic here.

  double convoy_rng = m_recap.getConvoyRng();
  double slower_rng = m_spd_policy.getSlowerConvoyRng();
  double faster_rng = m_spd_policy.getFasterConvoyRng();
  
  m_fastened = m_tethered;
  if(convoy_rng < slower_rng)
    m_fastened = false;
  else if(convoy_rng > faster_rng)
    m_fastened = false;
}

//---------------------------------------------------------
// Procedure: updateAttainMetrics()

void EvalConvoyEngine::updateAttainMetrics()
{
  if(!m_attained_on_tail && m_on_tail) {
    m_attained_on_tail = true;
    m_tstamp_attained_on_tail = m_curr_time;
    m_time_attained_on_tail   = m_curr_time - m_tstamp_first_recap;
  }
  
  if(!m_attained_aligned && m_aligned) {
    m_attained_aligned = true;
    m_tstamp_attained_aligned = m_curr_time;
    m_time_attained_aligned   = m_curr_time - m_tstamp_first_recap;
  }
  
  if(!m_attained_tethered && m_tethered) {
    m_attained_tethered = true;
    m_tstamp_attained_tethered = m_curr_time;
    m_time_attained_tethered   = m_curr_time - m_tstamp_first_recap;
  }
  
  if(!m_attained_fastened && m_fastened) {
    m_attained_fastened = true;
    m_tstamp_attained_fastened = m_curr_time;
    m_time_attained_fastened   = m_curr_time - m_tstamp_first_recap;
  } 

  if(!m_attained_tracking && m_tracking) {
    m_attained_tracking = true;
    m_tstamp_attained_tracking = m_curr_time;
    m_time_attained_tracking   = m_curr_time - m_tstamp_first_recap;
  } 
}

//---------------------------------------------------------
// Procedure: updateTimeMetrics()

void EvalConvoyEngine::updateTimeMetrics()
{
  if(m_prev_time == 0)
    return;
  
  double delta_time = m_curr_time - m_prev_time;
  if(m_on_tail)
    m_time_on_tail += delta_time;
  if(m_aligned)
    m_time_aligned += delta_time;
  if(m_tethered)
    m_time_tethered += delta_time;
  if(m_fastened)
    m_time_fastened += delta_time;
  if(m_tracking)
    m_time_tracking += delta_time;

  double total_time = m_curr_time - m_tstamp_first_recap;
  m_pct_time_on_tail  = 100 * m_time_on_tail  / total_time;
  m_pct_time_aligned  = 100 * m_time_aligned  / total_time;
  m_pct_time_tethered = 100 * m_time_tethered / total_time;
  m_pct_time_fastened = 100 * m_time_fastened / total_time;
  m_pct_time_tracking = 100 * m_time_tracking / total_time;
}

//---------------------------------------------------------
// Procedure: updateSwitchMetrics()

void EvalConvoyEngine::updateSwitchMetrics()
{
  double convoy_rng = m_recap.getConvoyRng();
  double ideal_convoy_rng = m_spd_policy.getIdealConvoyRng();

  string rng_side = "close";
  if(convoy_rng > ideal_convoy_rng)
    rng_side = "far";

  if(m_rng_side == "") {
    m_rng_side = rng_side;
    return;
  }

  if(m_rng_side == "close") {
    if(convoy_rng > (ideal_convoy_rng + m_rng_switch_thresh)) {
      m_rng_side = "far";
      m_rng_switches++;
    }
  }
  if(m_rng_side == "far") {
    if(convoy_rng < (ideal_convoy_rng - m_rng_switch_thresh)) {
      m_rng_side = "close";
      m_rng_switches++;
    }
  }
}


//---------------------------------------------------------
// Procedure: updateTrackErrMetrics()

void EvalConvoyEngine::updateTrackErrMetrics()
{
  if(!m_attained_on_tail)
    return;
  
  double track_err = m_recap.getTrackErr();
  double track_err_snapped = snapToStep(track_err, m_track_err_snap);

  m_map_track_err_bins[track_err_snapped]++;
}


//---------------------------------------------------------
// Procedure: buildReport()

vector<string> EvalConvoyEngine::buildReport() const
{
  vector<string> msgs;
  // =======================================================
  // Part 1: Config info
  // =======================================================
  string s1 = " (" + getStrUInt("recap_rcvd") + ")";
  string s2 = " (" + getStrUInt("stat_recap_rcvd") + ")";
  string s3 = " (" + getStrUInt("spd_policy_rcvd") + ")";
  string str_on_tail_thresh = getStrDouble("on_tail_thresh");
  string str_align_thresh = getStrDouble("alignment_thresh");
  string str_track_thresh = getStrDouble("tracking_thresh");
  string str_ideal_rng = getStrDouble("ideal_range");
  string str_switch_thresh = getStrDouble("rng_switch_thresh");

  msgs.push_back("Configuration:");
  msgs.push_back("  recap_var:        " + m_recap_var +s1);
  msgs.push_back("  stat_recap_var:   " + m_stat_recap_var + s2);
  msgs.push_back("  spd_policy_var:   " + m_spd_policy_var + s3);
  msgs.push_back("  on_tail_thresh:   " + str_on_tail_thresh);
  msgs.push_back("  alignment_thresh: " + str_align_thresh);
  msgs.push_back("  tracking_thresh:  " + str_track_thresh);
  msgs.push_back("  ideal_convoy_rng: " + str_ideal_rng);
  msgs.push_back("  rng_switch_thres: " + str_switch_thresh);
  msgs.push_back("");
  msgs.push_back("Speed Policy: ");
  msgs.push_back("  " + getSpdPolicyTerse());
  msgs.push_back("");

  // =======================================================
  // Part 2: Most Recent Recap
  // =======================================================
  vector<string> vrecap = breakLen(getRecapSpec(), 60);
  msgs.push_back("Most Recent Recap:");
  for(unsigned int i=0; i<vrecap.size(); i++)
    msgs.push_back("   " + vrecap[i]);
  msgs.push_back("");

  // =======================================================
  // Part 3: Raw Recap Status
  // =======================================================
  string str_convoy_rng = getStrDouble("convoy_rng");
  string str_tail_rng  = getStrDouble("tail_rng");
  string str_tail_ang  = getStrDouble("tail_ang");
  string str_mark_bng  = getStrDouble("marker_bng");
  string str_track_err = getStrDouble("track_err");
  string str_rng_delta = getStrDouble("convoy_rng_delta");
  str_convoy_rng = padString(str_convoy_rng, 6, false);
  str_rng_delta = " (" + str_rng_delta + ")";
  
  msgs.push_back("Raw Recap Status: ");
  msgs.push_back("  convoy_rng: " + str_convoy_rng + str_rng_delta);
  msgs.push_back("  tail_rng:   " + str_tail_rng);
  msgs.push_back("  tail_ang:   " + str_tail_ang);
  msgs.push_back("  marker_bng: " + str_mark_bng);
  msgs.push_back("  track_err:  " + str_track_err);
  msgs.push_back("");
  
  // =======================================================
  // Part 4: Achievement and Pct Time achieved
  // =======================================================
  string str_on_tail  = getStrBool("on_tail");
  string str_aligned  = getStrBool("aligned");
  string str_tethered = getStrBool("tethered");
  string str_fastened = getStrBool("fastened");
  string str_tracking = getStrBool("tracking");

  string str_pct_on_tail = getStrDouble("pct_time_on_tail",1);
  string str_pct_aligned = getStrDouble("pct_time_aligned",1);
  string str_pct_tethered = getStrDouble("pct_time_tethered",1);
  string str_pct_fastened = getStrDouble("pct_time_fastened",1);
  string str_pct_tracking = getStrDouble("pct_time_tracking",1);

  if(getBool("tethered"))
    str_fastened += " (" + getCorrMode() + ")";
  
  ACTable actab(3,3);
  actab << "Status | % true  | State";
  actab.addHeaderLines();
  actab << "on_tail"  << str_pct_on_tail  << str_on_tail; 
  actab << "aligned"  << str_pct_aligned  << str_aligned;
  actab << "tethered" << str_pct_tethered << str_tethered;
  actab << "fastened" << str_pct_fastened << str_fastened;
  actab << "tracking" << str_pct_tracking << str_tracking;
  msgs.push_back(actab.getFormattedString());
  msgs.push_back("");
  
  // =======================================================
  // Part 5: Range Side Switches
  // =======================================================
  string str_rng_switches = getStrUInt("rng_switches");
  msgs.push_back("Range Side:    " + getRngSide());
  msgs.push_back("Side Switches: " + str_rng_switches);

  string convoy_summary = m_order_detector.getConvoySummary();
  msgs.push_back("Convoy:  " + convoy_summary);
  
  return(msgs);
}

//---------------------------------------------------------
// Procedure: getRepTrackErrBins()

vector<string> EvalConvoyEngine::getRepTrackErrBins() const
{
  // Part 1: Find the total across bins so we can normalize
  unsigned int total_bin_count = 0;
  map<double, unsigned int>::const_iterator p;
  for(p=m_map_track_err_bins.begin(); p!=m_map_track_err_bins.end(); p++) {
    total_bin_count += p->second;
  }

  
  // Part 2: Make the report
  vector<string> lines;
  for(p=m_map_track_err_bins.begin(); p!=m_map_track_err_bins.end(); p++) {
    double bin = p->first;
    unsigned int bin_count = p->second;

    double pct = (double)(bin_count) / (double)(total_bin_count);
    
    string line = doubleToStringX(bin,3) + " " + doubleToStringX(pct*100,3);
    lines.push_back(line);
  }

  return(lines);
}
  
  


