/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyRecap.cpp                                      */
/*    DATE: July 17th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "ConvoyRecap.h"
#include "MBUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ConvoyRecap::ConvoyRecap()
{
  m_convoy_rng = -1;
  m_convoy_rng_delta = -1;

  m_tail_rng  = -1;
  m_tail_ang  = -1;
  m_mark_bng  = -1;
  m_track_err = -1;
  m_alignment = -1;
  m_set_spd   = -1;
  m_cnv_avg2  = -1;
  m_cnv_avg5  = -1;

  m_time_utc  = -1;
  m_marker_x  = 0;
  m_marker_y  = 0;
  m_marker_id = 0;
  m_tail_cnt  = 0;
  m_index     = 0;

  m_marker_x_set  = false;
  m_marker_y_set  = false;
  m_marker_id_set = false;
  m_tail_cnt_set  = false;
  m_index_set     = false;

  m_idle = false;
}

//---------------------------------------------------------------
// Procedure: getStringValue()

string ConvoyRecap::getStringValue(string key) const
{
  key = tolower(key);
  if(key == "convoy_rng")
    return(doubleToStringX(m_convoy_rng, 2));
  else if(key == "vname")
    return(m_vname);
  else if(key == "cname")
    return(m_cname);
  else if(key == "convoy_rng_delta")
    return(doubleToStringX(m_convoy_rng_delta, 2));
  else if(key == "tail_rng")
    return(doubleToStringX(m_tail_rng, 8));
  else if(key == "tail_ang")
    return(doubleToStringX(m_tail_ang, 8));
  else if(key == "marker_bng")
    return(doubleToStringX(m_mark_bng, 2));
  else if(key == "track_err")
    return(doubleToStringX(m_track_err, 2));
  else if(key == "utc_time")
    return(doubleToStringX(m_time_utc, 2));
  else
    return("");  
}

//------------------------------------------------------------
// Procedure: getSpec()

string ConvoyRecap::getSpec() const
{
  string str = "convoy_rng=" + doubleToString(m_convoy_rng,2);

  if(isSetVName())
    str += ",vname=" + m_vname;
  if(isSetCName())
    str += ",cname=" + m_cname;

  if(m_idle) {
    str += ",idle=true";
    return(str);
  }

  if(isSetConvoyRngDelta())
    str += ",rng_delta=" + doubleToString(m_convoy_rng_delta,2);

  if(isSetTailRng())
    str += ",tail_rng=" + doubleToString(m_tail_rng,2);
  if(isSetTailAng())
    str += ",tail_ang=" + doubleToString(m_tail_ang,2);
  if(isSetMarkerBng())
    str += ",mark_bng=" + doubleToString(m_mark_bng,2);
  if(isSetMarkerBng())
    str += ",trk_err=" + doubleToString(m_track_err,2);
  if(isSetAlignment())
    str += ",almnt=" + doubleToString(m_alignment,2);

  if(isSetSetSpd())
    str += ",set_spd=" + doubleToString(m_set_spd,2);
  if(isSetAvg2())
    str += ",cnv_avg2=" + doubleToString(m_cnv_avg2,2);
  if(isSetAvg5())
    str += ",cnv_avg5=" + doubleToString(m_cnv_avg5,2);
  if(isSetCorrMode())
    str += ",cmode=" + m_correction_mode;

  if(isSetTimeUTC())
    str += ",utc=" + doubleToStringX(m_time_utc,3);
  if(isSetMarkerX())
    str += ",mx=" + doubleToString(m_marker_x,2);
  if(isSetMarkerY())
    str += ",my=" + doubleToString(m_marker_y,2);
  if(isSetMarkerID())
    str += ",mid=" + uintToString(m_marker_id);
  if(isSetTailCnt())
    str += ",tail_cnt=" + uintToString(m_tail_cnt);
  if(isSetIndex())
    str += ",index=" + uintToString(m_index);
    
  return(str);
}

//---------------------------------------------------------
// Procedure: string2MissionTask()
//   Example: convoy_rng=17.4,rng_delta=-7.5,tail_rng=4.5,tail_ang=3.44,
//            mark_bng=46.41,trk_err=1.8,almnt=49.84,set_spd=0.661,
//            cnv_avg2=0.905,cnv_avg5=0.867,cmode=close,mx=30.6,
//            my=-11.8,mid=0,tail_cnt=6,index=390

ConvoyRecap string2ConvoyRecap(string msg)
{
  ConvoyRecap new_recap;

  vector<string> svector = parseString(msg, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    double dval  = atof(value.c_str());
    
    if(param == "vname")
      new_recap.setVName(value);
    else if(param == "cname")
      new_recap.setCName(value);
    else if(param == "convoy_rng")
      new_recap.setConvoyRng(dval);
    else if(param == "rng_delta")
      new_recap.setConvoyRngDelta(dval);
    else if(param == "tail_rng")
      new_recap.setTailRng(dval);
    else if(param == "tail_ang")
      new_recap.setTailAng(dval);
    else if(param == "mark_bng")
      new_recap.setMarkerBng(dval);
    else if(param == "trk_err")
      new_recap.setTrackErr(dval);
    else if(param == "almnt")
      new_recap.setAlignment(dval);
    else if(param == "set_spd")
      new_recap.setSetSpd(dval);
    else if(param == "cnv_avg2")
      new_recap.setAvg2(dval);
    else if(param == "cnv_avg5")
      new_recap.setAvg5(dval);
    else if(param == "cmode")
      new_recap.setCorrMode(value);
    else if(param == "mx")
      new_recap.setMarkerX(dval);
    else if(param == "my")
      new_recap.setMarkerY(dval);
    else if(param == "idle")
      new_recap.setIdle(tolower(value) == "true");
    else if(param == "mid")
      new_recap.setMarkerID((unsigned int)(dval));
    else if(param == "tail_cnt")
      new_recap.setTailCnt((unsigned int)(dval));
    else if(param == "index")
      new_recap.setIndex((unsigned int)(dval));
    else if(param == "utc")
      new_recap.setTimeUTC(dval);
  }

  return(new_recap);
}


