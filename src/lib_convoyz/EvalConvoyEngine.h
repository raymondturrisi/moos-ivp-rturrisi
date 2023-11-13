/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: EvalConvoyEngine.h                                   */
/*    DATE: July 22nd, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef EVAL_CONVOY_ENGINE_HEADER
#define EVAL_CONVOY_ENGINE_HEADER

#include <string>
#include "ConvoyRecap.h"
#include "ConvoyStatRecap.h"
#include "ConvoySpdPolicy.h"
#include "ConvoyOrderDetector.h"

class EvalConvoyEngine
{
 public:
   EvalConvoyEngine();
  ~EvalConvoyEngine() {};

  void setCurrTime(double);
  bool setParam(std::string, std::string);
  void updateMetrics();
  
  bool handleStatRecap(std::string);
  bool handleRecap(std::string);
  bool handleSpdPolicy(std::string);
  bool handleStatRecapAlly(std::string);

  bool   getBool(std::string) const;
  double getDouble(std::string) const;
  unsigned int getUInt(std::string) const;

  std::string getStrBool(std::string) const;
  std::string getStrDouble(std::string, int v=2) const;
  std::string getStrUInt(std::string) const;
  std::string getStrString(std::string) const;

  std::string getCorrMode() const  {return(m_recap.getCorrMode());}
  std::string getRngSide() const   {return(m_rng_side);}
  std::string getRecapSpec() const {return(m_recap.getSpec());}
  std::string getStatRecapSpec() const  {return(m_stat_recap.getSpec());}
  std::string getSpdPolicyTerse() const {return(m_spd_policy.getTerse());}

  std::vector<std::string> buildReport() const;
  std::vector<std::string> getRepTrackErrBins() const;

protected: 
  void updateCoreMetrics();
  void updateAttainMetrics();
  void updateTimeMetrics();
  void updateSwitchMetrics();
  void updateTrackErrMetrics();

 private: // Configuration variables

  std::string m_recap_var;
  std::string m_stat_recap_var;
  std::string m_spd_policy_var;

  double m_on_tail_thresh;
  double m_alignment_thresh;
  double m_tracking_thresh;
  double m_rng_switch_thresh;

 private: // Exposed State variables
  ConvoyRecap     m_recap;
  ConvoyStatRecap m_stat_recap;
  ConvoySpdPolicy m_spd_policy;
  ConvoyOrderDetector m_order_detector;
  
  unsigned int m_recap_rcvd;
  unsigned int m_stat_recap_rcvd;
  unsigned int m_spd_policy_rcvd;
  
  bool   m_on_tail;
  bool   m_aligned;
  bool   m_tethered;
  bool   m_fastened;
  bool   m_tracking;

  double m_time_on_tail;
  double m_time_aligned;
  double m_time_tethered;
  double m_time_fastened;
  double m_time_tracking;
  
  double m_pct_time_on_tail;
  double m_pct_time_aligned;
  double m_pct_time_tethered;
  double m_pct_time_fastened;
  double m_pct_time_tracking;

  unsigned int m_rng_switches;
  std::string  m_rng_side;
    
  bool   m_attained_on_tail;
  bool   m_attained_aligned;
  bool   m_attained_tethered;
  bool   m_attained_fastened;
  bool   m_attained_tracking;

  double m_time_attained_on_tail;
  double m_time_attained_aligned;
  double m_time_attained_tethered;
  double m_time_attained_fastened;  
  double m_time_attained_tracking;  

  std::map<double, unsigned int> m_map_track_err_bins;
  double m_track_err_snap;
  
 private: // Internal State variables
  double m_curr_time;
  double m_prev_time;
  double m_tstamp_first_recap;
  
  double m_tstamp_attained_on_tail;
  double m_tstamp_attained_aligned;
  double m_tstamp_attained_tethered;
  double m_tstamp_attained_fastened;  
  double m_tstamp_attained_tracking;  
};

#endif 


