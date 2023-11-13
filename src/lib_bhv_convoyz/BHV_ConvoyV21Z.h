/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_Convoy21X.h                                       */
/*    DATE: Oct 5th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef BHV_CONVOY_V21X_HEADER
#define BHV_CONVOY_V21X_HEADER

#include <string>
#include <list>
#include "VarDataPair.h"
#include "IvPContactBehavior.h"
#include "ConvoyMarker.h"
#include "ConvoySpdPolicy.h"
#include "MarkerTail.h"

class IvPDomain;
class BHV_ConvoyV21X : public IvPContactBehavior {
public:
  BHV_ConvoyV21X(IvPDomain);
  ~BHV_ConvoyV21X() {}

public: // Overloaded virtual functions
  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onRunToIdleState() ;
  std::string  checkParamCollective();
  std::string  expandMacros(std::string);

protected:
  double getRelevance();
  double getPriority();
  IvPFunction *buildOF();

  bool   handleConfigHoldingPolicy(std::string);  
  bool   handleParamVisualHints(std::string);  
  bool   handleParamCompression(std::string);  
  void   handleNewContactSpd(double);
  
  void   drawMarker(ConvoyMarker, int vertex_size=-1,
		    std::string color="");
  void   eraseMarker(ConvoyMarker);

  void   updateMetrics();

  bool   checkDropAftMarker();

  void   setCurrentMarker();
  void   clearMarkerTail();  

  void   postRecap(bool);
  void   postSpdPolicy();

  bool   handleMarkerUpdates();

protected: // State variables
  MarkerTail m_marker_tail;

  std::list<double> m_cn_spd_value;
  std::list<double> m_cn_spd_tstamp;
  
  double m_wptx;
  double m_wpty;
  bool   m_wpt_set;
  bool m_is_leader;
  
  double m_set_speed;
  
  double m_cnv_avg_2sec;
  double m_cnv_avg_5sec;

protected: // State variables (metrics)
  double m_convoy_range;
  double m_range_delta;
  double m_tail_range;
  double m_tail_angle;
  double m_marker_bng;
  double m_track_error;
  double m_alignment; 

  unsigned int m_reached_markers;
  unsigned int m_dropped_markers;

  unsigned int m_recap_index;
  unsigned int m_stat_recap_index;
  
private: // Configuration parameters
  double m_capture_radius;
  double m_slip_radius;
  double m_compression;
  bool   m_aft_patience;

  double m_patience; // [1,99]

  std::string m_holding_policy;
  
  ConvoySpdPolicy m_spd_policy_base;
  ConvoySpdPolicy m_spd_policy;

  bool m_active_convoying;
  bool m_has_announced_contact;

  std::vector<VarDataPair> m_marker_flags;
  std::vector<VarDataPair> m_convoy_flags;

private: // Config Visual hints, output
  std::string m_hint_marker_color;
  std::string m_hint_marker_label_color;
  double      m_hint_marker_size;
  bool        m_post_recap_verbose;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name,
						   IvPDomain domain)
  {return new BHV_ConvoyV21X(domain);}
}
#endif


