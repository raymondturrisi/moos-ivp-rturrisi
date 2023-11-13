/************************************************************/
/*    NAME: .                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ConvoyPD.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef ConvoyPD_HEADER
#define ConvoyPD_HEADER

#include <string>
#include "IvPBehavior.h"
#include <list> 
#include <map> 
#include "ConvoyPointQueue.h"




class AgentInfo {
  public:
    std::string name;
    double x;
    double y;
    double z;
    double h;
    double speed;
    double utc;
};


class BHV_ConvoyPD : public IvPBehavior {
public:
  BHV_ConvoyPD(IvPDomain);
  ~BHV_ConvoyPD() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  void         updateMessages();
  IvPFunction* onRunState();
  

protected: // Local Utility functions

protected: // Configuration parameters

//Parameters
bool m_is_leader, m_is_midship, m_is_tail;

double m_desired_speed;
double m_point_update_distance;
double m_ideal_follow_range;
double m_max_tail_length;

double kp_spd, kd_spd, ki_spd;
double kp_hdg, kd_hdg, ki_hdg;

std::string m_des_spd_k;
std::string m_point_update_dist_k;

protected: // State variables
double m_osx, m_osy, m_osh;
double m_osx_prv, m_osy_prv, m_osh_prv;
double m_osx_tprv, m_osy_tprv, m_osh_tprv;
double m_speed, m_osh_dot;

XYPoint m_ownship;
XYPoint m_target;
std::string m_contact;
AgentInfo m_contact_info;
std::map<std::string,AgentInfo> m_contacts_lookup;
std::string m_contact_list_str;
std::vector<std::string> m_contact_list;
std::string m_type_assignment;
std::string m_task_state;
bool m_has_broadcast_contact;
bool m_has_broadcast_leadership;


double m_interval_odo;
double m_eps;
unsigned long int m_posted_points;
std::map<std::string,std::string> m_follower_to_leader_mapping;
std::map<std::string,std::string> m_leader_to_follower_mapping;
std::vector<std::string> m_ordering_vector;
size_t m_place_in_convoy;
std::string m_follower;

std::vector<XYPoint> m_previous_points;
ConvoyPointQueue m_cpq;


//Messages
std::string m_nav_x_k;
std::string m_nav_y_k;
std::string m_nav_h_k;
std::string m_nav_spd_k;
std::string m_leader_k;
std::string m_contact_k;
std::string m_agent_info_k;
std::string m_lead_point_k;
std::string m_contact_list_k;
std::string m_task_state_k;
std::string m_nml_k;
std::string m_whotowho_k;
std::string m_updates_var_k;
std::string m_updates_buffer;

double m_latest_buffer_time;

};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ConvoyPD(domain);}
}
#endif
