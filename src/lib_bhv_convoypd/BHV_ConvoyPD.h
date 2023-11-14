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
#include <cstdarg> //va_list, va_start, va_end

class AgentInfo
{
public:
  std::string name; //0
  double x; //1
  double x_dot; //2
  double y; //3
  double y_dot; //4
  double z; //5
  double z_dot; //6
  double h; //7
  double h_dot; //8
  double u; //9
  double v; //10
  double utc; //11
  std::string color; //12

  AgentInfo() {
    name = "LARRY!";
    x = 0;
    x_dot = 0;
    y = 0;
    y_dot = 0;
    z = 0;
    z_dot = 0;
    h = 0;
    h_dot = 0;
    u = 0;
    v = 0;
    utc = 0;
    color = "yellow";
  }

  AgentInfo(std::string strrep) {
    std::vector<std::string> fields;
    if(strrep.front() == '{' && strrep.back() == '}') {
      strrep.pop_back();
      strrep.erase(0,1);
    }
    fields = parseString(strrep,',');

    for(int i = 0; i < fields.size(); i++) {
      biteString(fields[i],'=');
      switch (i)
      {
      case 0:
        name = fields[i];
        break;
      case 1:
        x = stod(fields[i]);
        break;
      case 2:
        x_dot = stod(fields[i]);
        break;
      case 3:
        y = stod(fields[i]);
        break;
      case 4:
        y_dot = stod(fields[i]);
        break;
      case 5:
        z = stod(fields[i]);
        break;
      case 6:
        z_dot = stod(fields[i]);
        break;
      case 7:
        h = stod(fields[i]);
        break;
      case 8:
        h_dot = stod(fields[i]);
        break;
      case 9:
        u = stod(fields[i]);
        break;
      case 10:
        v = stod(fields[i]);
        break;
      case 11:
        utc = stod(fields[i]);
        break;
      case 12:
        color = fields[i];
        break;
      default:
        break;
      }
    }
  }

  // std::string repr() {
  //   return repr(",");
  // }
  std::string repr(std::string delim=",") {
    std::string result;
    result="{" 
      + string("name=") + name 
      + delim 
      + string("x=") +floatToString(x,2) 
      + delim 
      + string("x_dot=") +floatToString(x_dot,2) 
      + delim 
      + string("y=") +floatToString(y,2)
      + delim 
      + string("y_dot=") +floatToString(y_dot,2) 
      + delim
      + string("z=") +floatToString(z,2) 
      + delim
      + string("z_dot=") +floatToString(z_dot,2) 
      + delim 
      + string("h=") +floatToString(h,2)
      + delim
      + string("h_dot=") +floatToString(h_dot,2)
      + delim
      + string("u=") +floatToString(u,2)
      + delim
      + string("v=") +floatToString(v,2)
      + delim
      + string("utc=") +floatToString(utc,3)
      + delim
      + color 
      + "}";
    ;
    return result;
  }
};

class BHV_ConvoyPD : public IvPBehavior
{
public:
  BHV_ConvoyPD(IvPDomain);
  ~BHV_ConvoyPD(){};

  bool setParam(std::string, std::string);
  void onSetParamComplete();
  void onCompleteState();
  void onIdleState();
  void onHelmStart();
  void postConfigStatus();
  void onRunToIdleState();
  void onIdleToRunState();

  IvPFunction *onRunState();

protected: // Local Utility functions
  void updateMessages();
  void postStateMessages();
  void postAgentInfo();
  void updateAgentInfo(std::string name);
  void updateOwnshipState();
  void updateCapturePoint();
  void propagatePoint(ConvoyPoint prv_cp);
  void updateIsLeader();
  void updateExtOrdering();
  void updateContactList();
  void updateCheckForContact();
  void updateLeadPoint();
  void updateFtoLMapping();
  void updateContactInfo();
  void handleUpdateVar();
  void handleNodeReport();
  void clamp(double &value, double min, double max);

  void postLeadership();
  void postOrdering();

  void generalBroadcasts();

  void seedPoints();

  bool shouldRepost(std::string field);

  void scheduleRepost(std::string field);

  bool dbg_print(const char *format, ...);

protected: // Configuration parameters
  // Parameters
  bool m_is_leader, m_is_midship, m_is_tail;

  double m_desired_speed;
  double m_point_update_distance;
  double m_ideal_follow_range;
  double m_max_tail_length;

  double m_redudant_update_interval;
  std::map<std::string, double> m_next_redundant_update;

  double kp_spd, kd_spd, ki_spd;
  double kp_hdg, kd_hdg, ki_hdg;

  std::string m_des_spd_k;
  std::string m_point_update_dist_k;

protected: // State variables
  double m_osx, m_osy, m_osh;
  double m_osx_prv, m_osy_prv, m_osh_prv;
  double m_osx_tprv, m_osy_tprv, m_osh_tprv;
  double m_speed, m_osh_dot;

  std::string m_color;

  XYPoint m_ownship;
  XYPoint m_target;
  std::string m_contact;
  AgentInfo m_self_agent_info;
  std::map<std::string, AgentInfo> m_contacts_lookup;
  std::string m_contact_list_str;
  std::vector<std::string> m_contact_list;
  std::string m_type_assignment;
  std::string m_task_state;
  bool m_has_broadcast_contact;
  bool m_has_broadcast_leadership;

  bool m_debug;
  FILE *m_cfile;
  std::string m_debug_fname;

  double m_interval_odo;
  double m_eps;
  unsigned long int m_posted_points;
  std::map<std::string, std::string> m_follower_to_leader_mapping;
  std::map<std::string, std::string> m_leader_to_follower_mapping;
  std::vector<std::string> m_ordering_vector;
  std::string m_ordering_str;
  size_t m_place_in_convoy;
  std::string m_follower;

  std::vector<XYPoint> m_previous_points;
  ConvoyPointQueue m_cpq;

  // Messages
  std::string m_nav_x_k;
  std::string m_nav_y_k;
  std::string m_nav_h_k;
  std::string m_nav_spd_k;
  std::string m_leader_k;
  std::string m_contact_k;
  std::string m_ext_ordering_k;
  std::string m_agent_info_k;
  std::string m_lead_point_k;
  std::string m_contact_list_k;
  std::string m_task_state_k;
  std::string m_nml_k;
  std::string m_nrl_k;
  std::string m_whotowho_k;
  std::string m_updates_var_k;
  std::string m_updates_buffer;
  XYPoint m_prev_err_point;
  double m_latest_buffer_time;
};

#define IVP_EXPORT_FUNCTION

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name, IvPDomain domain)
  {
    return new BHV_ConvoyPD(domain);
  }
}
#endif
