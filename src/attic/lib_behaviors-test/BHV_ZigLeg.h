/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef ZigLeg_HEADER
#define ZigLeg_HEADER

#include "IvPBehavior.h"
#include <string>

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg(){};

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
protected: // Configuration parameters
  std::string m_old_wpt_idx = "-1";
  double m_zig_duration = 10;
  double m_zig_angle = 45;
  double m_zig_init_time = -1;
  double m_init_heading = 0;
  double m_zig_boot_time = 0;
  bool m_boot = false;
protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name,
                                                IvPDomain domain) {
  return new BHV_ZigLeg(domain);
}
}
#endif
