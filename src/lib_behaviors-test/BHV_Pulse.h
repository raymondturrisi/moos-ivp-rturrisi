/************************************************************/
/*    NAME: Raymond Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Pulse_HEADER
#define Pulse_HEADER

#include "IvPBehavior.h"
#include <string>

class BHV_Pulse : public IvPBehavior {
public:
  BHV_Pulse(IvPDomain);
  ~BHV_Pulse(){};

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
  double m_range = 20;
  double m_pulse_duration = 4;
  std::string m_old_wpt_idx = "-1";

protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name,
                                                IvPDomain domain) {
  return new BHV_Pulse(domain);
}
}
#endif
