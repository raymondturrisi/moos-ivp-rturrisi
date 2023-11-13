/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoySpdPolicy.h                                    */
/*    DATE: June 29th, 2021, broken out to separate class        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef CONVOY_SPD_POLICY_HEADER
#define CONVOY_SPD_POLICY_HEADER

#include <string>
#include <vector>
#include <stdlib.h>  // atof()

class ConvoySpdPolicy {
public:
  ConvoySpdPolicy();
  ~ConvoySpdPolicy() {}

public: // setters
  void   setFullStopConvoyRng(double);
  void   setSlowerConvoyRng(double);
  void   setIdealConvoyRng(double);
  void   setFasterConvoyRng(double);
  void   setFullLagConvoyRng(double);
  void   setMaxCompression(double);
  void   setLagSpeedDelta(double);
  void   setPolicyName(std::string s) {m_policy_name=s;}
  void   setVName(std::string s)      {m_vname=s;}
  bool   setParam(std::string, std::string);

public: // getters
  double getFullStopConvoyRng() const {return(m_full_stop_convoy_rng);}
  double getSlowerConvoyRng() const   {return(m_slower_convoy_rng);}
  double getIdealConvoyRng() const    {return(m_ideal_convoy_rng);}
  double getFasterConvoyRng() const   {return(m_faster_convoy_rng);}
  double getFullLagConvoyRng() const  {return(m_full_lag_convoy_rng);}
  double getLagSpeedDelta() const     {return(m_lag_speed_delta);}
  double getMaxCompression() const    {return(m_max_compression);}
  std::string getPolicyName() const   {return(m_policy_name);}
  //std::string getVName() const        {return(m_vname);}

  std::string status();
  std::string getCorrectionMode() const {return(m_correction_mode);}
  
  double getSpdFromPolicy(double, double, double);

  bool   compress(double pct);


  bool   atIdealConvoyRng(double);

  std::string getSpec() const;
  std::string getTerse() const;
  
private: // Config params
  double m_full_stop_convoy_rng;
  double m_slower_convoy_rng;
  double m_ideal_convoy_rng;
  double m_faster_convoy_rng;
  double m_full_lag_convoy_rng;
  
  double m_lag_speed_delta;
  double m_max_compression;

  std::string m_policy_name;
  std::string m_vname;
  
private: // State vars

  std::string m_correction_mode;
};

ConvoySpdPolicy string2ConvoySpdPolicy(std::string);

#endif


