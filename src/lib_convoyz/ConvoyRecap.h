/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyRecap.h                                        */
/*    DATE: July 17th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef CONVOY_RECAP_HEADER
#define CONVOY_RECAP_HEADER

#include <string>
#include <map>
#include <stdlib.h>  // atof()

class ConvoyRecap
{
 public:
  ConvoyRecap();
  ~ConvoyRecap() {}

  // Setters
  void setConvoyRng(double v)      {m_convoy_rng=v;}
  void setConvoyRngDelta(double v) {m_convoy_rng_delta=v;}
  void setTailRng(double v)        {m_tail_rng=v;}
  void setTailAng(double v)        {m_tail_ang=v;}
  void setMarkerBng(double v)      {m_mark_bng=v;}
  void setTrackErr(double v)       {m_track_err=v;}
  void setAlignment(double v)      {m_alignment=v;}
  void setSetSpd(double v)         {m_set_spd=v;}
  void setAvg2(double v)           {m_cnv_avg2=v;}
  void setAvg5(double v)           {m_cnv_avg5=v;}

  void setCorrMode(std::string s)  {m_correction_mode=s;}
  void setVName(std::string s)     {m_vname=s;}
  void setCName(std::string s)     {m_cname=s;}
  
  void setTimeUTC(double v)        {m_time_utc=v;}
  void setMarkerX(double v)        {m_marker_x=v; m_marker_x_set=true;}
  void setMarkerY(double v)        {m_marker_y=v; m_marker_y_set=true;}
  void setMarkerID(unsigned int v) {m_marker_id=v; m_marker_id_set=true;}
  void setTailCnt(unsigned int v)  {m_tail_cnt=v; m_tail_cnt_set=true;}
  void setIndex(unsigned int v)    {m_index=v; m_index_set=true;}

  void setIdle(bool v=true)        {m_idle=v;}
  
  // Getters
  double getConvoyRng() const      {return(m_convoy_rng);}
  double getConvoyRngDelta() const {return(m_convoy_rng_delta);}

  double getTailRng() const        {return(m_tail_rng);}
  double getTailAng() const        {return(m_tail_ang);}
  double getMarkerBng() const      {return(m_mark_bng);}
  double getTrackErr() const       {return(m_track_err);}
  double getAlignment() const      {return(m_alignment);}
  double getSetSpd() const         {return(m_set_spd);}
  double getAvg2() const           {return(m_cnv_avg2);}
  double getAvg5() const           {return(m_cnv_avg5);}

  std::string getCorrMode() const  {return(m_correction_mode);}
  std::string getVName() const     {return(m_vname);}
  std::string getCName() const     {return(m_cname);}
  
  double getTimeUTC() const        {return(m_time_utc);}
  double getMarkerX() const        {return(m_marker_x);}
  double getMarkerY() const        {return(m_marker_y);}
  unsigned int getMarkerID() const {return(m_marker_id);}
  unsigned int getTailCnt() const  {return(m_tail_cnt);}
  unsigned int getIndex() const    {return(m_index);}

  bool getIdle() const             {return(m_idle);}

  // IsSetters
  bool   isSetConvoyRng() const    {return(m_convoy_rng>=0);}
  bool   isSetConvoyRngDelta() const {return(m_convoy_rng_delta>=0);}
  bool   isSetTailRng() const      {return(m_tail_rng>=0);}
  bool   isSetTailAng() const      {return(m_tail_ang>=0);}
  bool   isSetMarkerBng() const    {return(m_mark_bng>=0);}
  bool   isSetTrackErr() const     {return(m_track_err>=0);}
  bool   isSetAlignment() const    {return(m_alignment>=0);}
  bool   isSetSetSpd() const       {return(m_set_spd>=0);}
  bool   isSetAvg2() const         {return(m_cnv_avg2>=0);}
  bool   isSetAvg5() const         {return(m_cnv_avg5>=0);}

  bool   isSetCorrMode() const     {return(m_correction_mode!="");}
  bool   isSetVName() const        {return(m_vname!="");}
  bool   isSetCName() const        {return(m_cname!="");}
  
  bool   isSetTimeUTC() const      {return(m_time_utc>=0);}
  bool   isSetMarkerX() const      {return(m_marker_x_set);}
  bool   isSetMarkerY() const      {return(m_marker_y_set);}
  bool   isSetMarkerID() const     {return(m_marker_id_set);}
  bool   isSetTailCnt() const      {return(m_tail_cnt_set);}
  bool   isSetIndex() const        {return(m_index_set);}

  // Serialization
  std::string getStringValue(std::string key) const;
  std::string getSpec() const;

 protected: 
  double m_convoy_rng;
  double m_convoy_rng_delta;

  double m_tail_rng;
  double m_tail_ang;
  double m_mark_bng;
  double m_alignment;
  double m_track_err;
  double m_set_spd;
  double m_cnv_avg2;
  double m_cnv_avg5;

  std::string m_correction_mode;
  std::string m_vname;
  std::string m_cname;
  
  double m_time_utc;
  double m_marker_x;
  double m_marker_y;
  unsigned int m_marker_id;
  unsigned int m_tail_cnt;
  unsigned int m_index;

  bool m_marker_x_set;
  bool m_marker_y_set;
  bool m_marker_id_set;
  bool m_tail_cnt_set;
  bool m_index_set;

  bool m_idle;
};

ConvoyRecap string2ConvoyRecap(std::string);

#endif 


