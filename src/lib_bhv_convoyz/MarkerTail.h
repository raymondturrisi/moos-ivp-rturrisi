/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: MarkerTail.h                                         */
/*    DATE: June 25th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef MARKER_TAIL_HEADER
#define MARKER_TAIL_HEADER

#include <string>
#include <list>
#include "ConvoyMarker.h"

class MarkerTail {
public:
  MarkerTail();
  ~MarkerTail() {}

public:
  void   setPosCN(double cnx, double cny);
  bool   setInterMarkRange(std::string);
  void   setInterMarkRange(double);
  void   setMaxGhostMarkers(unsigned int v) {m_max_ghost_markers=v;}

  bool   setMaxTailLength(std::string);
  void   setMaxTailLength(double);

  double getInterMarkRange() const {return(m_inter_mark_range);}
  double getMaxTailLength() const  {return(m_tail_length_max);}

  bool   handleNewActiveMarker(ConvoyMarker);
  bool   handleNewContactPos(double cnx, double cny, double cnh);
  bool   checkDropAftMarker(std::string& msg);
  void   dropAftMarker();
  
  void   clear();
  
  unsigned int size() const        {return(m_markers.size());}
  bool   empty() const             {return(m_markers.empty());}
  double getMarkerTailLen() const  {return(m_marker_tail_length);}  
  
  ConvoyMarker getLeadMarker() const;
  ConvoyMarker getAftMarker() const;
  ConvoyMarker getNearAftMarker() const;

  double distToLeadMarker(double, double) const;
  double distToAftMarker(double, double) const;  
  double distToTail(double, double) const;  
  double tailAngle(double, double);
  double markerBearing(double, double, double) const;
  double getTrackError(double osx, double osy) const;
  bool   aftMarkerClosest(double osx, double osy) const;
  
  std::list<ConvoyMarker> getMarkers() const {return(m_markers);}
  std::list<ConvoyMarker> getClearedMarkers();

  std::string getTailAngleInfo() const {return(m_tail_ang_info);}

  std::string getMarkerStr();
  std::string getTailType() const {return(m_tail_type);}

protected:  
  void   updateCoreTailLen();
  void   updateMarkerTailLen();
  void   updateMarkerTailNext();
  double distTailToContact();

  
protected: // State variables
  std::list<ConvoyMarker> m_markers;

  std::list<ConvoyMarker> m_ghost_markers;

  std::list<ConvoyMarker> m_cleared_markers;

  
  double m_cnx;
  double m_cny;

  double m_marker_tail_length;
  double m_core_tail_length;
  
  unsigned int m_marker_id;
  unsigned int m_marker_id_max_val;

  std::string m_tail_ang_info;

  std::string m_tail_type;
  
private: // Configuration parameters

  double m_inter_mark_range;
  double m_tail_length_max;
  unsigned int m_max_ghost_markers;
  
};

#endif


