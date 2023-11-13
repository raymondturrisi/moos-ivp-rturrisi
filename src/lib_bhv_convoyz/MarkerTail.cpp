/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: MarkerTail.cpp                                       */
/*    DATE: June 25th, 2021                                      */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cmath>
#include <cstdlib>
#include "MarkerTail.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

MarkerTail::MarkerTail() 
{
  // Intialize Config variables
  m_inter_mark_range = 10;
  m_tail_length_max  = 150;
  m_max_ghost_markers = 5;
  
  // Intialize State variables
  m_cnx = 0;
  m_cny = 0;

  m_core_tail_length = 0;
  m_marker_tail_length = 0;
  m_marker_id_max_val = (m_tail_length_max / m_inter_mark_range) + 2;
  m_marker_id = 0;

  m_tail_type = "passive";
}

//-----------------------------------------------------------
// Procedure: setPosCN()

void MarkerTail::setPosCN(double cnx, double cny)
{
  m_cnx = cnx;
  m_cny = cny;
}

//-----------------------------------------------------------
// Procedure: setMaxTailLength(string)

bool MarkerTail::setMaxTailLength(string sval)
{
  if(!isNumber(sval))
    return(false);

  double dval = atof(sval.c_str());
  if(dval <= 0)
    return(false);

  setMaxTailLength(dval);
  return(true);
}

//-----------------------------------------------------------
// Procedure: setMaxTailLength()

void MarkerTail::setMaxTailLength(double v)
{
  m_tail_length_max = v;
  if(m_tail_length_max < 0)
    m_tail_length_max = 0;

  if(m_inter_mark_range > 0) {
    double amt = (m_tail_length_max / m_inter_mark_range) + 2;
    m_marker_id_max_val = (unsigned int)(amt);
  }
}

//-----------------------------------------------------------
// Procedure: setInterMarkRange(string)

bool MarkerTail::setInterMarkRange(string sval)
{
  if(!isNumber(sval))
    return(false);

  double dval = atof(sval.c_str());
  if(dval <= 0)
    return(false);

  setInterMarkRange(dval);
  return(true);
}

//-----------------------------------------------------------
// Procedure: setInterMarkRange()

void MarkerTail::setInterMarkRange(double v)
{
  m_inter_mark_range = v;
  if(m_inter_mark_range <= 0)
    m_inter_mark_range = 0.1;

  if(m_inter_mark_range > 0) {
    double amt = (m_tail_length_max / m_inter_mark_range) + 2;
    m_marker_id_max_val = (unsigned int)(amt);
  }
}

//-----------------------------------------------------------
// Procedure: handleNewActiveMarker()
//     Notes: The MarkerTail must be comprised solely of either
//              active or passive markers.
//            Passive markers are inferred from the contact
//              position and set with handleNewContactPos()
//            Active markers are explicitly given here.
//            If this MarkerTail was previously passive, then
//              this function call will switch it to active and
//              all previous markers will be cleared.

bool MarkerTail::handleNewActiveMarker(ConvoyMarker marker)
{
  if(!marker.valid())
    return(false);

  if(m_tail_type == "passive") 
    clear();

  // Receiving and active marker will always dictate that this
  // marker tail is active. Not the other way around.
  m_tail_type = "active";
  m_markers.push_front(marker);
  
  //update what will be the next marker id
  m_marker_id++;
  if(m_marker_id > m_marker_id_max_val)
    m_marker_id = 0;
  
  // core_tail_len needs to be recalculated with new marker
  updateCoreTailLen();
  updateMarkerTailLen();
  return(true);
}  

//-----------------------------------------------------------
// Procedure: handleNewContactPos()
//   Returns: true if new marker is added.
// 
//    ownship           next
//             oldest  oldest                newest  contact
//    o------------x-----x-----x-----x-----x-----x-------o 
//                 |-----------------------------|
//                      core_tail_length
//                 |-------------------------------------|
//                                  marker_tail_length         
//    |--------------------------------------------------|
//         convoy_range         
			 

bool MarkerTail::handleNewContactPos(double cnx, double cny, double cnh)
{
  cout << "MarkerTail::handleNewContactPos: type=" << m_tail_type << endl; 
  cout << "MarkerTail::handleNewContactPos: size=" << size() << endl; 
  // If type is not passive and there are existing markers
  //if((size() > 0) && (m_tail_type != "passive"))
  if(m_tail_type != "passive")
    return(false);

  m_tail_type = "passive";
  
  m_cnx = cnx;
  m_cny = cny;
  
  if(empty() || (distToLeadMarker(cnx, cny) >= m_inter_mark_range)) {
  
    // Add a new marker one meter behind the contact. This point is used
    // rather than cnx,cny so that the contact position always may serve
    // as a near-aft marker if needed.
    double mx, my;
    projectPoint((cnh + 180), 1, cnx, cny, mx, my);

    ConvoyMarker new_marker(mx, my, m_marker_id);
    m_markers.push_front(new_marker);

    //update what will be the next marker id
    m_marker_id++;
    if(m_marker_id > m_marker_id_max_val)
      m_marker_id = 0;
      
    // core_tail_len needs to be recalculated with new marker
    updateCoreTailLen();
  }
  
  updateMarkerTailLen();
  
  return(true);
}

//-----------------------------------------------------------
// Procedure: checkDropAftMarker()
//   Returns: true if marker is dropped

bool MarkerTail::checkDropAftMarker(string& msg)
{
  if(empty())
    return(false);
  if(m_marker_tail_length <= m_tail_length_max)
    return(false);

  msg = "size=" + uintToString(m_markers.size());
  msg += ",cnx=" + doubleToStringX(m_cnx,1);
  msg += ",cny=" + doubleToStringX(m_cny,1);
  msg += ",core_len=" + doubleToStringX(m_core_tail_length,1);
  msg += ",mt_len=" + doubleToStringX(m_marker_tail_length,1);
  msg += ",max=" + doubleToStringX(m_tail_length_max);
  
  cout << "MarkerTail::dropAftMarker +++++++++++++++++++" << endl;
  dropAftMarker();

  // Whenever the tail becomes empty, revert to being in the
  // passive mode. When/if an active marker is received, we
  // will then switch to being in active mode.  The below lines
  // ensure that if active markers stop, reverting to passive
  // markers is supported.
  if(size() == 0)
    m_tail_type = "passive";
    
  return(true);
}

//-----------------------------------------------------------
// Procedure: DropAftMarker()

void MarkerTail::dropAftMarker()
{
  if(empty())
    return;

  ConvoyMarker new_ghost_marker = m_markers.back();
  m_markers.pop_back();

  m_ghost_markers.push_front(new_ghost_marker);
  if(m_ghost_markers.size() > m_max_ghost_markers)
    m_ghost_markers.pop_back();
  
  // core_tail_len is sum of segments
  updateCoreTailLen();
  updateMarkerTailLen();
}

//-----------------------------------------------------------
// Procedure: clear()
//      Note: Just clear the data, not the configuration

void MarkerTail::clear()
{
  m_cleared_markers = m_markers;

  m_markers.clear();
  m_ghost_markers.clear();
  
  m_cnx = 0;
  m_cny = 0;

  m_marker_tail_length = 0;
  m_core_tail_length = 0;

  m_marker_id = 0;
}


//-----------------------------------------------------------
// Procedure: updateCoreTailLen()

void MarkerTail::updateCoreTailLen()
{
  m_core_tail_length = 0;

  if(m_markers.size() < 2)
    return;

  double prev_x = 0;
  double prev_y = 0;
  list<ConvoyMarker>::iterator p;
  for(p=m_markers.begin(); p!=m_markers.end(); p++) {
    double curr_x = p->getX();
    double curr_y = p->getY();
    if(p!=m_markers.begin())
      m_core_tail_length += hypot(curr_x - prev_x, curr_y - prev_y);
    prev_x = curr_x;
    prev_y = curr_y;
  }
}

//-----------------------------------------------------------
// Procedure: updateMarkerTailLen()
//      Note: marker_tail_len is core_tail_len plus range to contact

void MarkerTail::updateMarkerTailLen()
{
  m_marker_tail_length = 0;
  if(!m_markers.empty()) {
    m_marker_tail_length = m_core_tail_length;
    //    m_marker_tail_length += distToLeadMarker(m_cnx, m_cny);
    m_marker_tail_length += distTailToContact();
  }
}

//-----------------------------------------------------------
// Procedure: getLeadMarker()

ConvoyMarker MarkerTail::getLeadMarker() const
{
  ConvoyMarker null_marker;
  if(m_markers.empty())
    return(null_marker);
  
  return(m_markers.front());
}

//-----------------------------------------------------------
// Procedure: getAftMarker()

ConvoyMarker MarkerTail::getAftMarker() const
{
  ConvoyMarker null_marker;
  if(m_markers.empty())
    return(null_marker);

  return(m_markers.back());
}

//-----------------------------------------------------------
// Procedure: getNearAftMarker()

ConvoyMarker MarkerTail::getNearAftMarker() const
{
  ConvoyMarker null_marker;
  if(m_markers.size() < 2)
    return(null_marker);

  list<ConvoyMarker>::const_reverse_iterator p = m_markers.rbegin();
  p++;
  ConvoyMarker next_oldest_marker = *p;
  
  return(next_oldest_marker);
}

//-----------------------------------------------------------
// Procedure: getMarkerStr()

string MarkerTail::getMarkerStr()
{
  string rstr;
  
  list<ConvoyMarker>::const_iterator p;
  for(p=m_markers.begin(); p!= m_markers.end(); p++) {
    ConvoyMarker marker = *p;
    if(rstr != "")
      rstr += " : ";
    rstr += doubleToString(marker.getX(),1) + ",";
    rstr += doubleToString(marker.getY(),1);
  }

  return(rstr);
}

//-----------------------------------------------------------
// Procedure: getClearedMarkers()

list<ConvoyMarker> MarkerTail::getClearedMarkers()
{
  list<ConvoyMarker> cleared_markers = m_cleared_markers;
  m_cleared_markers.clear();
  return(cleared_markers);
}

//-----------------------------------------------------------
// Procedure: distToLeadMarker()

double MarkerTail::distToLeadMarker(double x, double y) const
{
  if(m_markers.empty())
    return(-1);

  double mx = m_markers.front().getX();
  double my = m_markers.front().getY();

  double dist = hypot(x-mx, y-my);

  return(dist);
}

//-----------------------------------------------------------
// Procedure: distToAftMarker()

double MarkerTail::distToAftMarker(double x, double y) const
{
  if(m_markers.empty())
    return(-1);

  double mx = m_markers.back().getX();
  double my = m_markers.back().getY();

  double dist = hypot(x-mx, y-my);

  return(dist);
}

//-----------------------------------------------------------
// Procedure: getTrackError()

double MarkerTail::getTrackError(double osx, double osy) const
{
  if(m_markers.empty() && m_ghost_markers.empty())
    return(-1);

  double aftx = m_markers.back().getX();
  double afty = m_markers.back().getY();

  XYSegList segl;
  segl.add_vertex(aftx, afty);

  list<ConvoyMarker>::const_iterator p;
  for(p=m_ghost_markers.begin(); p!=m_ghost_markers.end(); p++) {
    double gx = p->getX();
    double gy = p->getY();
    segl.add_vertex(gx, gy);
  }

  // Sanity check
  if(segl.size() == 0)
    return(-1);
  
  double dist = segl.dist_to_point(osx, osy);
  return(dist);
}

//-----------------------------------------------------------
// Procedure: tailAngle()
//      Note: The tail angle is (a) the relative bearing of the
//            aft marker to ownship, plus (b) the angle between
//            ownship, the aft marker and next-most aft marker.

double MarkerTail::tailAngle(double osx, double osy) 
{
  // Edge case 1: There are no markers. Tail angle is zero.
  if(size() == 0)
    return(0);

  ConvoyMarker marker = getAftMarker();
  double mx = marker.getX();
  double my = marker.getY();
  
  // Edge case 2: There is only one marker (no near-aft marker). The contact
  // position is instead used as the near-aft marker.
  ConvoyMarker near_aft_marker = ConvoyMarker(m_cnx, m_cny);
  if(size() > 1)
    near_aft_marker = getNearAftMarker();

  // Armed with a near-aft marker, calculate the tail angle
  double bx = near_aft_marker.getX();
  double by = near_aft_marker.getY();
  
  double tail_angle = angleFromThreePoints(mx, my, osx, osy, bx, by);

  // Build info string for debugging/logging if desired
  m_tail_ang_info  = doubleToString(mx,1) + ",";
  m_tail_ang_info += doubleToString(my,1) + " : ";
  m_tail_ang_info += doubleToString(osx,1) + ",";
  m_tail_ang_info += doubleToString(osy,1) + " : ";
  m_tail_ang_info += doubleToString(bx,1) + ",";
  m_tail_ang_info += doubleToString(by,1);
  
  // Sanity check, ensure marker_ang is in range [0, 180)
  tail_angle = angle180(tail_angle);
  if(tail_angle < 0)
    tail_angle = -tail_angle;
	   
  return(180 - tail_angle); 
}

//-----------------------------------------------------------
// Procedure: markerBearing()
//      Note: The marker_bearing is the relative bearing of the
//            aft marker to ownship. It there is no maker, it is
//            the relative bearing from the leader position to
//            ownship.

double MarkerTail::markerBearing(double osx, double osy, double osh) const
{
  // Edge case 1: There are no markers. marker_bearing is just the 
  // absolute relative bearing between ownship and the contact.
  if(size() == 0) {
    double relbng = absRelBearing(osx, osy, osh, m_cnx, m_cny);
    return(relbng);
  }
  
  double mx = m_markers.back().getX();
  double my = m_markers.back().getY();

  double relbng = absRelBearing(osx, osy, osh, mx, my);

  return(relbng);
}


//-----------------------------------------------------------
// Procedure: distTailToContact()
//      Note: The min distance between any marker in the tail
//            to the contact position

double MarkerTail::distTailToContact()
{
  double min_dist = -1;
  list<ConvoyMarker>::iterator p;
  for(p=m_markers.begin(); p!=m_markers.end(); p++) {
    ConvoyMarker marker = *p;
    double cx = marker.getX();
    double cy = marker.getY();
    double dist = hypot(m_cnx-cx, m_cny-cy);
    if((min_dist < 0) || (dist < min_dist))
      min_dist = dist;
  }
  return(min_dist);
}


//-----------------------------------------------------------
// Procedure: distToTail()
//      Note: The min distance between any marker in the tail
//            or the contact position, to the given position.

double MarkerTail::distToTail(double osx, double osy) const
{
  double min_dist = hypot(osx-m_cnx, osy-m_cny);

  list<ConvoyMarker>::const_iterator p;
  for(p=m_markers.begin(); p!=m_markers.end(); p++) {
    ConvoyMarker marker = *p;
    double cx = marker.getX();
    double cy = marker.getY();
    double dist = hypot(osx-cx, osy-cy);
    if(dist < min_dist)
      min_dist = dist;
  }
  return(min_dist);
}


//-----------------------------------------------------------
// Procedure: aftMarkerClosest()

bool MarkerTail::aftMarkerClosest(double osx, double osy) const
{
  // Edge case: when no markers are present we return true. In this
  // case we are asserting that the "aft marker" is the contact
  // position and there are no markers closer than that.
  if(m_markers.empty())
    return(true);

  if(distToTail(osx, osy) < distToAftMarker(osx, osy))
    return(false);

  return(true);
}


