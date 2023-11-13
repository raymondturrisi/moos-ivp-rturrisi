/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyOrderDetector.h                                */
/*    DATE: July 25th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef CONVOY_ORDER_DETECTOR_HEADER
#define CONVOY_ORDER_DETECTOR_HEADER

#include <string>
#include <list>
#include <vector>
#include "MBUtils.h"

class ConvoyOrderDetector
{
public:
  ConvoyOrderDetector() {m_removals=0;}
  ~ConvoyOrderDetector() {}

  bool addPairing(std::string pairing);
  bool removeVehicle(std::string vname);
  
  bool findConvoy();
  
  std::string getLeader() const;
  std::string getCaboose() const;

  std::vector<std::string> getConvoyVector() const;

  std::string getConvoySummary() const;

  std::string getPairsStr() {return(stringListToString(m_pairs, ':'));}
  
  bool isValid() const;

  unsigned int getRemovalCnt() const {return(m_removals);}

protected:
  bool addPairingAux(std::string pairing);
  bool isValidPairing(std::string pairing) const;
  bool resolvePendingPairings();

protected:
  std::list<std::string> m_convoy;
  std::list<std::string> m_pairs;
  std::list<std::string> m_pairs_pending;

  unsigned int m_removals;
  
};

#endif 


