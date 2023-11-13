/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyOrderDetector.cpp                              */
/*    DATE: July 17th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iostream>
#include "ConvoyOrderDetector.h"
#include "MBUtils.h"

using namespace std;

//---------------------------------------------------------------
// Procedure: getLeader()

string ConvoyOrderDetector::getLeader() const
{
  if(m_convoy.size() == 0)
    return("");

  return(m_convoy.front());
}

//---------------------------------------------------------------
// Procedure: getCaboose()

string ConvoyOrderDetector::getCaboose() const
{
  if(m_convoy.size() == 0)
    return("");

  return(m_convoy.back());
}

//------------------------------------------------------------
// Procedure: addPairing()
//      Note: A pairing is in the form of "follower,leader"
//   Returns: true if the pairing is syntactically valid.

bool ConvoyOrderDetector::addPairing(string pairing)
{
  string orig_pairing = tolower(pairing);
  
  // Part 1: Sanity checks
  if(!isValidPairing(pairing))
    return(false);  

  // Part 2: Does this pairing conflict with any previous?
  // If so, remove the older one. This is a dynamic structure.
  pairing = tolower(pairing);
  string follower = biteStringX(pairing, ',');
  string leader   = pairing;

  bool duplicate = false;
  list<string>::iterator p;
  for(p=m_pairs.begin(); p!=m_pairs.end(); ) {
    string apairing  = *p;
    string afollower = biteStringX(apairing, ',');
    string aleader   = apairing;
    bool conflict = false;
    if((follower == afollower) && (leader != aleader))
      conflict = true;
    if((leader == aleader) && (follower != afollower))
      conflict = true;

    if((leader == aleader) && (follower == afollower))
      duplicate = true;

    if(conflict)
      p = m_pairs.erase(p);
    else
      ++p;
  }

  // Part 3: If this is not a duplicate, add it
  if(!duplicate)
    m_pairs.push_back(orig_pairing);
  
  return(true);
}

//------------------------------------------------------------
// Procedure: removeVehicle()

bool ConvoyOrderDetector::removeVehicle(string vname)
{
  vname = tolower(vname);
  list<string>::iterator p;
  for(p=m_pairs.begin(); p!=m_pairs.end(); ) {
    string pairing  = *p;
    string follower = tolower(biteStringX(pairing, ','));
    string leader   = tolower(pairing);

    if((follower == vname) || (leader == vname)) {
      p = m_pairs.erase(p);
      m_removals++;
    }
    else
      ++p;
  }

  cout << "RemPairs:" << stringListToString(m_pairs, ':');
  
  return(true);
}

//------------------------------------------------------------
// Procedure: findConvoy()

bool ConvoyOrderDetector::findConvoy()
{
  m_convoy.clear();
  m_pairs_pending = m_pairs;

  //cout << "Pairs: " << stringListToString(m_pairs, ':') << endl;
  
  bool progress = true;
  while(progress) 
    progress = resolvePendingPairings();
  
  return(true);
}

//------------------------------------------------------------
// Procedure: resolvePendingPairings()
//   Returns: true if a pairing was added to the convoy

bool ConvoyOrderDetector::resolvePendingPairings()
{
  if(m_pairs_pending.size() == 0)
    return(false);

  list<string> pairs_pending = m_pairs_pending;
  m_pairs_pending.clear();

  bool at_least_one_pairing_added = false;
  
  list<string>::iterator p;
  for(p=pairs_pending.begin(); p!=pairs_pending.end(); p++) {
    string pairing = *p;
    bool pairing_added = addPairingAux(pairing);
    if(pairing_added)
      at_least_one_pairing_added = true;
    else
      m_pairs_pending.push_back(pairing);
  }

  return(at_least_one_pairing_added);
}

//------------------------------------------------------------
// Procedure: addPairingAux()
//      Note: A valid pairing is assumed, so the Boolean return
//            value represents whether the valid pairing was
//            currently possible.

bool ConvoyOrderDetector::addPairingAux(string pairing)
{
  pairing = tolower(pairing);
  string follower = biteStringX(pairing, ',');
  string leader   = pairing;

  // Part 1: Handle case of initially empty convoy
  if(m_convoy.size() == 0) {
    m_convoy.push_front(follower);
    m_convoy.push_front(leader);
    return(true);
  }

  // Part 2: Handle case where new pairing is tacked onto front
  if(follower == m_convoy.front()) {
    // make sure the pairing leader is not already in the convoy
    if(!listContains(m_convoy, leader)) {
      m_convoy.push_front(leader);
      return(true);
    }
  }
  // Part 3: Handle case where new pairing is tacked onto back
  if(leader == m_convoy.back()) {
    // make sure the pairing follower is not already in the convoy
    if(!listContains(m_convoy, follower)) {
      m_convoy.push_back(follower);
      return(true);
    }
  }

  // Part 4: check if both the follower and leader are already in the
  // convoy list. Since conflicting pairings are filtered upon
  // addition, this could mean a circular convoy. So do not process.
  if(listContains(m_convoy, follower) && listContains(m_convoy, leader))
    return(false);
  
  // Returning false simply means "no progress"
  return(false);
}

//------------------------------------------------------------
// Procedure: isValidPairing()
//      Note: A pairing is in the form of "follower,leader"

bool ConvoyOrderDetector::isValidPairing(string pairing) const
{
  pairing = tolower(pairing);
  string follower = biteStringX(pairing, ',');
  string leader   = pairing;

  if((follower == "") || (leader == ""))
    return(false);
  if(follower == leader)
    return(false);

  return(true);
}


//---------------------------------------------------------
// Procedure: getConvoyVector()

vector<string> ConvoyOrderDetector::getConvoyVector() const
{
  vector<string> rvector;

  list<string>::const_reverse_iterator p;
  for(p=m_convoy.rbegin(); p!=m_convoy.rend(); p++)
    rvector.push_back(*p);
  
  return(rvector);
}

//---------------------------------------------------------
// Procedure: getConvoySummary()

string ConvoyOrderDetector::getConvoySummary() const
{
  string summary;

  list<string>::const_iterator p;
  for(p=m_convoy.begin(); p!=m_convoy.end(); p++)  {
    string vname = *p;
    if(summary != "")
      summary += "<--";
    summary += vname;
  }

  return(summary);
}

//---------------------------------------------------------
// Procedure: isValid()

bool ConvoyOrderDetector::isValid() const
{
  if(m_convoy.size() == 0)
    return(false);
  //if(m_pairs_pending.size() != 0)
  //  return(false);
  
  return(true);
}


