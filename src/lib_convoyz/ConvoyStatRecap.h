/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyStatRecap.h                                    */
/*    DATE: July 17th 2021                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef CONVOY_STAT_RECAP_HEADER
#define CONVOY_STAT_RECAP_HEADER

#include <string>
#include <map>
#include <stdlib.h>  // atof()

class ConvoyStatRecap
{
 public:
  ConvoyStatRecap();
  ~ConvoyStatRecap() {}

  void setLeader(std::string s)    {m_leader=s;}
  void setFollower(std::string s)  {m_follower=s;}
  void setIdealRng(double v)       {m_ideal_rng=v;}
  void setCompression(double v)    {m_compression=v;}
  void setIndex(unsigned int v)    {m_index=v;}
  void setIdle(bool v=true)        {m_idle=v;}

  std::string getLeader() const    {return(m_leader);}
  std::string getFollower() const  {return(m_follower);}
  double getIdealRange() const     {return(m_ideal_rng);}
  double getCompression() const    {return(m_compression);}
  unsigned int getIndex() const    {return((unsigned int)(m_index));}
  bool   getIdle() const           {return(m_idle);}
  
  bool   isSetLeader() const       {return(m_leader!="");}
  bool   isSetFollower() const     {return(m_follower!="");}
  bool   isSetIdealRng() const     {return(m_ideal_rng>=0);}
  bool   isSetCompression() const  {return(m_compression>=0);}
  //bool   isSetIndex() const        {return(m_index_set);}

  std::string getStringValue(std::string key) const;
  
  std::string getSpec() const;

 protected: 
  std::string m_leader;
  std::string m_follower;
  
  double m_ideal_rng;
  double m_compression;
  int    m_index;

  bool   m_idle;
};

ConvoyStatRecap string2ConvoyStatRecap(std::string);

#endif 


