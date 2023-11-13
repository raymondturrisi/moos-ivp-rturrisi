/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ConvoyMarker.h                                       */
/*    DATE: Jun 25th, 2021                                       */
/*    DATE: Oct 5th, 2021 Added TStamp and vname                 */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef CONVOY_MARKER
#define CONVOY_MARKER

#include <string>

class ConvoyMarker {
public:
  ConvoyMarker(double x, double y, unsigned int id=0);
  ConvoyMarker();
  ~ConvoyMarker() {}

  void setXY(double x, double y) {m_x=x; m_y=y; m_set=true;}
  void setID(unsigned int id)    {m_id=id;}
  void setVName(std::string s)   {m_vname=s;}
  void setUTC(double v)          {m_utc=v;}
  
  double getX() const        {return(m_x);}
  double getY() const        {return(m_y);}
  unsigned int getID() const {return(m_id);}
  std::string getVName() const {return(m_vname);}
  double getUTC() const      {return(m_utc);}
  
  double distToMarker(double x, double y);

  bool valid() const {return(m_set);}

  std::string getSpec(std::string vname="") const;
  
 private: 
  bool   m_set;
  double m_x;
  double m_y;
  unsigned int m_id;
  std::string m_vname;
  double m_utc;
};

ConvoyMarker string2ConvoyMarker(std::string);

#endif


