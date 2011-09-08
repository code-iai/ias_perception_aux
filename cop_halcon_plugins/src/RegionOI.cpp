/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "Camera.h"
#include "RegionOI.h"
#include "RelPoseHTuple.h"
#include "SearchParams3d.h"
#include "SegmentPrototype.h"
#include <sstream>


using namespace Halcon;
using namespace cop;

RegionOI::RegionOI()
{
  Halcon::gen_empty_region(&m_reg);
}

RegionOI::RegionOI(RegionRuns row_colStart_colEnd)
{

  HTuple r, cs, ce;
  RegionRuns::const_iterator it = row_colStart_colEnd.begin();
  for (;it != row_colStart_colEnd.end(); it++)
  {
    r = r.Concat((*it).first);
    cs = cs.Concat((*it).second.first);
    ce = ce.Concat((*it).second.second);
  }
  Halcon::gen_region_runs(&m_reg,r, cs, ce);

}

RegionOI::RegionOI(RelPose* pose, LocatedObjectID_t cam_pose_id, Calibration* calib)
{

    Halcon::HTuple pose_s(7,0.0), extents;
  bool cov = true;
  Matrix m,ExtremePoses;
  double gravPoint[3];
  gravPoint[0] = 0.0;
  gravPoint[1] = 0.0;
  gravPoint[2] = 0.0;
#ifdef _DEBUG
  printf("RegionOI: Calculating new Search Region\n");
#endif /*_DEBUG*/
  try
  {
    if(pose->m_uniqueID != ID_WORLD)
    {
      Matrix dings = pose->GetMatrix(cam_pose_id);
      if(dings.element(0,0) >= 0.9999 &&
         dings.element(1,1) >= 0.9999 &&
         dings.element(2,2) >= 0.9999 &&
         fabs(dings.element(0,3)) <= 0.00001 &&
         fabs(dings.element(1,3)) <= 0.00001 &&
         fabs(dings.element(2,3)) <= 0.00001)
      {
        printf("Identity transform, no sense to calc search spaces.\n");
        cov = false;
      }
      else
      {
        m = pose->GetCovarianceMatrix();
        double d = m.trace();
        if (d == 0.0)
        {
          printf("RegionOI: Ignoring covariances with trace = 0\n");
          throw "RegionOI: Ignoring covariances with trace = 0\n";
        }
        else
        {
          m = pose->GetCovarianceMatrix();
        }
       }
      }
      else
        cov = false;
   }
   catch(...)
   {
     cov = false;
   }
   if(cov == true)
   {
     try
     {
       RelPoseHTuple::GetPose(pose, &pose_s, cam_pose_id);
       ExtremePoses = GetExtremePoses(m);
       extents=GetExtents(ExtremePoses.t(), pose_s, gravPoint, calib, this);
       printf("Assigned new region of size: %d\n", GetSize());
     }
     catch(...)
     {
       printf("Error in RegionOI!\n");

     }
   }

}


RegionOI::RegionOI(RelPose* pose, LocatedObjectID_t cam_pose_id, HTuple CamPar)
{

  Halcon::HTuple pose_s(7,0.0), extents;
  bool cov = true;
  Matrix m,ExtremePoses;
  double gravPoint[3];
  gravPoint[0] = 0.0;
  gravPoint[1] = 0.0;
  gravPoint[2] = 0.0;
#ifdef _DEBUG
  printf("RegionOI: Calculating new Search Region\n");
#endif /*_DEBUG*/
  try
  {
    if(pose->m_uniqueID != ID_WORLD)
    {
      Matrix dings = pose->GetMatrix(cam_pose_id);
      if(dings.element(0,0) >= 0.9999 &&
         dings.element(1,1) >= 0.9999 &&
         dings.element(2,2) >= 0.9999 &&
         fabs(dings.element(0,3)) <= 0.00001 &&
         fabs(dings.element(1,3)) <= 0.00001 &&
         fabs(dings.element(2,3)) <= 0.00001)
      {
        printf("Identity transform, no sense to calc search spaces.\n");
        cov = false;
      }
      else
      {
        m = pose->GetCovarianceMatrix();
        double d = m.trace();
        if (d == 0.0)
        {
          printf("RegionOI: Ignoring covariances with trace = 0\n");
          throw "RegionOI: Ignoring covariances with trace = 0\n";
        }
        else
        {
          m = pose->GetCovarianceMatrix();
        }
       }
      }
      else
        cov = false;
   }
   catch(...)
   {
     cov = false;
   }
   if(cov == true)
   {
     try
     {
       RelPoseHTuple::GetPose(pose, &pose_s, cam_pose_id);
       ExtremePoses = GetExtremePoses(m);
       extents=GetExtents(ExtremePoses.t(), pose_s, gravPoint, CamPar, this);
       printf("Assigned new region of size: %d\n", GetSize());
     }
     catch(...)
     {
       printf("Error in RegionOI!\n");

     }
   }

}


RegionOI::RegionOI(SegmentPrototype* proto, LocatedObjectID_t cam_pose_id, Calibration* calib)
{

  sensor_msgs::PointCloud pcd_in = proto->GetPointCloud(proto->GetLastMatchedPose()->m_uniqueID);
  sensor_msgs::PointCloud pcd_trans = cloud_trans(proto->GetLastMatchedPose()->m_uniqueID, cam_pose_id, pcd_in);

  Halcon::HTuple X,  Y,  Z;
  int  count = 0;
  for(size_t i = 0; i < (pcd_trans.points.size()); i++)
  {
    if(pcd_trans.points[i].z > 0)
    {
      X[count] = pcd_trans.points[i].x;
      Y[count] = pcd_trans.points[i].y;
      Z[count] = pcd_trans.points[i].z;
      count++;
    }
    else{
      printf("Point %ld (%f %f %f) is not projectable\n", i, pcd_trans.points[i].x, pcd_trans.points[i].y, pcd_trans.points[i].z);
    }


  }
  HTuple row, column;
  try
  {
    project_3d_point(X, Y, Z, calib->CamParam(), &row, &column);
    gen_region_points(&m_reg, row, column);
    Halcon::shape_trans(m_reg, &m_reg, "convex");
    Halcon::HTuple r_s(5, 1);
    Halcon::HTuple c_s(5,1);
    r_s[0] = 0;
    r_s[4] = 2;
    c_s[1] = 0;
    c_s[3] = 2;
    Halcon::shape_trans(m_reg, &m_reg, "convex");
    Hobject reg_struct_cross;
    Halcon::gen_region_points(&reg_struct_cross, r_s, c_s);
    Halcon::minkowski_add1(m_reg, reg_struct_cross, &m_reg, 30);
  }
  catch(...)
  {
    printf("Error: TODO! do something\n");
  }
  printf("Assigned new region of size: %d\n", GetSize());
}

RegionOI::RegionOI(std::string stFilename)
{

  Halcon::read_region(&m_reg,stFilename.c_str());

}

int RegionOI::GetSize()
{

  HTuple area, row, column;
  Halcon::area_center(m_reg, &area, &row, &column);
  return area[0].I();

}

void RegionOI::AddPoint(double Row, double Column)
{

  Halcon::HTuple r, c;
  Halcon::get_region_points(m_reg, &r, &c);
  tuple_concat(r, Row, &r);
  tuple_concat(c, Column, &c);
  Halcon::gen_region_points(&m_reg, r, c);

}

void RegionOI::TransitiveHull(int width)
{

  Halcon::Hobject reg_struct_cross;
  Halcon::HTuple r_s(5, 1);
  Halcon::HTuple c_s(5,1);
  r_s[0] = 0;
  r_s[4] = 2;
  c_s[1] = 0;
  c_s[3] = 2;
  Halcon::shape_trans(m_reg, &m_reg, "convex");
  Halcon::gen_region_points(&reg_struct_cross, r_s, c_s);

  Halcon::minkowski_add1(m_reg, reg_struct_cross, &m_reg, width / 25);

  Halcon::HTuple area,r,c;
  Halcon::area_center(m_reg, &area, &r, &c);
  printf("Area of a new Region: %d\n", area[0].I());
  if(area < 500)
     Halcon::dilation_circle(m_reg, &m_reg, width / 16);

}

static unsigned int s_regionCounter = 0;

XMLTag* RegionOI::Save(std::string tagName)
{
 XMLTag* ret = NULL;
  std::ostringstream stFileNameBuf;
  stFileNameBuf << "Region" << s_regionCounter++ << "_" << (unsigned long)time(NULL) << ".dat";
  std::string stFileName(stFileNameBuf.str());

  try
  {
    Halcon::write_region(m_reg, stFileName.c_str());
  }
  catch(Halcon::HException ex)
  {
    printf("Error writing Region OI to %s: %s\n", stFileName.c_str(), ex.message);
    throw "Tried saving of a bad region";
  }
  ret = new XMLTag(tagName);
  ret ->AddProperty("Filename", stFileName);
/*  Halcon::HTuple r, cs, ce;
  Halcon::get_region_runs(m_reg, &r, &cs, &ce);
  RegionRuns row_colStart_colEnd;
  for (int i = 0;i < r.Num(); i++)
  {
    std::pair< int, std::pair<int, int > >  r_cs_ce;
    r_cs_ce.first = r[i].I();
    r_cs_ce.second.first = cs[i].I();
    r_cs_ce.second.second = ce[i].I();
    row_colStart_colEnd.push_back(r_cs_ce);
  }
  ret = XMLTag::Tag(row_colStart_colEnd, (tagName.length() > 0) ? tagName : XML_NODE_ROI);*/

  return ret;
}

Halcon::Hobject& RegionOI::GetRegion(double scale)
{
 if(scale == 1.0)
   return m_reg;
  else
  {
    try
    {
      Halcon::zoom_region(m_reg, &m_regZoomTmp, scale, scale);
    }
    catch(Halcon::HException ex)
    {
      printf("Error in RegionOI Get Region: %s \n", ex.message);
      return m_reg;
    }
    return m_regZoomTmp;
  }
}

