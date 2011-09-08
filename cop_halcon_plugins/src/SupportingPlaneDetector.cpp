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


#include "SupportingPlaneDetector.h"
#include "SupportingPlaneDescriptor.h"
#include "FindCalTab.h"
#include "XMLTag.h"

#ifdef SWISS_RANGER_SERVICE
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
/*Tabletop messages*/
#include <tabletop_msgs/ObjectOnTable.h>
#endif
#include "ClusterDetector.h"
using namespace cop;


#ifdef SWISS_RANGER_SERVICE
class PlaneClusterResult
{
  public:
  double a;
  double b;
  double c;
  double d;
  geometry_msgs::Point32 pcenter;
  std::vector<tabletop_msgs::ObjectOnTable> oclusters;
};
#endif

SupportingPlaneDetector::SupportingPlaneDetector(void)
{
}

SupportingPlaneDetector::~SupportingPlaneDetector(void)
{
}

#ifdef SWISS_RANGER_SERVICE

inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}



/*
float64 a
float64 b
float64 c
float64 d
robot_msgs/Point32 pcenter
robot_msgs/ObjectOnTable[] oclusters
*/
bool GetPlane(RelPose* rel, RelPose*& pose_plane)
{
  int m_swissranger_jlo_id = 21;
  int m_ptu_jlo_id = 14;
  PlaneClusterResult response;

  if (!ClusterDetector::CallStaticPlaneClusterExtractor(&response, m_swissranger_jlo_id, m_ptu_jlo_id))
  {
    printf("Error in Supporting plane!\n");
    return false;
  }
  double a = response.a;
  double b = response.b;
  double c = response.c;
//  double s = response.d;

  geometry_msgs::Point32 &pcenter = response.pcenter;
  std::vector<tabletop_msgs::ObjectOnTable> &vec = response.oclusters;
   /*Norm v1*/
  normalize(a,b,c);

  /*Init v2*/
  double d,e,f,g,h,j;
  if (a == b && a == c)
  {
     d = 1; e = 0; f = 0;
  }
  else
  {
    d = b; e = a; f = c;
  }
  /*Orthogonalize v2*/
  double tmp = scalarproduct(a,b,c,d,e,f);
  d = d - tmp * a;
  e = e - tmp * b;
  f = f - tmp * c;

  /*Norma v2*/
  normalize(d,e,f);

  /*Create v3*/

  CrossProduct_l(a,b,c,d,e,f, g, h,j);
  /**  Build Matrix:
  *   d g a p.x
  *   e h b p.y
  *   f i c p.z
  *   0 0 0 1
  *   for every cluster
  */
    printf("No Clusters found, adding a meaningless cluster");
   tabletop_msgs::ObjectOnTable on;
   on.center.x = pcenter.x;
   on.center.y = pcenter.y;
   on.center.z = pcenter.z;

   on.min_bound.x = -0.5;
   on.min_bound.y = -0.3;
   on.min_bound.z = pcenter.z - 0.20;

   on.max_bound.x = 0.5;
   on.max_bound.y = 0.3;
   on.max_bound.z = pcenter.z + 0.20;
   vec.clear();
   vec.push_back(on);
  for(unsigned int i = 0; i < vec.size(); i++)
  {
    printf("a: %f , b: %f , c: %f\n", a, b, c);
    const geometry_msgs::Point32 &center = vec[i].center;
    const geometry_msgs::Point32 &min_bound = vec[i].min_bound;
    const geometry_msgs::Point32 &max_bound = vec[i].max_bound;
    Matrix rotmat;
    rotmat << d << g << a << center.x
           << e << h << b << center.y
           << f << j << c << center.z
           << 0 << 0 << 0 << 1;

    Matrix cov;
    double covx = max(fabs(center.x - max_bound.x), fabs(center.x - min_bound.x)) ;
    double covy = max(fabs(center.y - max_bound.y), fabs(center.y - min_bound.y)) ;
    double covz = max(fabs(center.z - max_bound.z), fabs(center.z - min_bound.z)) ;
    /*Fill covariance with the cluster size and hardcoded full rotation in normal direction */
     cov << covx <<0    << 0    << 0   << 0   << 0
     <<   0    << covy << 0    << 0   << 0   << 0
     <<0    << 0    << covz << 0   << 0   << 0
     <<0    << 0    << 0    << 0.1 << 0   << 0
     <<0    << 0    << 0    << 0   << 0.3 << 0
     <<0    << 0    << 0    << 0   << 0   << 0.3;


     pose_plane = RelPoseFactory::FRelPose(rel, rotmat, cov);
  }
  return true;
}
#endif /*SWISS_RANGER_SERVICE*/

std::vector<RelPose*> SupportingPlaneDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
    std::vector<RelPose*> results;
    Camera* cam = Camera::GetFirstCamera(sensors);
    Image* img = cam->GetImage(-1);
    Calibration* calib = &cam->m_calibration;
    results = Inner(img, cam->m_relPose, calib, pose, object, numOfObjects, qualityMeasure, 0);
    /*TODO plane clusters*/
    return results;
}

std::vector<RelPose*> SupportingPlaneDetector::Inner(Image* img, RelPose* campose, Calibration* calib, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure, int index)
{
  std::vector<RelPose*> results;
  printf("Endtering SupportingPlaneDetector::Inner\n");
  try
  {
  SupportingPlaneDescriptor* plane = (SupportingPlaneDescriptor*)object.GetElement(0, DESCRIPTOR_PLANE);
  if(plane != NULL)
  {
    Elem* elem = plane->GetMarker();
    if(elem == NULL)
    {
      #ifndef SWISS_RANGER_SERVICE
      printf("SupportingPlaneDetector: No marker available\n");
      #else
      RelPose* pose_plane;
      if(GetPlane(campose,pose_plane))
        results.push_back(pose_plane);
      #endif
    }
    else if(elem->GetType() == DESCRIPTOR_CALTAB)
    {
      CalTab* cl = (CalTab*)elem;
      FindCalTab fc;
      printf("SupportingPlaneDetector: Entering Inner of findCaltab\n");
      results = fc.Inner(img , campose, calib, pose, cl, numOfObjects, qualityMeasure);
      printf("SupportingPlaneDetector: Exiting Inner of findCaltab\n");
    }
  }
  else
  {
      #ifndef SWISS_RANGER_SERVICE
      printf("SupportingPlaneDetector: No marker available\n");
      #else
      RelPose* pose_plane;
      if(GetPlane(campose,pose_plane))
        results.push_back(pose_plane);
      #endif
  }
  }
  catch(...)
  {
    printf("Error occured\n");
  }
  return results;
}
double SupportingPlaneDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
    SupportingPlaneDescriptor* plane = (SupportingPlaneDescriptor*)object.GetElement(0, DESCRIPTOR_PLANE);
    if(plane != NULL)
        return 1.0;
    else
    {
      return 0.0;
  }
}

bool SupportingPlaneDetector::TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose)
{
    return false;
}

XMLTag* SupportingPlaneDetector::Save()
{
    return new XMLTag(XML_NODE_SUPPORTINGPLANEDETECTOR);
}
