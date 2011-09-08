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
#include "SearchParams3d.h"
//#include "cpp/HObjectModel3D.h"
//#include "cpp/HalconCpp.h"
using namespace cop;


using namespace Halcon;
using namespace std;

ReturnMatrix cop::GetExtremePoses(const Matrix& cov)
{
  Matrix m(6, 12);

  try
  {

  int i = cov.nrows() * cov.ncols() ;
  if(i != 0 && !cov.is_zero())
  {
    memset((Real*)m.Store(), 0, sizeof(Real) * 72);
    DiagonalMatrix eigenValues;
    Matrix eigenVectors, V;
            //Decompose Covariance
    SVD(cov, eigenValues, eigenVectors, V);
    ColumnVector ExMax(6), ExMin(6), Diff(6);
    Real EigVal;
    for(int i = 1; i <= 6; i++)
    {
      Diff = eigenVectors.column(i);
      EigVal = eigenValues.element(i-1);
      for (int j = 0; j<6; j++)
      {
          Diff.element(j)*=EigVal;
      }
      ExMax = Diff; // transposed i-th eigenvector
      ExMin = Diff*(-1);
      m.column(2*i-1)=ExMax;
      m.column(2*i)=ExMin;
    }
    m.release();
    return m;
  }
  }catch(...)
  {
   printf("Big problem in Extreme Poses\n");
  }
  return m;
}


void cop::GetVPFromPose(const HTuple& Pose, HTuple *ViewPoint, double *ObjGravPoint)
{
  HTuple Hom, HomTrans,invHomTrans, camPose;
  try
  {
    pose_to_hom_mat3d(Pose, &Hom); //represent pose as hom matrix

    hom_mat3d_translate_local(Hom,ObjGravPoint[0],ObjGravPoint[1],ObjGravPoint[2], &HomTrans);
    hom_mat3d_invert(HomTrans, &invHomTrans);
    hom_mat3d_to_pose(invHomTrans, &camPose); // convert back to pose to get clean x,y,z coords of "camera"
  }
  catch(HException ex)
  {
    printf("Halcon Error in GetVPFromPose: %s\n", ex.message);
    throw "Error estimating pose range";
  }
  catch(...)
  {
    printf("Unknown  Error\n");
    throw "Error estimating pose range";
  }
    HTuple Equat,Merid,Lon,Lat,Rad;
        Equat = "-y";
  Merid = "-z";
  convert_point_3d_cart_to_spher(camPose[0],camPose[1],camPose[2],Equat, Merid, &Lon, &Lat, &Rad);
        tuple_concat((const HTuple)Lon, (const HTuple)Lat, ViewPoint);
  tuple_concat((const HTuple)*ViewPoint, (const HTuple)Rad, ViewPoint);
}

HTuple cop::GetExtents(const Matrix& ExtremePosesTransp, const HTuple& MeanPose, double* gravPoint, Calibration* calib, RegionOI* r)
{
  return GetExtents(ExtremePosesTransp, MeanPose, gravPoint, calib->CamParam(), r);
}

HTuple cop::GetExtents(const Matrix& ExtremePosesTransp, const HTuple& MeanPose, double* gravPoint, const HTuple&  CamParam, RegionOI* r)
{
  HTuple ViewPoint(3,0.0), SSpaceMax(3,0.0), SSpaceMin(3,0.0), Contrib(3,0.0), MeanVPoint(3,0.0), hommat, hommatExtreme;
  HTuple Row, Col;
  HTuple Pose(7,0.0);
  GetVPFromPose(MeanPose, &MeanVPoint, gravPoint);
  pose_to_hom_mat3d(MeanPose, &hommat);

#ifdef _DEBUGd
 /* cerr<< "ExtremePosesTrasp: "<<endl<<ExtremePosesTransp<<endl;*/
#endif
  Halcon::set_system("clip_region", "false");
  for(int vp=0; vp<12;vp++)
  {
      for (int p=0;p<6;p++)
      {
          Pose[p] = ExtremePosesTransp.element(vp, p);
      }
      pose_to_hom_mat3d(Pose, &hommatExtreme);
      hom_mat3d_compose(hommat, hommatExtreme, &hommatExtreme);
      hom_mat3d_to_pose(hommatExtreme, &Pose);

      try
      {
        project_3d_point(Pose[0].D(),Pose[1].D(),Pose[2].D(),CamParam, &Row, &Col);
        r->AddPoint(Row[0].D(), Col[0].D());
        GetVPFromPose(Pose, &ViewPoint, gravPoint);
        Contrib=ViewPoint-MeanVPoint;

                  for(int i = 0; i < 3; i++){
                          if (i!=2){
                          if(fabs(Contrib[i].D()) > M_PI)
                          Contrib[i]=-(2*M_PI - fabs(Contrib[i].D()))* ((Contrib[i].D())>0?1:-1);
                          }
            if(SSpaceMax[i] <= Contrib[i])
                                  SSpaceMax[i] =  Contrib[i].D();// > M_PI ? M_PI : Contrib[i].D();
            if(SSpaceMin[i] >= Contrib[i])
                                  SSpaceMin[i]=Contrib [i].D();// < -M_PI ? -M_PI : Contrib[i].D();
        }
      }
      catch(Halcon::HException ex)
      {
        printf("Error in GetExtents: %s\n", ex.message);
      }

  }
  r->TransitiveHull(CamParam[6].I());
  tuple_concat(SSpaceMax, SSpaceMin, &SSpaceMax);
  return SSpaceMax;
}


