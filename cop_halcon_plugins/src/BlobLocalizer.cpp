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


#include "BlobLocalizer.h"
#include "Blob.h"
#include "XMLTag.h"
#include "RelPoseHTuple.h"

#include "RelPoseHTuple.h"

#include <cpp/HalconCpp.h>

using namespace cop;



BlobLocalizer::BlobLocalizer()
{

}

BlobLocalizer::BlobLocalizer(XMLTag* /*tag*/)
{

}

BlobLocalizer::~BlobLocalizer(void)
{
}


bool LocalizeBlob(Halcon::Hobject Image, Blob* blob, Calibration* calib, RelPose* poseIn, Halcon::HTuple* Pose, Halcon::HTuple* Quality)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  Image1, Image2, Image3, Red, Green, Blue;
  Hobject  RegionIntersection, RegionIntersection2, RegionUnion;
  Hobject  FinalRegion, Contours, Rectangle, ImageReduced;
  Hobject  Regions, RegionComplement, ConnectedRegions, Cross;


  // Local control variables
  HTuple  MinAreaOut, MinRoundnessOut, Number, Area, MaxAreaOut, CamParam, PoseIn;
  HTuple  Row, Column, Row3, Column3, Phi, Length1, Length2;
  HTuple  Axis1_x, Axis1_y, Axis2_x, Axis2_y, rt, ct, max_area = 0;
  RelPoseHTuple::GetPose(poseIn, &PoseIn);
  CamParam = calib->CamParam();
  MinAreaOut = blob->MinArea;
  MaxAreaOut = blob->MaxArea;
  MinRoundnessOut = blob->MinRoundness;
  decompose3(Image, &Image1, &Image2, &Image3);
  trans_to_rgb(Image1, Image2, Image3, &Image1, &Image2, &Image3, "yuv");
  threshold(Image1, &Red, blob->RedThresMin, blob->RedThresMax);
  threshold(Image2, &Green, blob->GreenThresMin, blob->GreenThresMax);
  threshold(Image3, &Blue, blob->BlueThresMin, blob->BlueThresMax);
  //*
  intersection(Red, Green, &RegionIntersection);
  intersection(Red, Blue, &RegionIntersection2);
  union2(RegionIntersection, RegionIntersection2, &RegionUnion);
  connection(RegionUnion, &FinalRegion);
  closing_circle(FinalRegion, &FinalRegion, 6);
  count_obj(FinalRegion, &Number);
  HTuple a,b,c,RoundNess;
  if(Number == 0)
    return false;
  roundness(FinalRegion, &a,&b,&RoundNess,&c);
  count_obj(FinalRegion, &Number);
  if(Number == 0)
    return false;

  area_center(FinalRegion, &Area, &Row, &Column);
  for(int i = 0; i < Number; i++)
  {
    if(max_area < Area[i].I())
    {
      max_area = Area[i].I();
      rt = Row[i].D();
      ct = Column[i].D();
    }
  }
  /*for(int i = 0; i < RoundNess.Num(); i++)*/
  /*printf("1 Region %d: roundness %f area %d (%f, %f)\n", i, RoundNess[i].D(), Area[i].I(), Row[i].D(), Column[i].D());*/
  while (Number>1)
  {
    select_shape(FinalRegion, &FinalRegion, HTuple("area").Concat("roundness"), "and",
        MinAreaOut.Concat(MinRoundnessOut), MaxAreaOut.Concat(1.0));
    count_obj(FinalRegion, &Number);
    /*printf("2 Number of objects in BlobLocalizer: %d\n", Number[0].I());*/
    if(Number[0].I() == 1)
      break;
    MinAreaOut += blob->MinArea / 3;
    MinRoundnessOut += 0.05;
    roundness(FinalRegion, &a,&b,&RoundNess,&c);
    area_center(FinalRegion, &Area, &Row, &Column);
    /*for(int i = 0; i < RoundNess.Num(); i++)*/
    /*printf("2 Region %d: roundness %f area %d (%f, %f)\n", i, RoundNess[i].D(), Area[i].I(), Row[i].D(), Column[i].D());*/
  }
  count_obj(FinalRegion, &Number);
  area_center(FinalRegion, &Area, &Row, &Column);
  if(Number != 1)
  {
    Row = rt;
    Column = ct;
  }
  try
  {
      HTuple X, Y, HomMat3D, HomMat3D1, HomMat3DCompose;
      image_points_to_world_plane(CamParam, PoseIn, Row, Column, "m", &X, &Y);
      pose_to_hom_mat3d (X.Concat(Y).Concat(0).Concat(0).Concat(0).Concat(0).Concat(0), &HomMat3D);
      pose_to_hom_mat3d (PoseIn,&HomMat3D1);
      hom_mat3d_compose (HomMat3D1, HomMat3D, &HomMat3DCompose);
      hom_mat3d_to_pose (HomMat3DCompose, Pose);
      Quality[0] = 0.99;
  }
  catch(Halcon::HException ex)
  {
      printf("Error in BlobLocalizer::Inner: %s", ex.message);
      throw "Error Localizing Blobs\n";
  }
  return true;
}

XMLTag* BlobLocalizer::Save()
{
  return new XMLTag(XML_NODE_BLOBLOCALIZER);
}
std::vector<RelPose*> BlobLocalizer::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Calibration* calib = &(cam->m_calibration);
    Image* img = cam->GetImage(-1);
     RelPose* camPose = img->GetPose();
        if(img != NULL && camPose != NULL)
        result = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
  }
  
  return result;
}

double BlobLocalizer::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  if(object.GetElement(0,DESCRIPTOR_BLOB) != NULL)
    return 1.0;
  else
    return 0.0;
}
std::vector<RelPose*> BlobLocalizer::Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  Blob* blob = (Blob*)object.GetElement(0,DESCRIPTOR_BLOB);
  try
  {
    Halcon::HTuple FinalPose, Quality;
    try
    {
      if (!LocalizeBlob(*img->GetHImage(), blob, calib, lastKnownPose,&FinalPose, &Quality))
      {
          numOfObjects = 0;
          qualityMeasure = 0.0;
         return result;
      }
    }
    catch(Halcon::HException ex)
    {
      printf("Error in BlobLocalizer::Inner: %s", ex.message);
      throw "Error Localizing Blobs\n";
    }
    Halcon::HTuple cov(6, (double)(1 - Quality[0].D()));
    cov[2] = 1 - Quality[0].D();
    cov[3] = 1 - Quality[0].D();
    cov[4] = 1 - Quality[0].D();
    cov[5] = 1 - Quality[0].D();
    qualityMeasure = Quality[0].D();

    if(object.GetObjectPose() != NULL && object.GetObjectPose()->m_uniqueID != ID_WORLD)
    {
        result.push_back(RelPoseHTuple::FRelPose(FinalPose, cov, camPose, object.GetObjectPose()->m_uniqueID));
        cerr<<"In const. 4"<<endl;
    }
    else
    {
        result.push_back(RelPoseHTuple::FRelPose(FinalPose, cov,  camPose));
        cerr<<"In const. 3"<<endl;
    }
    numOfObjects = 1;
    img->Free();
  }
  catch(...)
  {
    numOfObjects = 0;
    qualityMeasure = 0.0;
  }
  return result;

}
