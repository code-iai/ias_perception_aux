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

#include "RFASurfaceByCov.h"
#include "SurfaceDetection.h"
#include "Camera.h"
#include "XMLTag.h"

#include "RangeSensor.h"

#include <cpp/HalconCpp.h>

using namespace cop;



RFASurfaceByCov::RFASurfaceByCov()
{
}

void RFASurfaceByCov::SetData(XMLTag* tag)
{
  printf("Loading Algorithm RFASurfaceByCov\n");
}

RFASurfaceByCov::~RFASurfaceByCov(void)
{
}

using namespace Halcon;

void PCDToXYZImageWithRestrict(Hobject *ImageX, Hobject *ImageY, Hobject *ImageZ, Hobject *Region,sensor_msgs::PointCloud& pcd, std::vector<double> camparams, 
                               double minx, double maxx, double miny, double maxy, double  minz, double maxz,
                               double meanx, double meany, double meanz)
{
  Hobject  Region1, RegionDifference1, RegionClosing, ConnectedRegions;
  Hobject  SelectedRegions, ObjectSelected;
  // Local control variables
  HTuple  Qxt, Qyt, Qzt,Qx, Qy, Qz, Row, Column, AbsoluteHisto, SizeCluster;
  HTuple  RelativeHisto, Min1, Max1, Range, Function, SmoothedFunction;
  HTuple  Min, Max, Y1, Index, i, thres, start, Or, Number;
  HTuple  index, Rows, Columns, Xreg, Yreg, Zreg, MeanX, MeanY;
  HTuple  MeanZ, Deviation, DeviationY, DeviationX;
  HTuple  Row_add, Row_sgn, Index_R, Col_add, Col_sgn, Index_C;

   Halcon::HTuple CamParam(8);
  for(size_t i = 0; i < camparams.size(); i++)
  {
    CamParam[i] = camparams[i];
  }

  Halcon::HTuple X,  Y,  Z;
  int counter = 0;
  for(size_t i = 0; i < (pcd.points.size()); i++)
  {
    if(pcd.points[i].x == pcd.points[i].x && 
        pcd.points[i].x > minx && pcd.points[i].x < maxx &&
        pcd.points[i].y > miny && pcd.points[i].y < maxy &&
        pcd.points[i].z > minz && pcd.points[i].z < maxz  )
    {
      X[counter] = pcd.points[i].x;
      Y[counter] = pcd.points[i].y;
      Z[counter] = pcd.points[i].z;
      counter++;
    }
  }
  //printf("CamParam: %f %f %f %f %f %f\n", CamParam[0].D(),CamParam[1].D(),CamParam[2].D(),CamParam[3].D(),CamParam[4].D(),CamParam[5].D());
  try
  {
    HTuple zeros;
    tuple_find(Z, HTuple(0.0), &zeros);
    if(!(zeros.Num() == 1 && zeros[0].I() == -1))
    {
      printf("pcd contians %ld points == 0\n", zeros.Num());
      for(int i = 0; i  < zeros.Num(); i++)
      {
        Z[zeros[i].I()] = 0.0000001;
      }
    }
    project_3d_point(X, Y, Z, CamParam, &Row, &Column);
  }
  catch(HException ex)
  {
    printf("Fix failed: %s but s_x= %f s_y =%f\n", ex.message, CamParam[2].D(), CamParam[3].D()  );
  }

  gen_image_const(ImageZ, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);
  gen_image_const(ImageX, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);
  gen_image_const(ImageY, "float", CamParam[6].I() + 1, CamParam[7].I() + 1);
  //clear tuple from Rows and Columns that are too big or below 0
  tuple_add(Row, 1, &Row_add);				//all rows are at least 0 + 1
  tuple_sgn(Row_add, &Row_sgn); 			//all rows > 0 become 1
  tuple_find(Row_sgn, 1, &Index_R); 		//find all rows > 0
  tuple_select(Row, Index_R, &Row); 		//select all rows > 0
  tuple_select(Column, Index_R, &Column);
  tuple_select(X, Index_R, &X);
  tuple_select(Y, Index_R, &Y);
  tuple_select(Z, Index_R, &Z);

  tuple_add(Column, 1, &Col_add); 			//all cols are at least 0 + 1
  tuple_sgn(Col_add, &Col_sgn); 			//all cols > 0 become 1
  tuple_find(Col_sgn, 1, &Index_C);			//find all cols > 0
  tuple_select(Column, Index_C, &Column);	//select all cols > 0
  tuple_select(Row, Index_C, &Row);
  tuple_select(X, Index_C, &X);
  tuple_select(Y, Index_C, &Y);
  tuple_select(Z, Index_C, &Z);

  tuple_sub(Row, CamParam[7].I(), &Row_add);	//all rows from 0-143 are < 0
  tuple_sgn(Row_add, &Row_sgn);					//all rows >= 0 <= 143 become -1
  tuple_find(Row_sgn, -1, &Index_R);			//select all rows > 0
  if(Index_R.Num() == 1 && Index_R[0].I() == -1)
  {
    printf("No values inside the image (row)\n");
  }
  else
  {
    tuple_select(Row, Index_R, &Row);				//select all rows >= 0 <= 143
    tuple_select(Column, Index_R, &Column);
    tuple_select(X, Index_R, &X);
    tuple_select(Y, Index_R, &Y);
    tuple_select(Z, Index_R, &Z);
  }
  tuple_sub(Column, CamParam[6].I(), &Col_add);	//all cols from 0-175 are < 0
  tuple_sgn(Col_add, &Col_sgn);						//all cols >= 0 <= 175 become -1
  tuple_find(Col_sgn, -1, &Index_C);				//select all cols > 0

  if(Index_C.Num() == 1 && Index_C[0].I() == -1)
  {
    printf("No values inside the image (cols)\n");
  }
  else
  {
    tuple_select(Column, Index_C, &Column);			//select all cols >= 0 <= 175
    tuple_select(Row, Index_C, &Row);
    tuple_select(X, Index_C, &X );
    tuple_select(Y, Index_C, &Y );
    tuple_select(Z, Index_C, &Z);
  }


  HTuple rowcollect , colcollect;
  try
  {

    set_grayval(*ImageX, Row.Int(), Column.Int(), X - meanx);
    set_grayval(*ImageY, Row.Int(), Column.Int(), Y - meany);
    //
    set_grayval(*ImageZ, Row.Int(), Column.Int(), Z - meanz);
    gen_region_points(Region, Row.Int(), Column.Int());
  }
  catch(HException ex)
  {
    printf("got an exception trying directly %s\n", ex.message);
    for(int i = 0; i < Row.Num(); i++)
    {
      if(Row[i].I() >= 0 && Column[i].I() >= 0 &&
         Row[i].I() < CamParam[7].I() + 1 && Column[i].I() < CamParam[6].I() + 1)
      {
          set_grayval(*ImageX, Row[i].I(), Column[i].I(), X[i].D() - meanx);
          set_grayval(*ImageY, Row[i].I(), Column[i].I(), Y[i].D() - meany);
          set_grayval(*ImageZ, Row[i].I(), Column[i].I(), Z[i].D() - meanz);
          tuple_concat(rowcollect, Row[i], &rowcollect);
          tuple_concat(colcollect, Column[i], &colcollect);
      }
      else
        printf("Error at: r %d c %d \n x %f y %f  z  %f\n Cam: %d %d (%ld)\n", Row[i].I(), Column[i].I(), X[i].D(), Y[i].D(), Z[i].D(), CamParam[6].I(),CamParam[7].I(), CamParam.Num());
    }
    gen_region_points(Region, rowcollect.Int(), colcollect.Int());
  }


}



Descriptor* RFASurfaceByCov::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  SurfaceModel *surface_model = NULL;
  std::vector<double> camparams;
  if(pose != NULL)
  {
    for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
    {
      if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
      {
        printf("Starting to learn a surface model\n");
          try
          {
            Hobject imgx, imgy, imgz, region;
            HTuple ObjectModel3DModel;
            SwissRangerReading* reading =  ((SwissRangerReading*)((RangeSensor*)(*it))->GetReading());
            sensor_msgs::PointCloud &pcd = reading->m_image;
            
            camparams = (*it)->GetUnformatedCalibrationValues().second;
            
            /* TODO Check, pcd_in can not be projected*/
            Matrix m = pose->GetMatrix(reading->GetPose()->m_uniqueID);
            Matrix cov = pose->GetCovarianceMatrix(reading->GetPose()->m_uniqueID);
            PCDToXYZImageWithRestrict(&imgx, &imgy, &imgz, &region, pcd, camparams, m.element(0,3) - cov.element(0,0) * 2,
                                    m.element(0,3) + cov.element(0,0) * 2,
                                    m.element(1,3) - cov.element(1,1) * 2,
                                    m.element(1,3) + cov.element(1,1) * 2,
                                    m.element(2,3) - cov.element(2,2) * 2,
                                    m.element(2,3) + cov.element(2,2) * 2, 
                                    m.element(0,3),  m.element(1,3), m.element(2,3));
            HTuple a, r, c;
            area_center(region, &a, &r, &c);
            printf("Num of points inside: %d\n", a[0].I());
            reduce_domain(imgx, region, &imgx);
            reduce_domain(imgy, region, &imgy);
            reduce_domain(imgz, region, &imgz);

  
            surface_model = new SurfaceModel();
            surface_model->SetClass((Class*)(sig.GetClass(0)));
            surface_model->filename = "SurfaceTrainedByJlo_" + surface_model->GetClass()->GetName() +".sfm";
       
            Halcon::HTuple OM3DID, Status, SMID, empty, pn, pv;
      
            xyz_to_object_model_3d (imgx, imgy, imgz, &ObjectModel3DModel);

            Halcon::HTuple Pose, Score, SurfaceMatchingResultID;
            
            Halcon::create_surface_model (ObjectModel3DModel, 0.03, empty, empty, &SMID);
            find_surface_model(SMID, ObjectModel3DModel , 0.03, 0.05, 0.15, "false",
                                  "num_matches", 1, &Pose, &Score, &SurfaceMatchingResultID);
                                if(Score.Num() == 1)
            {
              Halcon::write_surface_model(SMID,  surface_model->filename.c_str());
              surface_model->m_surface_handle = SMID[0].L();
              surface_model->m_hommat = IdentityMatrix(4);
              surface_model->m_approxCov = cov;
              surface_model->Evaluate(1.0, 100.0);
            }
            else
            {
              printf("No match found with the new model on the training data\n");
              delete surface_model;
              surface_model = NULL;
            }
            break;
          }
          catch(Halcon::HException ex)
          {
            ROS_ERROR("Error reading object model: %s\n", ex.message);
            throw "Error in RFASurfaceByCov::Perform";
          }
        } 
      }
  }
  return surface_model;

}

double RFASurfaceByCov::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sens)
{
  if(sig.GetElement(0,DESCRIPTOR_SURFACE) == NULL && sig.GetElement(0, DESCRIPTOR_SEGMPROTO) == NULL )
  {
      return 1.0;
  }
  else
    return -0.0;
}

XMLTag* RFASurfaceByCov::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  return tag;
}


