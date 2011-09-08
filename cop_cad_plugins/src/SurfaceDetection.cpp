
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

#include "SurfaceDetection.h"



#include "XMLTag.h"
#include "RangeSensor.h"
#include "BoostUtils.h"

#include "RelPoseHTuple.h"
#include "SegmentPrototype.h"
#include "RegionOI.h"
#include "Camera.h"
#include "ShapeModel.h"
#include <sys/time.h>




#define XML_NODE_OBJECTMEAN  "Hommat"
#define XML_NODE_OBJECTCOV   "Cov"
#define XML_ATTRIBUTE_MINSCORE "MinScore"

using namespace cop;

#include <cpp/HalconCpp.h>

using namespace Halcon;

void extract_table_height (Halcon::Hobject &Region,  Halcon::Hobject &ImageZ, Halcon::HTuple *table_height);

SurfaceDetection::SurfaceDetection()
{
}

void SurfaceDetection::SetData(XMLTag* tag)
{
  printf("Loading Algorithm SurfaceDetection\n");
/*  if(tag != NULL)
  {
    m_camfilename = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");
    if(m_camfilename.length() > 0)
    {
      Halcon::HTuple camparam_h;
      Halcon::read_cam_par(m_camfilename.c_str(), &camparam_h);
      for(int i = 0; i < camparam_h.Num(); i++)
      {
        m_camparams.push_back(camparam_h[i].D());
      }
    }
  }*/
    //printf("Read camparam: %s\n", camparam.c_str());
 }


SurfaceModelExtraction::SurfaceModelExtraction()
{
}

SurfaceModelExtraction::~SurfaceModelExtraction()
{
}


void SurfaceModelExtraction::SetData(XMLTag* tag)
{
  printf("Loading Algorithm SurfaceModelExtraction\n");
  /*if(tag != NULL)
  {
    m_camfilename = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");

    if(m_camfilename.length() > 0)
    {
      Halcon::HTuple camparam_h;
      Halcon::read_cam_par(m_camfilename.c_str(), &camparam_h);
      for(int i = 0; i < camparam_h.Num(); i++)
      {
        m_camparams.push_back(camparam_h[i].D());
      }
    }
  }*/
    //printf("Read camparam: %s\n", camparam.c_str());
 }


void SurfaceModel::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  if(tag != NULL)
  {
      filename = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");
      if(filename.length() > 0)
      {
        HTuple surface_handle;
        try
        {
          read_surface_model(filename.c_str(), &surface_handle);
          m_surface_handle = surface_handle[0].I();
        }
        catch(HException)
        {
          ROS_ERROR("Can not load surface model\n");
          throw "Can not load surface model";
        }
      }
      else
      {
        ROS_ERROR("No surface model specified\n");
          throw "No surface model specified";
      }
      m_minscore = tag->GetPropertyDouble(XML_ATTRIBUTE_MINSCORE, 0.3);

      XMLTag* hommat_tag = tag->GetChild(XML_NODE_OBJECTMEAN);
      if(hommat_tag != NULL)
      {
        m_hommat = XMLTag::Load(hommat_tag, &m_hommat);
      }
      else
      {
        m_hommat = IdentityMatrix(4);
      }
      XMLTag* cov_tag = tag->GetChild(XML_NODE_OBJECTCOV);
      if(cov_tag != NULL)
      {
        m_approxCov = XMLTag::Load(cov_tag, &m_approxCov);
      }
      else
      {
        m_approxCov = IdentityMatrix(6);
        m_approxCov.element(0,0) = 0.01; m_approxCov.element(1,1) = 0.01; m_approxCov.element(2,2) = 0.01;
        m_approxCov.element(3,3) = 0.1; m_approxCov.element(4,4) = 0.1; m_approxCov.element(4,4) = 0.3;
      }

  }
}


SurfaceDetection::~SurfaceDetection()
{

}


void PCDToXYZImage(Hobject *ImageX, Hobject *ImageY, Hobject *ImageZ, Hobject *Region,sensor_msgs::PointCloud& pcd, std::vector<double> camparams)
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
    if(pcd.points[i].x == pcd.points[i].x)
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

  gen_image_const(ImageZ, "float", CamParam[6].I() , CamParam[7].I() );
  gen_image_const(ImageX, "float", CamParam[6].I() , CamParam[7].I() );
  gen_image_const(ImageY, "float", CamParam[6].I() , CamParam[7].I() );
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
    tuple_select(X, Index_C, &X);
    tuple_select(Y, Index_C, &Y);
    tuple_select(Z, Index_C, &Z);
  }


  HTuple rowcollect , colcollect;
  try
  {

    set_grayval(*ImageX, Row.Int(), Column.Int(), X);
    set_grayval(*ImageY, Row.Int(), Column.Int(), Y);
    //
    set_grayval(*ImageZ, Row.Int(), Column.Int(), Z);
    gen_region_points(Region, Row.Int(), Column.Int());
  }
  catch(HException ex)
  {
    printf("got an exception trying directly %s\n", ex.message);
    for(int i = 0; i < Row.Num(); i++)
    {
      if(Row[i].I() >= 0 && Column[i].I() >= 0 &&
         Row[i].I() < CamParam[7].I()  && Column[i].I() < CamParam[6].I())
      {
          set_grayval(*ImageX, Row[i].I(), Column[i].I(), X[i]);
          set_grayval(*ImageY, Row[i].I(), Column[i].I(), Y[i]);
          set_grayval(*ImageZ, Row[i].I(), Column[i].I(), Z[i]);
          tuple_concat(rowcollect, Row[i], &rowcollect);
          tuple_concat(colcollect, Column[i], &colcollect);
      }
      else
        printf("Error at: r %d c %d \n x %f y %f  z  %f\n Cam: %d %d (%ld)\n", Row[i].I(), Column[i].I(), X[i].D(), Y[i].D(), Z[i].D(), CamParam[6].I(),CamParam[7].I(), CamParam.Num());
    }
    gen_region_points(Region, rowcollect.Int(), colcollect.Int());
  }


}


std::vector<RelPose*> SurfaceDetection::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
    //Calibration* calib = &cam[0]->m_calibration;
  SurfaceModel* proto = (SurfaceModel*)object.GetElement(0, DESCRIPTOR_SURFACE);
  printf("SurfaceDetection::Perform: Got SegmentPrototype\n");
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
    {
      try
      {
        HTuple area_roi, HomMat3DRotate, ObjectModel3DIDAffineTrans, empty, table_height;
        Hobject imgx, imgy, imgz, region, roi_obj, TranImgX, TranImgY, TranImgZ, region_wo_table;
        HTuple ObjectModel3DIDOnline, Pose, Score, SurfaceMatchingResultID;
        SwissRangerReading* reading = (SwissRangerReading*)((*it)->GetReading());
        m_camparams =  (*it)->GetUnformatedCalibrationValues().second;
        PCDToXYZImage(&imgx, &imgy, &imgz, &region, reading->m_image, m_camparams);
        
        reduce_domain(imgx, region, &imgx);
        Calibration calib;

        Halcon::HTuple CamParam(8);
        for(size_t i = 0; i < m_camparams.size(); i++)
        {
          CamParam[i] = m_camparams[i];
        }

        calib.SetCamParam(CamParam);

        RegionOI roi(pose, reading->GetPose()->m_uniqueID, &calib);
        try
        {
          HTuple rowtmp, coltmp;
          dilation_circle(roi.GetRegion(), &roi_obj, 20);
          intersection(roi_obj, region, &roi_obj);
          area_center(roi_obj, &area_roi, &rowtmp, &coltmp );
        }
        catch(HException ex)
        {
          printf("Surface::Perform: Error creating region: %s\n", ex.message);
          area_roi = 0;
        }
#ifdef _DEBUG
 #endif
        if(area_roi[0].I() > 500)
        {
          Hobject roi_obj_x, roi_obj_y, roi_obj_z;
          reduce_domain(imgx, roi_obj, &imgx);
          Matrix m = pose->GetMatrix(reading->GetPose()->m_uniqueID);
          Matrix cov = pose->GetCovarianceMatrix(reading->GetPose()->m_uniqueID);
          
          threshold(imgx, &roi_obj_x, m.element(0, 3) - 2*cov.element(0,0), m.element(0, 3) + 2*cov.element(0,0));
          threshold(imgy, &roi_obj_y, m.element(1, 3) - 2*cov.element(1,1), m.element(1, 3) + 2*cov.element(1,1));
          threshold(imgz, &roi_obj_z, m.element(2, 3) - 2*cov.element(2,2), m.element(2, 3) + 2*cov.element(2,2));
                    
          printf("Region is big enough\n");
        }
        xyz_to_object_model_3d (imgx, imgy, imgz, &ObjectModel3DIDOnline);
        /*  If the search is limited to tables*/

        if(true)
        {
          try
          {
          HTuple rowtmp, coltmp;
          Hobject domain;
          RelPoseHTuple::GetHommat((*it)->GetRelPose(), &HomMat3DRotate, 1);
          affine_trans_object_model_3d (ObjectModel3DIDOnline, HomMat3DRotate, &ObjectModel3DIDAffineTrans);
          object_model_3d_to_xyz (&TranImgX, &TranImgY, &TranImgZ, ObjectModel3DIDAffineTrans,
                                    "from_xyz_map", empty, empty);
          get_domain( TranImgX, &domain);
          extract_table_height(domain, TranImgZ, &table_height);

          m_lastTableHeight = table_height[0].D();

          threshold(TranImgZ, &region_wo_table, table_height[0].D() + 0.01, table_height[0].D() + 0.80);
          area_center(region_wo_table, &area_roi, &rowtmp, &coltmp);
          clear_object_model_3d(ObjectModel3DIDAffineTrans);
          if(area_roi[0].I() > 500)
          {

            printf("Region without table has area of: %d\n", area_roi[0].I());
            clear_object_model_3d(ObjectModel3DIDOnline);
            reduce_domain(imgx, region_wo_table, &imgx);
            area_center(region_wo_table, &area_roi, &rowtmp, &coltmp);

            xyz_to_object_model_3d (imgx, imgy, imgz, &ObjectModel3DIDOnline);
            printf("Created object model\n");
          }
          }
          catch(HException ex)
          {
            printf("SurfaceDetection::Perform: Error excluding table: %s\n", ex.message);
          }
        }

        find_surface_model (proto->m_surface_handle, ObjectModel3DIDOnline, 0.03, 0.05, proto->m_minscore, "false",
                      "num_matches", numOfObjects, &Pose, &Score, &SurfaceMatchingResultID);
        if(Pose.Num() == 0)
        {
          printf("Retry with lower minscore (%f -> %f)\n", proto->m_minscore, proto->m_minscore* (5.0/6.0));
          find_surface_model (proto->m_surface_handle, ObjectModel3DIDOnline, 0.03, 0.05, proto->m_minscore * (5.0/6.0), "false",
                      "num_matches", numOfObjects, &Pose, &Score, &SurfaceMatchingResultID);
        }
        clear_object_model_3d(ObjectModel3DIDOnline);

        numOfObjects = 0;
        if(Pose.Num() > 0)
        {
          printf("Got %ld Results\n", Pose.Num() / 7);
          for(int i = 0; i < Pose.Num()/7; i++)
          {
            HTuple cov_sel, pose_sel, hom, hom_t(12);
            tuple_select_range(Pose, 0 + 7*i, 6 + 7*i, &pose_sel);

            pose_to_hom_mat3d(pose_sel, &hom);
            hom_t[0] = proto->m_hommat.element(0,0); hom_t[1] = proto->m_hommat.element(0,1); hom_t[2] = proto->m_hommat.element(0,2); hom_t[3] = proto->m_hommat.element(0,3);
            hom_t[4] = proto->m_hommat.element(1,0); hom_t[5] = proto->m_hommat.element(1,1); hom_t[6] = proto->m_hommat.element(1,2); hom_t[7] = proto->m_hommat.element(1,3);
            hom_t[8] = proto->m_hommat.element(2,0); hom_t[9] = proto->m_hommat.element(2,1); hom_t[10] = proto->m_hommat.element(2,2); hom_t[11] = proto->m_hommat.element(2,3);
            hom_mat3d_compose(hom, hom_t, &hom);
            hom_mat3d_to_pose(hom, &pose_sel);
            result.push_back(RelPoseHTuple::FRelPose(pose_sel, proto->m_approxCov, reading->GetPose()));
            if(i < Score.Num())
            {
              result.back()->m_qualityMeasure = Score[i].D();
              if(i == 0)
              {
                qualityMeasure = Score[i].D();
                ShapeModel *shape;
                if((shape = (ShapeModel*)object.GetElement(0, DESCRIPTOR_SHAPE )) != NULL)
                {
                  printf("Show the CAD model\n");
                  shape->m_showNow = true;
                }
              }
            }
            else
              qualityMeasure = 1.0;
            numOfObjects ++;

          }
        }
        else
        {
          printf("No Results\n");
        }
      }
      catch (const char* text )
      {
         printf("Error in SurfaceDetection: %s\n", text);
      }
      break;
    }
  }
  /*TODO plane clusters*/
  return result;
}

Descriptor* SurfaceModelExtraction::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  SegmentPrototype* proto =  (SegmentPrototype*)sig.GetElement(0, DESCRIPTOR_SEGMPROTO);
  SurfaceModel* surfacemodel = NULL;
  qualityMeasure = 0.0;
  if(proto != NULL )
  {

   for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
   if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
   {
    printf("Starting to learn a surface model\n");
    try
    {
      try
      {
        Hobject imgx, imgy, imgz, region;
        HTuple ObjectModel3DModel, empty;
        
        sensor_msgs::PointCloud pcd_in;
        pcd_in = proto->GetPointCloud(pose->m_uniqueID);
        if(pcd_in.points.size() == 0)
        {
          /* Ignoring input pose */ 
          printf("Ignoring input pose, taking pose from object\n");
          pose = proto->GetLastMatchedPose();
          if(pose != NULL)
            pcd_in = proto->GetPointCloud(pose->m_uniqueID);
          else
            break;
        }

        m_camparams = (*it)->GetUnformatedCalibrationValues().second;
        
        sensor_msgs::PointCloud pcd = cloud_trans(pose->m_uniqueID, proto->GetSensorFrameId(pose->m_uniqueID), pcd_in);
        
        /* TODO Check, pcd_in can not be projected*/
        PCDToXYZImage(&imgx, &imgy, &imgz, &region, pcd, m_camparams);

        reduce_domain(imgx, region, &imgx);
        reduce_domain(imgy, region, &imgy);
        reduce_domain(imgz, region, &imgz);

        xyz_to_object_model_3d (imgx, imgy, imgz, &ObjectModel3DModel);

        surfacemodel = new SurfaceModel();
        printf("Call create_surface_model(sizeof(ObjectModel3D) = %ld, 0.03, [], [], ... ) \n", pcd.points.size());
        try
        {
        write_image(imgx, "tiff", 0,"create_surface_model_imgx.tiff");
        write_image(imgy, "tiff", 0,"create_surface_model_imgy.tiff");
        write_image(imgz, "tiff", 0,"create_surface_model_imgz.tiff");
        write_region(region, "create_surface_model_region.reg");
        }
        catch(HException ex)
        {
          printf("Exception in SurfaceModelExtracion: %s\n", ex.message);
        }

        create_surface_model (ObjectModel3DModel, 0.03, empty, empty, &(surfacemodel->m_surface_handle));

        surfacemodel->m_hommat = pose->GetMatrix((*it)->GetRelPose()->m_uniqueID);
        surfacemodel->m_approxCov = pose->GetCovariance((*it)->GetRelPose()->m_uniqueID);
        Class *cl = new Class();
        std::stringstream st;
        st << "Surface_" << cl->m_ID;
        cl->SetName(st.str());
        surfacemodel->SetClass(cl);
        std::stringstream filename;
        filename << "Surface_" << cl->m_ID << ".sfm";
        surfacemodel->filename = filename.str();
        printf("Write surface model\n");
        write_surface_model(surfacemodel->m_surface_handle, surfacemodel->filename.c_str());
      }
      catch(HException ex)
      {
        printf("Exception in SurfaceModelExtracion: %s\n", ex.message);
      }
      catch(char const* ex)
      {
        printf("Learning of surface model failed: %s\n", ex);
      }
      catch(...)
      {
        printf("Unknown Error in SurfaceModelExtracion::Perform\n");
      }
     }
     catch(char const* ex)
     {
       printf("Error: %s\n", ex);
     }
     break;
    }
  }
  return surfacemodel;
}



double SurfaceDetection::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
    {
      if(object.GetElement(0, DESCRIPTOR_SURFACE) != NULL)
        return 3.0;
      else
        return 0.0;
    }
  }
  return 0.0;
}


double SurfaceModelExtraction::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  /*for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    printf("%s ==?   %s || %s\n", (*it)->GetName().c_str(), XML_NODE_SWISSRANGER, XML_NODE_RANGESENSOR);
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
    {*/
      printf("in here?\n");
      if(object.GetElement(0, DESCRIPTOR_SURFACE) == NULL && object.GetElement(0, DESCRIPTOR_SEGMPROTO) != NULL )
        return 1.0;
      else
        return 0.0;
  /*  }
  }*/
  return 0.0;
}



XMLTag* SurfaceDetection::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  tag->AddProperty(XML_ATTRIBUTE_FILENAME, m_camfilename);
  return tag;
}


XMLTag* SurfaceModelExtraction::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  tag->AddProperty(XML_ATTRIBUTE_FILENAME, m_camfilename);
  return tag;
}


void SurfaceModel::SaveTo(XMLTag* tag)
{
    Descriptor::SaveTo(tag);
    tag->AddProperty(XML_ATTRIBUTE_FILENAME, filename);
    tag->AddChild(XMLTag::Tag(m_hommat, XML_NODE_OBJECTMEAN));
    tag->AddChild(XMLTag::Tag(m_approxCov, XML_NODE_OBJECTCOV));
}

