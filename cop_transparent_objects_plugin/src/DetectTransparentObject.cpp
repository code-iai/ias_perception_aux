;/*
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

#include "DetectTransparentObject.h"
#include "DetectTransparentObjectCandidate.h"
#include "TransparentObjectCandidate.h"
#include "TransparentObject.h"
#include "SwissRangerReading.h"
#include "RelPoseHTuple.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
//#include "icp/icp.h"
#include "math.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"


#include "Camera.h"

#define SQR(A) ((A)*(A))

#ifdef _DEBUG
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF if(false) printf
#endif

using namespace cop;
using namespace Halcon;
DetectTransparentObject::DetectTransparentObject()
{
	  ros::NodeHandle Node_Handle;
	  Rec_Cloud_pub  = Node_Handle.advertise<sensor_msgs::PointCloud>("reconstructed_transparent_obj", 1, false);
	  DEBUG_PRINTF("Advertise Node created. \n\n");
}

void DetectTransparentObject::SetData(XMLTag* tag)
{


}

DetectTransparentObject::~DetectTransparentObject(void)
{
}

bool    open4 = false, open5 =  false, open6 = false;
Halcon::HTuple WindowHandle4, WindowHandle5, WindowHandle6;

Descriptor* DetectTransparentObject::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
	TransparentObject* tod = NULL;
  Sensor* selected_sensor = NULL;
  Halcon::Hobject Intensity1, Distance1, Object_Segmentation1, ImageX1, ImageY1, ImageZ1, Intensity2, Distance2, Object_Segmentation2, ImageX2, ImageY2, ImageZ2;
  Halcon::HTuple Glass_Rec_X_base, Glass_Rec_Y_base, Glass_Rec_Z_base, Dev_X_base, Dev_Y_base, Dev_Z_base;
  Halcon::HTuple SRCamParam, RelPoseReal, Hommat_Obj_base, Hommat_base_Obj, ptr, width, height, type, BestPose, GlassMeanXYZ, GlassDeviation;
  Halcon::HTuple k, Glass_Rec_X, Glass_Rec_Y, Glass_Rec_Z, Index;
  Halcon::HTuple Dev_Vec_Xx, Dev_Vec_Xy, Dev_Vec_Xz, Dev_Vec_Yx, Dev_Vec_Yy, Dev_Vec_Yz, Dev_Vec_Zx, Dev_Vec_Zy, Dev_Vec_Zz, Dev_Mat, Hommat_base_SR;
  Halcon::HTuple X, Y, Z, I, D, Qx, Qy, Qz, Row, Column, Row_add, Col_add, Row_sgn, Col_sgn, Index_R, Index_C, DomainRow, DomainCol, HomMat3D_calib;
  Halcon::HTuple OrderTrafo, OrderRot, ViewTrafo, HomMat3D_calib_inv, CandPoseReal, RowCand, ColCand, Hommat_point_SR_to_base;
  Halcon::HTuple X_Sort, Y_Sort, Z_Sort;

  DEBUG_PRINTF("Debug: Before readout of first view. \n\n");
  TransparentObjectCandidate* proto = (TransparentObjectCandidate*)object.GetElement(0, DESCRIPTOR_TRANSPARENTOBJECTCAND);
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      selected_sensor = *it;
      break;
    }
  }

  if(selected_sensor == NULL)
  {
    ROS_ERROR("Unexpectedly failed to find a sensor\n");
    return tod;
  }
  Object_Segmentation1 = proto->m_detectedRegions[pose->m_uniqueID];
  Intensity1 = proto->m_usedIntensities[pose->m_uniqueID];
  Distance1 = proto->m_usedDistance[pose->m_uniqueID];
  ImageX1 = proto->m_usedImageX1[pose->m_uniqueID];
  ImageY1 = proto->m_usedImageY1[pose->m_uniqueID];
  ImageZ1 = proto->m_usedImageZ1[pose->m_uniqueID];

  DEBUG_PRINTF("Debug: Before SwissRangerReading. \n\n");
  SwissRangerReading* reading = (SwissRangerReading*)selected_sensor->GetReading(-1);
	  if(reading == 0)
	  {throw "Error: reading = 0  \n\n";}

  sensor_msgs::PointCloud& pcd = reading->m_image;

  //generate camera parameters and calibration-transformation matrix
  SRCamParam[0] = 0.0105631;
  tuple_concat(SRCamParam, -8623.59, &SRCamParam);
  tuple_concat(SRCamParam, 3.99851e-05, &SRCamParam);
  tuple_concat(SRCamParam, 4e-05, &SRCamParam);
  tuple_concat(SRCamParam, 87.8255, &SRCamParam);
  tuple_concat(SRCamParam, 71.9162, &SRCamParam);
  tuple_concat(SRCamParam, 176, &SRCamParam);
  tuple_concat(SRCamParam, 144, &SRCamParam);
  /*SRCamParam[0] = 0.008;
  tuple_concat(SRCamParam, 0.000394659, &SRCamParam);
  tuple_concat(SRCamParam, 4e-05, &SRCamParam);
  tuple_concat(SRCamParam, 4e-05, &SRCamParam);
  tuple_concat(SRCamParam, 87.5, &SRCamParam);
  tuple_concat(SRCamParam, 71.5, &SRCamParam);
  tuple_concat(SRCamParam, 176, &SRCamParam);
  tuple_concat(SRCamParam, 144, &SRCamParam);*/
  HomMat3D_calib[0] = 1.0;
  tuple_concat(HomMat3D_calib, 9.23486e-05, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 4.58653e-05, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 7.40468e-05, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -9.23646e-05, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 1.0, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 0.000346921, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -0.00111648, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -4.58333e-05, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -0.000346926, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 1.0, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib,  0.00898110, &HomMat3D_calib);
  /*HomMat3D_calib[0] = 1.0;
  tuple_concat(HomMat3D_calib, -2.52254e-11, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -6.27903e-10, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -6.56066e-11, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 2.52254e-11, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 1.0, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 4.25327e-09, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 1.04212e-09, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 6.27903e-10, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, -4.25327e-09, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib, 1.0, &HomMat3D_calib);
  tuple_concat(HomMat3D_calib,  -2.78425e-09, &HomMat3D_calib);*/
  hom_mat3d_invert(HomMat3D_calib ,&HomMat3D_calib_inv);

	  //Image acquisition
	  for(size_t i = 0; i < (pcd.points.size()); i++)
	      {
	        X[i] = pcd.points[i].x;
	        Y[i] = pcd.points[i].y;
	        Z[i] = pcd.points[i].z;
	        I[i] = (unsigned short)pcd.channels[0].values[i];
	        D[i] = sqrt( pcd.points[i].x*pcd.points[i].x + pcd.points[i].y*pcd.points[i].y + pcd.points[i].z*pcd.points[i].z );
	      }
	  try
	  {
		  HTuple zeros;
		  tuple_find(Z, HTuple(0.0), &zeros);
		  if(!(zeros.Num() == 1 && zeros[0].I() == -1))
		  {
			  DEBUG_PRINTF("Found %ld zeros\n", zeros.Num());
			  for(int i = 0; i  < zeros.Num(); i++)
			  {
				  Z[zeros[i].I()] = 0.0000001;
			  }
		  }
		  //move the points away from camera because focal point is not right on the CCD-chip
		  affine_trans_point_3d(HomMat3D_calib, X, Y, Z, &Qx, &Qy, &Qz);
		  //DEBUG_PRINTF("CamParam: %f %f %f %f %f %f\n", CamParam[0].D(),CamParam[1].D(),CamParam[2].D(),CamParam[3].D(),CamParam[4].D(),CamParam[5].D());
		  project_3d_point(Qx, Qy, Qz, SRCamParam, &Row, &Column);
		  tuple_round(Row, &Row);
		  tuple_round(Column, &Column);
	  }
	  catch(HException ex)
	  {
		  DEBUG_PRINTF("Error: %s \n", ex.message);
		  DEBUG_PRINTF("View 2: Image acqusition reprojection or affine transformation failed! \n");
		  DEBUG_PRINTF("Fix failed: %s but s_x= %f s_y =%f \n\n", ex.message, SRCamParam[2].D(), SRCamParam[3].D()  );
	  }
	  //clear tuple from Rows and Columns that are too big or below 0
	  tuple_add(Row, 1, &Row_add);				//all rows are at least 0 + 1
	  tuple_sgn(Row_add, &Row_sgn); 			//all rows > 0 become 1
	  tuple_find(Row_sgn, 1, &Index_R); 		//find all rows > 0
	  tuple_select(Row, Index_R, &Row); 		//select all rows > 0
	  tuple_select(Column, Index_R, &Column);
	  tuple_select(X, Index_R, &X);
	  tuple_select(Y, Index_R, &Y);
	  tuple_select(Z, Index_R, &Z);
	  tuple_select(I, Index_R, &I);
	  tuple_select(D, Index_R, &D);

	  tuple_add(Column, 1, &Col_add); 			//all cols are at least 0 + 1
	  tuple_sgn(Col_add, &Col_sgn); 			//all cols > 0 become 1
	  tuple_find(Col_sgn, 1, &Index_C);			//find all cols > 0
	  tuple_select(Column, Index_C, &Column);	//select all cols > 0
	  tuple_select(Row, Index_C, &Row);
	  tuple_select(X, Index_C, &X);
	  tuple_select(Y, Index_C, &Y);
	  tuple_select(Z, Index_C, &Z);
	  tuple_select(I, Index_C, &I);
	  tuple_select(D, Index_C, &D);

	  tuple_sub(Row, SRCamParam[7].I(), &Row_add);	//all rows from 0-143 are < 0
	  tuple_sgn(Row_add, &Row_sgn);					//all rows >= 0 <= 143 become -1
	  tuple_find(Row_sgn, -1, &Index_R);			//select all rows > 0
	  tuple_select(Row, Index_R, &Row);				//select all rows >= 0 <= 143
	  tuple_select(Column, Index_R, &Column);
	  tuple_select(X, Index_R, &X);
	  tuple_select(Y, Index_R, &Y);
	  tuple_select(Z, Index_R, &Z);
	  tuple_select(I, Index_R, &I);
	  tuple_select(D, Index_R, &D);

	  tuple_sub(Column, SRCamParam[6].I(), &Col_add);	//all cols from 0-175 are < 0
	  tuple_sgn(Col_add, &Col_sgn);						//all cols >= 0 <= 175 become -1
	  tuple_find(Col_sgn, -1, &Index_C);				//select all cols > 0
	  tuple_select(Column, Index_C, &Column);			//select all cols >= 0 <= 175
	  tuple_select(Row, Index_C, &Row);
	  tuple_select(X, Index_C, &X);
	  tuple_select(Y, Index_C, &Y);
	  tuple_select(Z, Index_C, &Z);
	  tuple_select(I, Index_C, &I);
	  tuple_select(D, Index_C, &D);

	  //generate Images
	  gen_image_const(&ImageX2, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&ImageY2, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&ImageZ2, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&Intensity2, "uint2", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&Distance2, "float", SRCamParam[6].I(), SRCamParam[7].I());

	  try
	  {
		  set_grayval(ImageX2, Row.Int(), Column.Int(), X);
		  set_grayval(ImageY2, Row.Int(), Column.Int(), Y);
		  set_grayval(ImageZ2, Row.Int(), Column.Int(), Z);
		  set_grayval(Intensity2, Row.Int(), Column.Int(), I);
		  set_grayval(Distance2, Row.Int(), Column.Int(), D);
	  }
	  catch(HException ex)
	  {
		  DEBUG_PRINTF("Got an exception. Trying to directly acqire images! %s \n", ex.message);
		  for(int i = 0; i < Row.Num(); i++)
		  {
			  if(Row[i].I() >= 0 && Column[i].I() >= 0 &&
					  Row[i].I() < SRCamParam[7].I() + 1 && Column[i].I() < SRCamParam[6].I() + 1)
			  {
				  set_grayval(ImageX2, Row[i].I(), Column[i].I(), X[i]);
				  set_grayval(ImageY2, Row[i].I(), Column[i].I(), Y[i]);
				  set_grayval(ImageZ2, Row[i].I(), Column[i].I(), Z[i]);
				  set_grayval(Intensity2, Row[i].I(), Column[i].I(), I[i]);
				  set_grayval(Distance2, Row[i].I(), Column[i].I(), D[i]);
			  }
			  else
				  DEBUG_PRINTF("Error at: r %d c %d \n x %f y %f  z  %f\n Cam: %d %d (%ld)\n", Row[i].I(), Column[i].I(), X[i].D(), Y[i].D(), Z[i].D(), SRCamParam[6].I(), SRCamParam[7].I(), SRCamParam.Num());
		  }
	  }

  DEBUG_PRINTF("Debug: Before GetPose and Segmentation 2 \n\n");
  try
  {
	  RelPoseHTuple::GetPose(reading->m_relPose, &RelPoseReal, pose->m_parentID);
	  RelPoseHTuple::GetPose(pose, &CandPoseReal, reading->m_relPose->m_uniqueID);
	  project_3d_point(CandPoseReal[0], CandPoseReal[1], CandPoseReal[2], SRCamParam, &RowCand, &ColCand);
	  //if(RelPoseReal[0]==0.0 && RelPoseReal[0]... )  //TODO min Movement
	  //throw
	  get_pose_type(RelPoseReal, &OrderTrafo, &OrderRot, &ViewTrafo);
	  DEBUG_PRINTF("RelPose type: OrderTrafo=%s OrderRot=%s ViewTrafo=%s \n\n", OrderTrafo[0].S(), OrderRot[0].S(), ViewTrafo[0].S());
	  //generate /base_link relPose
	  RelPose* base_link = RelPoseFactory::GetRelPose("/base_link");
	  RelPoseHTuple::GetHommat(reading->m_relPose, &Hommat_point_SR_to_base, base_link->m_uniqueID);
  }
  catch(const char *text)
  {
	  DEBUG_PRINTF("Error: GetPose Error! \n\n");
	  throw text;
  }

  try
  {
	  Segmentation_Glasses(Intensity2, Distance2, Row, Column, &Object_Segmentation2);
	  	  DEBUG_PRINTF("Debug: Segmentation 2 finished \n\n");
  }
  catch(const char *text)
  {
	  DEBUG_PRINTF("Error: Searchspace_Reduction failed! \n\n");
	  throw text;\
  }
  catch(const char* text)
  {
	  DEBUG_PRINTF("%s \n", text);
  }
  catch(...)
  {
	  DEBUG_PRINTF("Error: Searchsparce_Reduction failed! \n\n");
  }

#ifdef _DEBUG
  /*Output for Debugging purposes*/
	  write_image(Intensity1,"tiff",0,"Test_Output/Int1");
	  write_image(Distance1,"tiff",0,"Test_Output/Dis1");
  write_region(Object_Segmentation1, "Test_Output/ObjSeg1.tiff");
	  write_image(Intensity2,"tiff",0,"Test_Output/Int2");
	  write_image(Distance2,"tiff",0,"Test_Output/Dis2");
  write_region(Object_Segmentation2, "Test_Output/ObjSeg2.tiff");
	  write_image(ImageX2,"tiff",0,"Test_Output/IX2");
	  write_image(ImageY2,"tiff",0,"Test_Output/IY2");
	  write_image(ImageZ2,"tiff",0,"Test_Output/IZ2");

  //set_window_attr("background_color","black");
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Intensity1, WindowHandle);
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Distance1, WindowHandle);
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Object_Segmentation1, WindowHandle);
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Intensity2, WindowHandle);
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Distance2, WindowHandle);
  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
  //set_part(WindowHandle,-1,-1,-1,-1);
  //disp_obj(Object_Segmentation2, WindowHandle);
  //DEBUG_PRINTF("Opened Windows to display Regions \n\n\n\n\n");
#endif

	//find out whether there is a glass within the candidates and where it is
	try
	{
		Reconstruct_3D_Glasses(Intensity1, Distance1, Object_Segmentation1, Intensity2,
				Distance2, Object_Segmentation2, ImageX1, ImageY1, ImageZ1, ImageX2, ImageY2, ImageZ2,
				&Glass_Rec_X, &Glass_Rec_Y, &Glass_Rec_Z, SRCamParam, RelPoseReal, &BestPose, HomMat3D_calib_inv, pose->m_uniqueID, RowCand, ColCand, Hommat_point_SR_to_base);
	}
	catch(HException ex)
	{
		DEBUG_PRINTF("Error: %s\n", ex.message);
	}
	catch(const char* text)
	{
		DEBUG_PRINTF("%s \n", text);
	}
	catch(...)
	{
		DEBUG_PRINTF("Error: Reconstruct_3D_Glasses failed! \n\n");
	}


	if(Glass_Rec_X.Num() == 0)
	{
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidatePosRecPos.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld 0\n", pose->m_uniqueID);
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
		numOfObjects = 0;
		qualityMeasure = 0;
		return tod;
	}

  //publish the reconstructed points of the glass
  //Create PCD
  sensor_msgs::PointCloud Rec_Point_Cloud;
  //Filling Rec_Point_Cloud with data
  DEBUG_PRINTF("Debug: Reading data from reconstructed pointloud. \n\n");
  try
  {
	  Index = Glass_Rec_X.Num();
	  Rec_Point_Cloud.points.resize(Index[0].I());
	  for(int iter = 0; iter < Index[0].I(); iter++)
	  {
		  Rec_Point_Cloud.points[iter].x = Glass_Rec_X[iter].D();
		  Rec_Point_Cloud.points[iter].y = Glass_Rec_Y[iter].D();
		  Rec_Point_Cloud.points[iter].z = Glass_Rec_Z[iter].D();
	  }
  }
  catch(HException ex)
  {
    DEBUG_PRINTF("Error: %s\n", ex.message);
  }

  //Publish
  Rec_Point_Cloud.header = pcd.header;
  DEBUG_PRINTF("Debug: Publishing reconstructed pointcloud \n\n");
  Rec_Cloud_pub.publish (Rec_Point_Cloud);

  try
  {
	  //Read mean and cov
	  DEBUG_PRINTF("BestPose: %f %f %f %f %f %f %f %f \n\n", BestPose[0].D(), BestPose[1].D(),BestPose[2].D(), BestPose[3].D(), BestPose[4].D(), BestPose[5].D(), BestPose[6].D(), BestPose[7].D());
	  GlassMeanXYZ[0] = HTuple(BestPose[1]);
	  tuple_concat(GlassMeanXYZ, BestPose[2], &GlassMeanXYZ);
	  tuple_concat(GlassMeanXYZ, HTuple(BestPose[3]), &GlassMeanXYZ);
	  tuple_concat(GlassMeanXYZ, 0, &GlassMeanXYZ);
	  tuple_concat(GlassMeanXYZ, 0, &GlassMeanXYZ);
	  tuple_concat(GlassMeanXYZ, 0, &GlassMeanXYZ);
	  tuple_concat(GlassMeanXYZ, 0, &GlassMeanXYZ);

	  GlassDeviation[0] = HTuple(BestPose[4]);
	  tuple_concat(GlassDeviation, HTuple(BestPose[5]), &GlassDeviation);
	  tuple_concat(GlassDeviation, HTuple(BestPose[6]), &GlassDeviation);
	  tuple_concat(GlassDeviation, 0, &GlassDeviation);
	  tuple_concat(GlassDeviation, 0, &GlassDeviation);
	  tuple_concat(GlassDeviation, 0, &GlassDeviation);

	  //Create m and cov matrices
	  Matrix m(4,4);
	  Matrix cov(6,6);
	  RelPoseHTuple::TupleToMat(GlassMeanXYZ, GlassDeviation, m, cov);

	  //Create Pose for Rviz
	  RelPose* ObjectPose = RelPoseFactory::FRelPose(reading->m_relPose, m, cov);

	  //generate /base_link relPose
	  RelPose* base_link = RelPoseFactory::GetRelPose("/base_link");
	  //Transform the ObjectPose such that it is shown relative to /base_link
	  //Get RelPose from /base_link to ObjectPose to obtain mean in /base_link
	  RelPoseHTuple::GetHommat(base_link, &Hommat_base_Obj, ObjectPose->m_uniqueID);
	  //Get RelPose from ObjectPose to /base_link to obtain a hommat that transforms dev
	  RelPoseHTuple::GetHommat(ObjectPose, &Hommat_Obj_base, base_link->m_uniqueID);
	  affine_trans_point_3d(Hommat_base_Obj, GlassDeviation[0], 0, 0,  &Dev_Vec_Xx, &Dev_Vec_Xy, &Dev_Vec_Xz);
	  affine_trans_point_3d(Hommat_base_Obj, GlassDeviation[1], 0, 0,  &Dev_Vec_Yx, &Dev_Vec_Yy, &Dev_Vec_Yz);
	  affine_trans_point_3d(Hommat_base_Obj, GlassDeviation[2], 0, 0,  &Dev_Vec_Zx, &Dev_Vec_Zy, &Dev_Vec_Zz);
	  //create the cov matrix
	  RelPoseHTuple::GetHommat(reading->m_relPose, &Hommat_base_SR, base_link->m_uniqueID);
	  affine_trans_point_3d(Hommat_base_SR, Glass_Rec_X, Glass_Rec_Y, Glass_Rec_Z, &Glass_Rec_X_base, &Glass_Rec_Y_base, &Glass_Rec_Z_base);
	  //tuple_deviation(Glass_Rec_X_base, &Dev_X_base);
	  //tuple_deviation(Glass_Rec_Y_base, &Dev_Y_base);
	  //tuple_deviation(Glass_Rec_Z_base, &Dev_Z_base);
		//calculate cropped deviation
		tuple_sort(Glass_Rec_X_base, &X_Sort);
		tuple_sort(Glass_Rec_Y_base, &Y_Sort);
		tuple_sort(Glass_Rec_Z_base, &Z_Sort);
		tuple_select_range(X_Sort, round(0.1*X_Sort.Num()) , round(0.9*X_Sort.Num()), &X_Sort);
		tuple_select_range(Y_Sort, round(0.3*Y_Sort.Num()) , round(0.7*Y_Sort.Num()), &Y_Sort);
		tuple_select_range(Z_Sort, round(0.1*Z_Sort.Num()) , round(0.9*Z_Sort.Num()), &Z_Sort);
		tuple_deviation(X_Sort, &Dev_X_base);
		tuple_deviation(Y_Sort, &Dev_Y_base);
		tuple_deviation(Z_Sort, &Dev_Z_base);

	  //generate Object Pose with /base_link as Origin
	 m<<1  << 0 << 0 << (Hommat_Obj_base[11].D() < 0.99 ? Hommat_Obj_base[3].D() - 0.03 : (Hommat_Obj_base[11].D() > 1.00 ?  Hommat_Obj_base[3].D() + 0.03 :  Hommat_Obj_base[3].D() + 0.01))  <<
		0  << 1 << 0 << Hommat_Obj_base[7].D() <<
		0  << 0 << 1 << (Hommat_Obj_base[11].D() < 0.99 ? 0.99 : (Hommat_Obj_base[11].D() > 1.0 ? 0.99 : Hommat_Obj_base[11].D())) <<
		0  << 0 << 0 << 1;
	 cov<<Dev_X_base[0].D() << 0 << 0 << 0 << 0 << 0 <<
		  0 << Dev_Y_base[0].D() << 0 << 0 << 0 << 0 <<
		  0 << 0 << Dev_Z_base[0].D() << 0 << 0 << 0 <<
		  0 << 0 << 0 << 0 << 0 << 0 <<
		  0 << 0 << 0 << 0 << 0 << 0 <<
		  0 << 0 << 0 << 0 << 0 << 0;
	  RelPoseFactory::FreeRelPose(&ObjectPose);
	  ObjectPose = RelPoseFactory::FRelPose(base_link->m_uniqueID, m, cov);

#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidatePosRecPos.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %ld\n", pose->m_uniqueID, ObjectPose->m_uniqueID);
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif

	  //Prepare ObjectPose for output
	  DEBUG_PRINTF("Debug: Object Pose  m_uniqueID %ld \n\n", ObjectPose->m_uniqueID);
	  Class* cl = new Class();
	  cl->SetName("TransparentObject");
	  tod= new TransparentObject(cl);
			  tod->SetLastMatchedImage(reading, ObjectPose);
			  proto->SetLastMatchedImage(reading, ObjectPose);

			  if (Glass_Rec_X.Num() == 0)
			  {
				numOfObjects = 0;
				qualityMeasure = 0;
			  }
			  else
			  {
				numOfObjects = 1;
				qualityMeasure = 1.0;
			  }
  }
  catch(HException ex)
  {
	  DEBUG_PRINTF("Error: %s\n", ex.message);
  }
  catch(const char* text)
  {
	  DEBUG_PRINTF("%s \n", text);
  }
  catch(...)
  {
	  DEBUG_PRINTF("Error: Reconstruct_3D_Glasses failed! \n\n");
  }

	return tod;


}

double DetectTransparentObject::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sensors)
{
  if(sig.GetElement(0,DESCRIPTOR_TRANSPARENTOBJECTCAND) != NULL &&
     sig.GetElement(0,DESCRIPTOR_TRANSPARENTOBJECT) == NULL)
  {
        for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
        {
          if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
          {
            
            return 1.0;
          }
        }
        return 0.0;
    }
  else
    return 0.0;
}

XMLTag* DetectTransparentObject::Save()
{
	XMLTag* tag = new XMLTag(GetName());

	//TODO: parameter?
	return tag;
}


void Reconstruct_3D_Glasses (Halcon::Hobject Intensity1, Halcon::Hobject Distance1,
    Halcon::Hobject CandidateRegion1, Halcon::Hobject Intensity2, Halcon::Hobject Distance2,
    Halcon::Hobject CandidateRegion2, Halcon::Hobject ImageX1, Halcon::Hobject ImageY1,
    Halcon::Hobject ImageZ1, Halcon::Hobject ImageX2, Halcon::Hobject ImageY2,
    Halcon::Hobject ImageZ2, Halcon::HTuple *PCDX_Rec, Halcon::HTuple *PCDY_Rec, Halcon::HTuple *PCDZ_Rec,
    Halcon::HTuple SwissCamParam, Halcon::HTuple RelPoseReal, Halcon::HTuple *FoundGlassMeanDev,
    Halcon::HTuple HomMat3D_calib_inv, LocatedObjectID_t CandidatePosID, Halcon::HTuple RowCand, Halcon::HTuple ColCand,
    Halcon::HTuple Hommat_point_SR_to_base)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  IntEquHisto1, Object_Seg1_Con, CandidateSelected;
  Hobject  ImReduced_to_Glass1, ModelContours, ModelContours_Trans;
  Hobject  IntEquHisto2, ImReduced_to_Glass2, ModelContours2;
  Hobject  ModelContours_Trans2, OriginRegion, Transformed_RegionGlass2;
  Hobject  GlassFirstView, GlassSecondView;
  Hobject  ContoursSplit1, UnionContours1, SelectedContours1;
  Hobject  CandidateSelectedDil, CandidateRegion2Dil;
  Hobject  Projection1to2, XYZVal2, CandidateRegion2_Reduced, CandidateDomain, FullDomain;


  // Local control variables
  HTuple  Number, i, ModelID1, Row_Model, Column_Model;
  HTuple  Angle_Model, ScaleR_Model, ScaleC_Model, Score1_Model;
  HTuple  HomMat2DIdentity, HomMat2DTranslate, HomMat2D_Glass1, HomMat2D_Glass1_Inv;
  HTuple  Row_Model2, Column_Model2, Angle_Model2, ScaleR_Model2;
  HTuple  ScaleC_Model2, Score1_Model2, HomMat2DIdentity2;
  HTuple  HomMat2DTranslate2, HomMat2D_Glass2, HomMat2D_Glass2_Comp,Rows_GlassRegion1;
  HTuple  Columns_GlassRegion1, Grayval_Glass1, HomMat2D_Glass_ToOrigin;
  HTuple  OrigX_GlassRegion1, OrigY_GlassRegion1, FoundX_GlassRegion2;
  HTuple  FoundY_GlassRegion2, RoundY_GlassRegion2, RoundX_GlassRegion2;
  HTuple  Grayval_Glass2, GrayDiff, AbsDiff, NegPos, Sgn;
  HTuple  Length, Found_Glass2_X, Found_Glass2_Y, RowsGlass1_Matched;
  HTuple  ColumnsGlass1_Matched, SelectedSgn, SelectedX, SelectedY;
  HTuple  SelectedRows, SelectedColumns, HomMat2D, Index;
  HTuple  SelectedMatch, HomMat3DReal, HomMat3DRealTrans;
  HTuple  RelPose, CovRelPose, Error, X, Y, Z, CovXYZ, RelPose1;
  HTuple  X_Glass_Rec, Y_Glass_Rec, Z_Glass_Rec, X_Rec, Y_Rec, Z_Rec, Dist_Glass_Rec;
  HTuple  X_Glass_Rec_mean, Y_Glass_Rec_mean, Z_Glass_Rec_mean, X_Glass_Rec_dev, Y_Glass_Rec_dev, Z_Glass_Meas_Rec_mean, Z_Glass_Meas_Rec_dev;
  HTuple  NumberContours1;
  HTuple  Rows1, Columns1, XVal1, YVal1, ZVal1, Rows2, Columns2, XVal2, YVal2, ZVal2;
  HTuple  Distance_Matching, Distance_to_Center;
  HTuple  TransX2to1, TransY2to1, TransZ2to1, Rot_X2to1, Rot_Y2to1, Rot_Z2to1;
  HTuple  Pose1in2, HomMat3D1to2;
  HTuple  RowMax_GlassRegion2, ColMax_GlassRegion2, RowMin_GlassRegion2, ColMin_GlassRegion2;
  HTuple  Qx1, Qy1, Qz1, RowRound_Reproj_1to2, ColRound_Reproj_1to2, Row_Reproj_1to2, Column_Reproj_1to2;
  HTuple  RowMax_Reproj_1to2, RowMin_Reproj_1to2, ColMax_Reproj_1to2, ColMin_Reproj_1to2;
  HTuple  Distance_Hom2D_Reproj, SumDistances, Number_Corres, Dist_LinesOfSight;
  HTuple  FoundGlassMeanDev_Temp, Aniso_Length, Check_ModelID;
  HTuple  HomMat2D_planar, ModelParamValues1, ModelParamValues2, aniso;
  HTuple  Rot_Thresh, RotX_Thresh, RotY_Thresh, RotZ_Thresh, RotY_direction, RotX_direction, RotZ_direction;
  HTuple  model_indices, data_indices, Rows2_icp, Columns2_icp, Rows1_icp, Columns1_icp;
  HTuple  XVal2_mean, YVal2_mean, ZVal2_mean, Qx1_mean, Qy1_mean, Qz1_mean, HomMat3D_screw, HomMat3D1to2_inv, RelPose1to2;
  HTuple  Rows2_sel, Columns2_sel, Rows1_sel, Rows1_ind, Rows1_len, Rows1_ind_max, Rows1_ind_min, Rows1_intersect, Columns1_intersect, Rows2_intersect, Columns2_intersect;
  HTuple  Sorted_1to2_Ind, Sorted_2_Ind, Sorted_1to2_Row, Sorted_1to2_Col, Sorted_2_Row, Sorted_2_Col, Length_Sort_2, Min_Row_1to2, Min_Row_2, Min_Col_1to2;
  HTuple  Min_Col_2, Mean_Row_1to2, Mean_Col_1to2, Mean_Row_2, Mean_Col_2, HomMat2DIdentity_RowMin, HomMat2DTranslate_RowMin, Row_1to2_aligned, Col_1to2_aligned;
  HTuple  Row_C1to2, Column_C1to2, Row_C2, Column_C2, HomMat2DIdentity_C1to2, HomMat2DTranslate_C1to2, HomMat2DIdentity_C2, HomMat2DTranslate_C2, Row_C1to2_Q;
  HTuple  Column_C1to2_Q, Row_C2_Q, Column_C2_Q, HomMat2D_C, Sx_C, Sy_C, Phi_C, Theta_C, Tx_C, Ty_C, Angle_C, Dif_Dist_Rec_Real, Dif_Dist_sqrt1, Dif_Dist_sqrt2;
  HTuple  Selected, DomainRow, DomainCol, Fail;
  HTuple  Qx1_base, Qy1_base, Qz1_base, XVal2_base, YVal2_base, ZVal2_base;
  double angle_planes=0;
  //Reconstruct Glasses
  DEBUG_PRINTF("Debug: Reconstruction routine started. \n\n");
  aniso = 0;
  DEBUG_PRINTF("Debug: aniso = 0 set - deformable models will be used! \n\n");
	//Init for fwrite
	SumDistances = -1.0;
	Angle_C = -1.0;
	Dif_Dist_Rec_Real = -1.0;
	angle_planes = -1.0;
	RotY_direction = -1.0;
	RotX_Thresh = -1.0;
	RotY_Thresh = -1.0;
	RotZ_Thresh = -1.0;
	Rot_Thresh = -1.0;
	Fail = 0;


  //Generate Models
  equ_histo_image(Intensity1, &IntEquHisto1);
  connection(CandidateRegion1, &Object_Seg1_Con);
  count_obj(Object_Seg1_Con, &Number);
  for (i=1; i<=Number; i+=1)
  {
	  while(1) //no transformation is still out of bounds
	  {

		while(1) //no decent matching was possible
		{


			while(1) //no decent shape_model can be created
			{

				if(i > Number)
				{break;}
				select_obj(Object_Seg1_Con, &CandidateSelected, i);

#ifdef _DEBUG
					DEBUG_PRINTF("Opening Window for Selected Candidate Region \n\n");
					//open_window(0,0, 176,144, 0, "","", &WindowHandle);
					//set_part(WindowHandle,-1,-1,-1,-1);
					//disp_obj(CandidateSelected, WindowHandle);
#endif

				dilation_circle(CandidateSelected, &CandidateSelectedDil, 2);
				reduce_domain(Intensity1/*IntEquHisto1*/, CandidateSelectedDil, &ImReduced_to_Glass1);
				try{
					set_check("~give_error");
					ModelID1 = HTuple();

					if(aniso == 1)
					{
						create_aniso_shape_model(ImReduced_to_Glass1, "auto", -3.14, 3.14, "auto", 0.9,
								1.1, "auto", 0.9, 1.1, "auto", "auto", "use_polarity", "auto", "auto", &ModelID1);
					}
					else
					{
						create_planar_uncalib_deformable_model(ImReduced_to_Glass1, "auto", -1.57, 1.57,
					        "auto", 0.9, 1.1, "auto", 0.9, 1.1, "auto", "none", "use_polarity", "auto", "auto",
					        HTuple(), HTuple(), &ModelID1);
						write_deformable_model(ModelID1, "Test_Output/Model.dfm");
						DEBUG_PRINTF("Debug: Deformable Model created \n\n");
					}

					set_check("give_error");
				}catch(HException ex)
				{
					DEBUG_PRINTF("Error: create_aniso_shape_model failed (model too small) %s \n\n",ex.message);
				}

				Check_ModelID = ModelID1.Num();
				if(Check_ModelID == 0)
				{
					DEBUG_PRINTF("Error: create_aniso_shape_model failed (model too small) \n");
					DEBUG_PRINTF("Moving on to next candidate! \n\n");
					if(aniso == 1)
					{
						clear_shape_model (ModelID1);
					}
					else
					{
						clear_deformable_model (ModelID1);
					}
					i = i+1;
				}
				else
				{
					try{
						set_check("~give_error");

							if(aniso == 1)
							{
								find_aniso_shape_model(ImReduced_to_Glass1, ModelID1, -3.14, 3.14, 0.9, 1.1,
									0.9, 1.1, 0.5, 1, 0.5, "least_squares", 2, 0.9, &Row_Model, &Column_Model,
									&Angle_Model, &ScaleR_Model, &ScaleC_Model, &Score1_Model);
							}
							else
							{
								DEBUG_PRINTF("Debug: trying to find deformable model in first view \n");
								find_planar_uncalib_deformable_model(ImReduced_to_Glass1, ModelID1, -0.2, 0.2,
							        0.9, 1.1, 0.9, 1.1, 0.1, 1, 0.8, 2, 0.9, HTuple(), HTuple(), &HomMat2D_planar, &Score1_Model);
								DEBUG_PRINTF("Debug: Deformable Model found in first view \n");
							    get_deformable_model_params(ModelID1, (HTuple("model_row").Append("model_col")),
							        &ModelParamValues1);
							    Row_Model.Reset();
							    Column_Model.Reset();
							    Row_Model[0] = HTuple(ModelParamValues1[0]);
							    Column_Model[0] = HTuple(ModelParamValues1[1]);
							    DEBUG_PRINTF("Debug: Deformable Model center found \n\n");

							}

						set_check("give_error");

					}catch(HException ex)
					{
						DEBUG_PRINTF("Error: find_aniso_shape_model after creation failed () %s \n\n",ex.message);

					}

						Aniso_Length = Score1_Model.Num();

						if(Aniso_Length == 0)
						{
							DEBUG_PRINTF("Error: Could not find match in first view for current candidates ansio_shape_model! \n");
							DEBUG_PRINTF("Moving on to next candidate! \n\n");
							if(aniso == 1)
							{
								clear_shape_model (ModelID1);
							}
							else
							{
								clear_deformable_model (ModelID1);
							}
							i = i+1;
						}
						else
						{
							if(aniso == 1)
							{
								get_shape_model_contours(&ModelContours, ModelID1, 1);
								hom_mat2d_identity(&HomMat2DIdentity);
								hom_mat2d_translate(HomMat2DIdentity, Row_Model, Column_Model, &HomMat2DTranslate);
								hom_mat2d_rotate(HomMat2DTranslate, Angle_Model, Row_Model, Column_Model, &HomMat2D_Glass1);
								affine_trans_contour_xld(ModelContours, &ModelContours_Trans, HomMat2D_Glass1);
							}
							else
							{
								DEBUG_PRINTF("Debug: Trying to get Model edges \n\n");
								edges_sub_pix (ImReduced_to_Glass1, &ModelContours, "canny", 1, 20, 40);
#ifdef _DEBUG
									//open_window(0,0, 176,144, 0, "","", &WindowHandle);
									//set_part(WindowHandle,-1,-1,-1,-1);
									//disp_obj(ModelContours, WindowHandle);
#endif
							    HomMat2D_Glass1.Reset();
							    HomMat2D_Glass1[0] = 1;
							    HomMat2D_Glass1[1] = 0;
							    HomMat2D_Glass1.Append(HTuple(ModelParamValues1[0]));
							    HomMat2D_Glass1.Append(0);
							    HomMat2D_Glass1.Append(1);
							    HomMat2D_Glass1.Append(HTuple(ModelParamValues1[1]));
							    HomMat2D_Glass1.Append(0);
							    HomMat2D_Glass1.Append(0);
							    HomMat2D_Glass1.Append(1);
							    DEBUG_PRINTF("Debug: Transformation Matrix to Origin created \n\n");
							    hom_mat2d_invert(HomMat2D_Glass1, &HomMat2D_Glass1_Inv);
							    try
							    {
							    	projective_trans_image(ImReduced_to_Glass1, &ModelContours_Trans, HomMat2D_Glass1_Inv, "bilinear",
							    			"false", "false");
							    }
							    catch(Halcon::HException ex)
							    {
							    	DEBUG_PRINTF("Error: %s\n", ex.message);
							    }

							}
							//if the model is too short and squishy it should be omitted
							//in order to avoid missmatches and 3D reconstruction of tiny
							//structures on surfaces
							if(aniso == 1)
							{
								segment_contours_xld (ModelContours_Trans, &ContoursSplit1, "lines_circles", 10, 1000, 100);
							}
							else
							{
								segment_contours_xld (ModelContours, &ContoursSplit1, "lines_circles", 10, 1000, 100);
							}
							union_adjacent_contours_xld (ContoursSplit1, &UnionContours1, 10000, 99999, "attr_forget");
							select_contours_xld (UnionContours1, &SelectedContours1, "contour_length", 50, 99999, 0, 0);
							count_obj (SelectedContours1, &NumberContours1);
								if (NumberContours1 == 0)
								{
									if(aniso == 1)
									{
										clear_shape_model (ModelID1);
									}
									else
									{
										clear_deformable_model (ModelID1);
									}
									i = i+1;
									DEBUG_PRINTF("Error: Shape-Model was too small. \n Moving on to next candidate. \n\n");
								}
								else
								{
									break;
								}
						}
				}//endif check_modelID

			}//END WHILE
			if(i > Number)
			{break;}
				DEBUG_PRINTF("Debug: Aniso Shape Model creation successful. \n\n");

			try{
				//Find Models in Image2
				Searchspace_Reduction (RelPoseReal, SwissCamParam, ImageX1, ImageY1, ImageZ1, CandidateSelected, &DomainRow, &DomainCol);
				gen_region_points(&CandidateDomain, DomainRow, DomainCol);
				gen_rectangle1(&FullDomain, 0, 0, SwissCamParam[7].I(), SwissCamParam[6].I());
				intersection(CandidateDomain, FullDomain, &CandidateDomain);
				intersection(CandidateRegion2, CandidateDomain, &CandidateRegion2_Reduced);
#ifdef _DEBUG
				write_region(CandidateRegion2, "Test_Output/CandidateRegion2.tiff");
#endif
				DEBUG_PRINTF("No Equalized Histograms used for deformable_models right now! \n\n");
				equ_histo_image(Intensity2, &IntEquHisto2);

				dilation_circle(CandidateRegion2_Reduced, &CandidateRegion2Dil, 2);
					/*open_window(0,0, 176,144, 0, "","", &WindowHandle);
					set_part(WindowHandle,-1,-1,-1,-1);
					disp_obj(CandidateRegion2_Reduced, WindowHandle);*/

				reduce_domain(Intensity2/*IntEquHisto2*/, CandidateRegion2Dil, &ImReduced_to_Glass2);
					/*open_window(0,0, 176,144, 0, "","", &WindowHandle);
					set_part(WindowHandle,-1,-1,-1,-1);
					disp_obj(ImReduced_to_Glass2, WindowHandle);*/

				set_check("~give_error");
				DEBUG_PRINTF("Debug: Trying to find Model in 2nd view.  \n\n");
				if(aniso == 1)
				{
					find_aniso_shape_model(ImReduced_to_Glass2, ModelID1, -3.14, 3.14, 0.9, 1.1,
							0.9, 1.1, 0.5, 1, 0.5, "least_squares", 2, 0.9, &Row_Model2, &Column_Model2,
							&Angle_Model2, &ScaleR_Model2, &ScaleC_Model2, &Score1_Model2);
				}
				else
				{
					find_planar_uncalib_deformable_model(ImReduced_to_Glass2, ModelID1, -0.78, 0.78,
					        0.9, 1.1, 0.95, 1.05, 0.5, 1, 0.7, 2, 0.9, HTuple(), HTuple(), &HomMat2D_Glass2, &Score1_Model2);
					Aniso_Length = Score1_Model2.Num();
					if(Aniso_Length == 0)
					{
						DEBUG_PRINTF("Debug: First planar matching failed! \n");
						find_planar_uncalib_deformable_model(ImReduced_to_Glass2, ModelID1, -1.54, 1.54,
								0.9, 1.1, 0.9, 1.1, 0.5, 1, 0.7, 2, 0.9, HTuple(), HTuple(), &HomMat2D_Glass2, &Score1_Model2);
						Aniso_Length = Score1_Model2.Num();
						if(Aniso_Length == 0)
						{
							DEBUG_PRINTF("Debug: Second planar matching failed! \n\n");
							find_planar_uncalib_deformable_model(ImReduced_to_Glass2, ModelID1, -0.78, 0.78,
									0.9, 1.1, 0.95, 1.05, 0.3, 1, 0.7, 2, 0.9, HTuple(), HTuple(), &HomMat2D_Glass2, &Score1_Model2);
							Aniso_Length = Score1_Model2.Num();
						}
					}
						if(Aniso_Length != 0)
						{
							projective_trans_pixel(HomMat2D_Glass2, 0, 0, &Row_Model2, &Column_Model2);
						}
				}
				set_check("give_error");

				Aniso_Length = Score1_Model2.Num();
				if(Aniso_Length == 0)
				{
					DEBUG_PRINTF("Error: Could not find match in second view for current candidates ansio_shape_model! \n");
					DEBUG_PRINTF("Moving on to next candidate! \n\n");
					if(aniso == 1)
					{
						clear_shape_model (ModelID1);
					}
					else
					{
						clear_deformable_model (ModelID1);
					}
					i = i+1;
				}
				else
				{
					//If the found shape Model is really far away fron the original
					//then its probably a mismatch as the robot wouldn't move that far.
					//distance_pp(Row_Model, Column_Model, Row_Model2, Column_Model2, &Distance_Matching);
					distance_pp(RowCand, ColCand, Row_Model2, Column_Model2, &Distance_Matching);
					distance_pp(SwissCamParam[5], SwissCamParam[4], Row_Model2, Column_Model2, &Distance_to_Center);

					 if (Distance_Matching > 50 && aniso == 0 && Distance_to_Center > 30)
					 {
						 DEBUG_PRINTF("Debug: Distance between Model in first and second view is very high! \n");
						 DEBUG_PRINTF("Debug: OR \n");
						 DEBUG_PRINTF("Debug: Distance between Match and image center is very high! \n");
						 DEBUG_PRINTF("Debug: Removing candidate in searchspace and redoing matching! \n\n");
						 Hobject WrongRegion, WrongComplement, WrongExtinct;
						 hom_mat2d_invert(HomMat2D_Glass1, &HomMat2D_Glass_ToOrigin);
						 hom_mat2d_compose(HomMat2D_Glass2 ,HomMat2D_Glass_ToOrigin , &HomMat2D_Glass2_Comp);
						 get_region_points(CandidateSelected, &Rows_GlassRegion1, &Columns_GlassRegion1);
						 projective_trans_pixel(HomMat2D_Glass2_Comp, Rows_GlassRegion1, Columns_GlassRegion1,
								&FoundX_GlassRegion2, &FoundY_GlassRegion2);
						 tuple_round(FoundY_GlassRegion2, &RoundY_GlassRegion2);
						 tuple_round(FoundX_GlassRegion2, &RoundX_GlassRegion2);
						 gen_region_points(&WrongRegion, RoundX_GlassRegion2, RoundY_GlassRegion2);
						 complement(WrongRegion, &WrongComplement);
						 intersection(CandidateRegion2Dil, WrongComplement, &WrongExtinct);
						 reduce_domain(Intensity2, WrongExtinct, &ImReduced_to_Glass2);
						 find_planar_uncalib_deformable_model(ImReduced_to_Glass2, ModelID1, -0.78, 0.78,
								0.9, 1.1, 0.95, 1.05, 0.5, 1, 0.7, 2, 0.9, HTuple(), HTuple(), &HomMat2D_Glass2, &Score1_Model2);
						 Aniso_Length = Score1_Model2.Num();
							if(Aniso_Length == 0)
							{
								DEBUG_PRINTF("Debug: Distance between Model in first and second view is very high! \n");
								DEBUG_PRINTF("MISMATCH WARNING! MOVING ON WITH NEXT CANDIDATE! \n\n");
								clear_deformable_model (ModelID1);
								i=i+1;
								continue;
							}
						 projective_trans_pixel(HomMat2D_Glass2, 0, 0, &Row_Model2, &Column_Model2);
						 //distance_pp(Row_Model, Column_Model, Row_Model2, Column_Model2, &Distance_Matching);
						 distance_pp(RowCand, ColCand, Row_Model2, Column_Model2, &Distance_Matching);
						 distance_pp(SwissCamParam[5], SwissCamParam[4], Row_Model2, Column_Model2, &Distance_to_Center);
					 }


					if (Distance_Matching > 50 && Distance_to_Center >30)
					{
						DEBUG_PRINTF("Debug: Distance between Model in first and second view is very high! \n");
						DEBUG_PRINTF("Debug: Row1 %f Col1 %f Row2 %f Col2 %f \n", Row_Model[0].D(), Column_Model[0].D(), Row_Model2[0].D(), Column_Model2[0].D());
						DEBUG_PRINTF("Debug: OR \n");
						DEBUG_PRINTF("Debug: Distance between Match and image center is very high! \n");
						DEBUG_PRINTF("Debug: RowCenter %f ColCenter %f Row2 %f Col2 %f \n", SwissCamParam[5].D(), SwissCamParam[4].D(), Row_Model2[0].D(), Column_Model2[0].D());
						DEBUG_PRINTF("MISMATCH WARNING! MOVING ON WITH NEXT CANDIDATE! \n\n");
						if(aniso == 1)
						{
							clear_shape_model (ModelID1);
						}
						else
						{
							clear_deformable_model (ModelID1);
						}
					i = i+1;
					}
					else
					{
						break;
					}
				}
			}catch(HException ex){DEBUG_PRINTF("%s \n",ex.message);}


		}//END WHILE
		if(i > Number)
		{break;}


	if(aniso == 1)
	{
		get_shape_model_contours(&ModelContours2, ModelID1, 1);
		hom_mat2d_identity(&HomMat2DIdentity2);
		hom_mat2d_translate(HomMat2DIdentity2, Row_Model2, Column_Model2, &HomMat2DTranslate2);
		hom_mat2d_rotate(HomMat2DTranslate2, Angle_Model2, Row_Model2, Column_Model2,
				&HomMat2D_Glass2);
		affine_trans_contour_xld(ModelContours2, &ModelContours_Trans2, HomMat2D_Glass2);
	}
	else
	{
		hom_mat2d_invert(HomMat2D_Glass1, &HomMat2D_Glass_ToOrigin);
		hom_mat2d_compose(HomMat2D_Glass2 ,HomMat2D_Glass_ToOrigin , &HomMat2D_Glass2_Comp);
		projective_trans_image(ImReduced_to_Glass1, &ModelContours_Trans2, HomMat2D_Glass2_Comp, "bilinear",
		        "false", "false");
	}

    DEBUG_PRINTF("Debug: Shape Model Matching done! \n");
    DEBUG_PRINTF("Debug: Row1 %f Col1 %f Row2 %f Col2 %f \n\n", Row_Model[0].D(), Column_Model[0].D(), Row_Model2[0].D(), Column_Model2[0].D());
		//open_window(0,0, 176,144, 0, "","", &WindowHandle);
		//set_part(WindowHandle,-1,-1,-1,-1);
		//disp_obj(ModelContours_Trans2, WindowHandle);

    //Create point correspondence
    //transform Object_Segmentation1 into Object_Segmentation2
    get_region_points(CandidateSelected, &Rows_GlassRegion1, &Columns_GlassRegion1);

    if(aniso == 1)
    {
    	hom_mat2d_invert(HomMat2D_Glass1, &HomMat2D_Glass_ToOrigin);
    	affine_trans_point_2d(HomMat2D_Glass_ToOrigin, Rows_GlassRegion1, Columns_GlassRegion1,
    			&OrigX_GlassRegion1, &OrigY_GlassRegion1);

    	affine_trans_point_2d(HomMat2D_Glass2, OrigX_GlassRegion1, OrigY_GlassRegion1,
    			&FoundX_GlassRegion2, &FoundY_GlassRegion2);
    }
    else
    {
    	projective_trans_pixel(HomMat2D_Glass2_Comp, Rows_GlassRegion1, Columns_GlassRegion1,
    	        &FoundX_GlassRegion2, &FoundY_GlassRegion2);
    }

    tuple_round(FoundY_GlassRegion2, &RoundY_GlassRegion2);
    tuple_round(FoundX_GlassRegion2, &RoundX_GlassRegion2);
    DEBUG_PRINTF("Debug: Candidate Region transformed into second view. \n\n");
#ifdef _DEBUG
					Hobject Cand_Trans;
					gen_region_points(&Cand_Trans, RoundX_GlassRegion2, RoundY_GlassRegion2);
					if(!open5)
					{
					  open_window(0,0, 176,144, 0, "","", &WindowHandle5);
					  open5 = true;
					}
					set_part(WindowHandle5,-1,-1,-1,-1);
					disp_obj(Intensity1, WindowHandle5);
					disp_obj(CandidateSelected, WindowHandle5);
					if(!open4)
					{
					  open_window(0,0, 176,144, 0, "","", &WindowHandle4);
					  open4 = true;
					}
					set_part(WindowHandle4,-1,-1,-1,-1);
					disp_obj(Intensity2, WindowHandle4);
					disp_obj(Cand_Trans, WindowHandle4);
					write_region(CandidateSelected, "Test_Output/CandidateSel.tiff");
					write_region(Cand_Trans, "Test_Output/CandidateTrans.tiff");
#endif

	//Exception-Handling for transformation from Image1 to Image2
    //If transformed Region is out of bounds
    tuple_max(RoundX_GlassRegion2, &RowMax_GlassRegion2);
    tuple_max(RoundY_GlassRegion2, &ColMax_GlassRegion2);
    tuple_min(RoundX_GlassRegion2, &RowMin_GlassRegion2);
	tuple_min(RoundY_GlassRegion2, &ColMin_GlassRegion2);
	DEBUG_PRINTF("RowMax %f  RowMin %f  ColMax %f  ColMin %f \n\n",RowMax_GlassRegion2[0].D(), RowMin_GlassRegion2[0].D(), ColMax_GlassRegion2[0].D(), ColMin_GlassRegion2[0].D());
	if (RowMax_GlassRegion2>143)
	{
		DEBUG_PRINTF("Error: Transformed Region OUT OF BOUNDS! \n");
		DEBUG_PRINTF("Moving on to next candidate! \n\n");

		if(aniso == 1)
		{
			clear_shape_model (ModelID1);
		}
		else
		{
			clear_deformable_model (ModelID1);
		}
		i = i+1;
	}
	else if (RowMin_GlassRegion2<0)
	{
		DEBUG_PRINTF("Error: Transformed Region OUT OF BOUNDS! \n");
		DEBUG_PRINTF("Moving on to next candidate! \n\n");

		if(aniso == 1)
		{
			clear_shape_model (ModelID1);
		}
		else
		{
			clear_deformable_model (ModelID1);
		}
		i = i+1;
	}
	else if (ColMax_GlassRegion2>175)
	{
		DEBUG_PRINTF("Error: Transformed Region OUT OF BOUNDS! \n");
		DEBUG_PRINTF("Moving on to next candidate! \n\n");

		if(aniso == 1)
		{
			clear_shape_model (ModelID1);
		}
		else
		{
			clear_deformable_model (ModelID1);
		}
		i = i+1;
	}
	else if (ColMin_GlassRegion2<0)
	{
		DEBUG_PRINTF("Error: Transformed Region OUT OF BOUNDS! \n");
		DEBUG_PRINTF("Moving on to next candidate! \n\n");

		if(aniso == 1)
		{
			clear_shape_model (ModelID1);
		}
		else
		{
			clear_deformable_model (ModelID1);
		}
		i = i+1;
	}
	else
	{
		gen_region_points(&Transformed_RegionGlass2, RoundX_GlassRegion2, RoundY_GlassRegion2);
		break;
	}


	} //END WHILE LAST BEFORE FOR
	  if(i > Number)
	  {break;}

    //From known robot movement we calculate the new position of the 3D points.
	//These new points will be projected into the image plane.
	//As a transparent object behaves different from opaque objects when perceived by
	//the SR4000, we will find a certain misplacement within our reprojection.
	//This misplacement shows us that the object we were considering is actually transparent.

	//Define image points that correspond to each other and
	//corresponding 3D points.
	get_region_points(CandidateSelected, &Rows1, &Columns1);
	get_grayval(ImageX1, Rows1, Columns1, &XVal1);
	get_grayval(ImageY1, Rows1, Columns1, &YVal1);
	get_grayval(ImageZ1, Rows1, Columns1, &ZVal1);

	Rows2 = RoundX_GlassRegion2;
	Columns2 = RoundY_GlassRegion2;
	get_grayval(ImageX2, Rows2, Columns2, &XVal2);
	get_grayval(ImageY2, Rows2, Columns2, &YVal2);
	get_grayval(ImageZ2, Rows2, Columns2, &ZVal2);

	DEBUG_PRINTF("Debug: Image point and 3D correspondences ready. \n\n");

	//Pose1 in Coordinates of Pose2
	//RelPoseReal contains the transformation from Pose1 to Pose2
	DEBUG_PRINTF("RelPose: %f %f %f %f %f %f \n\n",RelPoseReal[0].D(),RelPoseReal[1].D(),RelPoseReal[2].D(),RelPoseReal[3].D(),RelPoseReal[4].D(),RelPoseReal[5].D());
	TransX2to1[0] = RelPoseReal[0];
	TransY2to1[0] = RelPoseReal[1];
	TransZ2to1[0] = RelPoseReal[2];
	Rot_X2to1[0] = RelPoseReal[3];
	Rot_Y2to1[0] = RelPoseReal[4];
	Rot_Z2to1[0] = RelPoseReal[5];

	//Define Camera Poses
	try
	{
		//new image aquisition fits RelPose input
		create_pose(TransX2to1, TransY2to1, TransZ2to1, Rot_X2to1, Rot_Y2to1, Rot_Z2to1, "Rp+T", "gba", "point", &Pose1in2);
		DEBUG_PRINTF("Debug: Pose created. \n\n");
		pose_to_hom_mat3d(Pose1in2, &HomMat3D1to2);
		hom_mat3d_invert(HomMat3D1to2, &HomMat3D1to2_inv);
		DEBUG_PRINTF("Debug: HomMat3D created. \n\n");

		//Transform 3D Points and reproject 3D Points into 2D Image
		affine_trans_point_3d(HomMat3D1to2_inv, XVal1, YVal1, ZVal1, &Qx1, &Qy1, &Qz1);
		DEBUG_PRINTF("Debug: 3D points transformed \n\n");
	}
	catch(Halcon::HException ex)
	{
		DEBUG_PRINTF("Error: %s\n", ex.message);
	}

	try
	{
		project_3d_point(Qx1, Qy1, Qz1, SwissCamParam, &Row_Reproj_1to2, &Column_Reproj_1to2);
	}
	catch(Halcon::HException ex)
	{
		DEBUG_PRINTF("Error: %s\n", ex.message);
		for(int i = 0; i < Qz1.Num(); i++)
		{
		  if(Qz1[i].D() < 0.0001)
		  {
			  Qz1[i] = 0.001;
		  }
		}
	}
	DEBUG_PRINTF("Debug: 3D points Reprojected \n\n");
	tuple_round(Row_Reproj_1to2, &RowRound_Reproj_1to2);
	tuple_round(Column_Reproj_1to2, &ColRound_Reproj_1to2);
	gen_region_points(&Projection1to2, RowRound_Reproj_1to2, ColRound_Reproj_1to2);
	DEBUG_PRINTF("Debug: 3D Points transformed from view 1 to 2 \n and projected into image plane of view 2. \n\n");

	//Exception Handling for Reprojection OUT OF BOUNDS!
	tuple_max(RowRound_Reproj_1to2, &RowMax_Reproj_1to2);
	tuple_max(ColRound_Reproj_1to2, &ColMax_Reproj_1to2);
	tuple_min(RowRound_Reproj_1to2, &RowMin_Reproj_1to2);
	tuple_min(ColRound_Reproj_1to2, &ColMin_Reproj_1to2);
	DEBUG_PRINTF("RowMax_Reproj %f  RowMin_Reproj %f  ColMax_Reproj %f  ColMin_Reproj %f \n",RowMax_Reproj_1to2[0].D(), RowMin_Reproj_1to2[0].D(), ColMax_Reproj_1to2[0].D(), ColMin_Reproj_1to2[0].D());
	if (RowMax_Reproj_1to2 > 143 || RowMin_Reproj_1to2 < 0 || ColMax_Reproj_1to2 > 175 || ColMin_Reproj_1to2 < 0)
	{
		DEBUG_PRINTF("Warning: Reprojected Region OUT OF BOUNDS! \n\n");
	}

#ifdef _DEBUG
		/*open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(Transformed_RegionGlass2, WindowHandle);

		open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(Projection1to2, WindowHandle);

		open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(Transformed_RegionGlass2, WindowHandle);
		disp_obj(Projection1to2, WindowHandle);*/
#endif

    //Calculate the distance between Reprojected and Measured 2D Points
	//Robot localization error is ignored by aligning the lower boundaries of the regions.
	//The assumtption is that the bottom of a transparent objecct is the closest part respective to the camera
	//and therefor the lowest part of the region in the image. The twist of the transparent object should not
	//affect the part directly on the table as much as the elevated part.
	try
	{
		tuple_sort_index (RowRound_Reproj_1to2, &Sorted_1to2_Ind);
		tuple_sort_index (Rows2, &Sorted_2_Ind);
		tuple_select (RowRound_Reproj_1to2, Sorted_1to2_Ind, &Sorted_1to2_Row);
		tuple_select (ColRound_Reproj_1to2, Sorted_1to2_Ind, &Sorted_1to2_Col);
		tuple_select (Rows2, Sorted_2_Ind, &Sorted_2_Row);
		tuple_select (Columns2, Sorted_2_Ind, &Sorted_2_Col);
		tuple_length (Sorted_2_Row, &Length_Sort_2);
		tuple_select_range (Sorted_1to2_Row, 0, round(0.1*Length_Sort_2[0].D()), &Min_Row_1to2);
		tuple_select_range (Sorted_1to2_Col, 0, round(0.1*Length_Sort_2[0].D()), &Min_Col_1to2);
		tuple_select_range (Sorted_2_Row, 0, round(0.1*Length_Sort_2[0].D()), &Min_Row_2);
		tuple_select_range (Sorted_2_Col, 0, round(0.1*Length_Sort_2[0].D()), &Min_Col_2);
		tuple_mean (Min_Row_1to2, &Mean_Row_1to2);
		tuple_mean (Min_Col_1to2, &Mean_Col_1to2);
		tuple_mean (Min_Row_2, &Mean_Row_2);
		tuple_mean (Min_Col_2, &Mean_Col_2);
		hom_mat2d_identity (&HomMat2DIdentity_RowMin);
		hom_mat2d_translate (HomMat2DIdentity_RowMin, Mean_Row_2-Mean_Row_1to2, Mean_Col_2-Mean_Col_1to2, &HomMat2DTranslate_RowMin);
		affine_trans_point_2d (HomMat2DTranslate_RowMin, RowRound_Reproj_1to2, ColRound_Reproj_1to2, &Row_1to2_aligned, &Col_1to2_aligned);
		distance_pp (Row_1to2_aligned, Col_1to2_aligned, Rows2, Columns2, &Distance_Hom2D_Reproj);
		tuple_sum (Distance_Hom2D_Reproj, &SumDistances);
		tuple_length(Distance_Hom2D_Reproj, &Number_Corres);
	}
	catch(Halcon::HException ex)
	{
		DEBUG_PRINTF("Error: %s\n", ex.message);
	}
	Fail = 1; //used for fwrite checks
    if (SumDistances > Number_Corres[0].D()*4)
    {
    	DEBUG_PRINTF("Debug: Distance between pixels of reprojected and \n segmented regions > number_of_points*5 . \n\n");

    	//Calculate the rotation between the 2D region of the matching candidate and
    	//the transformed and reprojected candidate of view 1.
    	try
    	{
			tuple_mean (RowRound_Reproj_1to2, &Row_C1to2);
			tuple_mean (ColRound_Reproj_1to2, &Column_C1to2);
			tuple_mean (Rows2,  &Row_C2);
			tuple_mean (Columns2, &Column_C2);
			hom_mat2d_identity (&HomMat2DIdentity_C1to2);
			hom_mat2d_translate (HomMat2DIdentity_C1to2, -Row_C1to2, -Column_C1to2, &HomMat2DTranslate_C1to2);
			hom_mat2d_identity (&HomMat2DIdentity_C2);
			hom_mat2d_translate (HomMat2DIdentity_C2, -Row_C2, -Column_C2, &HomMat2DTranslate_C2);
			affine_trans_point_2d (HomMat2DTranslate_C1to2, RowRound_Reproj_1to2, ColRound_Reproj_1to2, &Row_C1to2_Q, &Column_C1to2_Q);
			affine_trans_point_2d (HomMat2DTranslate_C2, Rows2, Columns2, &Row_C2_Q, &Column_C2_Q);
			vector_to_similarity (Row_C1to2_Q, Column_C1to2_Q, Row_C2_Q, Column_C2_Q, &HomMat2D_C);
			hom_mat2d_to_affine_par (HomMat2D_C, &Sx_C, &Sy_C, &Phi_C, &Theta_C, &Tx_C, &Ty_C);
				if(Phi_C[0].D() < 0)
				{
					Phi_C = -Phi_C;
				}
			Angle_C[0] = (Phi_C[0].D()*180)/3.14;
    	}
    	catch(Halcon::HException ex)
    	{
    		DEBUG_PRINTF("Error: %s\n", ex.message);
    		Angle_C[0] = 16.0;
    	}

    	if (Angle_C[0].D() > 15.0)
    	{
    		DEBUG_PRINTF("Debug: 2D Rotation big enough to represent transparency! \n\n");

			//Mean must be behind mean_reconstruct because by a certain distance even though matching is wrong.
			//glass-shadow moves. For opaque: too much rotation needed or not enough translation.
    		try
    		{
				hom_mat3d_to_pose(HomMat3D1to2_inv, &RelPose1to2);
				intersect_lines_of_sight(SwissCamParam, SwissCamParam, RelPose1to2, Rows2, Columns2, Rows1, Columns1, &X_Glass_Rec, &Y_Glass_Rec, &Z_Glass_Rec, &Dist_LinesOfSight);
				affine_trans_point_3d(HomMat3D_calib_inv, X_Glass_Rec, Y_Glass_Rec, Z_Glass_Rec, &X_Glass_Rec, &Y_Glass_Rec, &Z_Glass_Rec);
				tuple_mean(X_Glass_Rec, &X_Glass_Rec_mean);
				tuple_mean(Y_Glass_Rec, &Y_Glass_Rec_mean);
				tuple_mean(Z_Glass_Rec, &Z_Glass_Rec_mean);
				tuple_mean(XVal2, &XVal2_mean);
				tuple_mean(YVal2, &YVal2_mean);
				tuple_mean(ZVal2, &ZVal2_mean);
				tuple_sqrt(SQR(XVal2_mean) + SQR(YVal2_mean) + SQR(ZVal2_mean), &Dif_Dist_sqrt1);
				tuple_sqrt(SQR(X_Glass_Rec_mean) + SQR(Y_Glass_Rec_mean) + SQR(Z_Glass_Rec_mean), &Dif_Dist_sqrt2);
				tuple_sub(Dif_Dist_sqrt1, Dif_Dist_sqrt2, &Dif_Dist_Rec_Real);

    		}
    		catch(Halcon::HException ex)
    		{
    			DEBUG_PRINTF("Error: %s\n", ex.message);
    		}

    	    if(Dif_Dist_Rec_Real[0].D() > -0.04)
    	    {
    		    DEBUG_PRINTF("Debug: Reconstructed planar points are closer to camera than real 3D candidate points! \n\n");

					//Reconstruction is planar and nearly orthogonal to glass on the flat surface.
					try
    		    	{
						pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
						cloud1.points.resize (Number_Corres[0].D());
						cloud2.points.resize (Number_Corres[0].D());
						for (int m = 0; m < (signed)cloud1.points.size(); m++)
						{
							tuple_select(XVal2, m, &Selected);
							cloud1.points[m].x = Selected[0].D();
							tuple_select(YVal2, m, &Selected);
							cloud1.points[m].y = Selected[0].D();
							tuple_select(ZVal2, m, &Selected);
							cloud1.points[m].z = Selected[0].D();
							tuple_select(X_Glass_Rec, m, &Selected);
							cloud2.points[m].x = Selected[0].D();
							tuple_select(Y_Glass_Rec, m, &Selected);
							cloud2.points[m].y = Selected[0].D();
							tuple_select(Z_Glass_Rec, m, &Selected);
							cloud2.points[m].z = Selected[0].D();
						}
							pcl::ModelCoefficients coefficients1, coefficients2;
							pcl::PointIndices inliers1, inliers2;
							// Create the segmentation object
							pcl::SACSegmentation<pcl::PointXYZ> seg;
							//Optional
							seg.setOptimizeCoefficients (true);
							//Mandatory
							seg.setModelType (pcl::SACMODEL_PLANE);
							seg.setMethodType (pcl::SAC_RANSAC);
							seg.setDistanceThreshold (0.02);
							seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud1));
							seg.segment (inliers1, coefficients1);
							if (inliers1.indices.size () == 0)
							{
								ROS_ERROR ("Could not estimate a planar model for the given dataset.");
							}
							seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud2));
							seg.segment (inliers2, coefficients2);
							if (inliers2.indices.size () == 0)
							{
								ROS_ERROR ("Could not estimate a planar model for the given dataset.");
							}
							//Calculate angle between the normals which are described as coefficients.
							double norm_length1 = sqrt( SQR(coefficients1.values[0]) + SQR(coefficients1.values[1]) + SQR(coefficients1.values[2]) + SQR(coefficients1.values[3]) );
							double norm_length2 = sqrt( SQR(coefficients2.values[0]) + SQR(coefficients2.values[1]) + SQR(coefficients2.values[2]) + SQR(coefficients2.values[3]) );
							angle_planes = acos( (coefficients1.values[0]*coefficients2.values[0] + coefficients1.values[1]*coefficients2.values[1] + coefficients1.values[2]*coefficients2.values[2] + coefficients1.values[3]*coefficients2.values[3]) / (norm_length1*norm_length2) ) * (180/3.14);
							DEBUG_PRINTF("Angle between the two planes fitted into the recinstructed \n and the SR4 pcd = %f \n\n", angle_planes);

    		    	}
    		    	catch(Halcon::HException ex)
    		    	{
    		    		DEBUG_PRINTF("Error: %s\n", ex.message);
    		    	}
    		    	if(angle_planes > 20 && angle_planes < 180)
    		    	{
    		    		DEBUG_PRINTF("Debug: Reconstructed planar points are roughly orthogonal to the SR4 3D points! \n\n");
    					//Resetting Variables
    					X_Glass_Rec = HTuple();
    					Y_Glass_Rec = HTuple();
    					Z_Glass_Rec = HTuple();
    					X_Glass_Rec_mean = HTuple();
    					Y_Glass_Rec_mean = HTuple();
    					Z_Glass_Rec_mean = HTuple();
    					XVal2_mean = HTuple();
    					YVal2_mean = HTuple();
    					ZVal2_mean = HTuple();

						//Calculate Rotations between two pointclouds
						//Angle range between 0-180 degree
						DEBUG_PRINTF("Debug: Threshold cascade passed! \n");
						DEBUG_PRINTF("Calculating 3D rotational threshold \n\n");
						try
						{
							DEBUG_PRINTF("Debug: Transforming ICP pcds to base_link \n");
							affine_trans_point_3d(Hommat_point_SR_to_base, Qx1, Qy1, Qz1, &Qx1_base, &Qy1_base, &Qz1_base);
							affine_trans_point_3d(Hommat_point_SR_to_base, XVal2, YVal2, ZVal2, &XVal2_base, &YVal2_base, &ZVal2_base);
							ICP_screw_param(XVal2_base, YVal2_base, ZVal2_base, Qx1_base, Qy1_base, Qz1_base, &RotX_Thresh, &RotY_Thresh, &RotZ_Thresh, &HomMat3D_screw, &RotY_direction, &RotX_direction, &RotZ_direction);
							DEBUG_PRINTF("Debug: Rotations for base_link RotZ = %f RotZDir = %f \n", RotZ_Thresh[0].D(), RotZ_direction[0].D());
							ICP_screw_param(XVal2, YVal2, ZVal2, Qx1, Qy1, Qz1, &RotX_Thresh, &RotY_Thresh, &RotZ_Thresh, &HomMat3D_screw, &RotY_direction, &RotX_direction, &RotZ_direction);
							DEBUG_PRINTF("Debug: Rotations for sr4 RotY = %f RotDir = %f \n\n", RotY_Thresh[0].D(), RotY_direction[0].D());
							Rot_Thresh = RotX_Thresh + RotY_Thresh + RotZ_Thresh;
						}
						catch(Halcon::HException ex)
						{
							DEBUG_PRINTF("Error: %s\n", ex.message);
						}

			    		//Estimate 3D movement and determine whether the 3D points move opposed to the camera.
			    		//The rotational direction estimated by the ICP gives an idea how the object behaves.
						if( (RelPoseReal[0].D() < 0 && RotY_direction > 0 && RotY_direction <= 180) || (RelPoseReal[0].D() > 0 && RotY_direction >= 180 && RotY_direction <= 360) )
						{
							DEBUG_PRINTF("Debug: Rotation between 3D points from view1 and view2 is \n opposed to camera translation and apposite to camera rotation! \n\n");


							if(RotY_Thresh > 10 && RotY_Thresh < 90)
							//if(RotX_Thresh < 45 && RotY_Thresh < 45 && RotZ_Thresh < 45 && Rot_Thresh > 20)
							{
								DEBUG_PRINTF("Rotation between reprojected and real 3D PCD: \n  RotX = %f   RotY = %f   RotZ = %f \n\n", RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D());


								//ICP formerly used to obtain a 3D matching
								//Create Input for ICP
								//Matching: Rows2 Columns2 XVal2 YVal2 ZVal2,  Rows1 Columns1 XVal1 YVal1 ZVal1
								//Matching: RowRound_Reproj_1to2 ColRound_Reproj_1to2 Qx1 Qy1 Qz1
							/*	DEBUG_PRINTF("Info: Starting ICP to obtain better 3D matching \n\n");
								int model_n = Rows1.Num();
								double rotation[9];
								rotation[0]=rotation[4]=rotation[8]=1;
								rotation[1]=rotation[2]=rotation[3]=rotation[5]=rotation[6]=rotation[7]=0;
								double translation[3];
								translation[0]=translation[1]=translation[2]=0;
								double *model_1xn3 = new double[3*model_n];
								int* model_index = new int[model_n];
								double* data_1xn3 = new double[3*model_n];
								double* data_weights = new double[model_n];
								//kdtree points
								double *model_1x3n = new double[model_n*3];

								//free points from centroid
								tuple_mean(XVal2, &XVal2_mean);
								tuple_mean(YVal2, &YVal2_mean);
								tuple_mean(ZVal2, &ZVal2_mean);
								tuple_mean(Qx1, &Qx1_mean);
								tuple_mean(Qy1, &Qy1_mean);
								tuple_mean(Qz1, &Qz1_mean);
								for(int k=0; k<model_n; k++)
								{
									HTuple  XSel, YSel, ZSel;
									//create model_points wkth structure XYZ XYZ ...
									//and kdtree model points with structure XXX YYY ZZZ
									tuple_select(XVal2, k, &XSel);
									tuple_select(YVal2, k, &YSel);
									tuple_select(ZVal2, k, &ZSel);
									//model_1x3n[k] = model_1xn3[3*k+0] = -(XSel[0].D() - XVal2_mean[0].D());
									model_1x3n[k] = model_1xn3[3*k+0] = (XSel[0].D() - XVal2_mean[0].D()); //TODO no more minuses
									//model_1x3n[model_n + k] = model_1xn3[3*k+1] = -(YSel[0].D() - YVal2_mean[0].D());
									model_1x3n[model_n + k] = model_1xn3[3*k+1] = (YSel[0].D() - YVal2_mean[0].D()); //TODO no more minuses
									model_1x3n[2*model_n + k] = model_1xn3[3*k+2] = ZSel[0].D() - ZVal2_mean[0].D();
									//create data_poknts with structure XYZ XYZ ...
									//tuple_select(XVal1, k, XSel);
									//tuple_select(YVal1, k, YSel);
									//tuple_select(ZVal1, k, ZSel);
									tuple_select(Qx1, k, &XSel);
									tuple_select(Qy1, k, &YSel);
									tuple_select(Qz1, k, &ZSel);
									data_1xn3[3*k+0] = XSel[0].D() - Qx1_mean[0].D();
									data_1xn3[3*k+1] = YSel[0].D() - Qy1_mean[0].D();
									data_1xn3[3*k+2] = ZSel[0].D() - Qz1_mean[0].D();
									//create index for kdtree
									model_index[k] = k;

									//initialize weights
									data_weights[k] = 1;
								}

								//Create the kd-tree
								//tree = build_kdtree(model.points, model.n, 3, model.index, model.n, 0);
								//tree is a pointer of the type tree* which is defined in kdtree_common.h
								//model.points contains X Y Z placed in row and is called reference in kdtree_common.cc
								//model.n is the Number of Points of the Model = N in kdtree_common.cc
								//Dimension = 3 = D in kdtree_common.cc
								//model.index is a pointer to the list 1,...,model.n which will be sorted by quicksort in kdtree_common.cc
								//model.n is needed twice and used as  L to describe the length of the X Y Z sequence in model.points
								//offset = 0, because in model.points the first element is a coordinate value
								//
								//build_kdtree uses L N und D for some loops
								//model.index is sortet in quicksort by using a define function
								//this function directly accesse model.points=reference
								//the define function needs dimension D and length L
								//in order to find the XYZ in reference=model.points but sorts index
								//
								//generate tree* pointer
								Tree *treeptr=NULL;
								//build tree
								treeptr = build_kdtree(model_1x3n, model_n, 3, model_index, model_n, 0);

								//start ICP
								//rotation is the initial rotation = trpr in icp
								//translation is the initial translation = ttpr in icp
								//model_1xn3 contains the model points in a XYZ XYZ... format = modelz in icp
								//model_n is the number of points = nmodelz in icp
								//data_1xn3 contains the points of the pointcloud that will be matched = dataz in icp
								//data_weights could contain a weighting function, but will contain model_n ones only = qltyz
								//model_n is the same for data and model as we have an initial matching
								//next is the break condition defined by the maximum number of iterations
								//treeptr is a pointer obtained by build_kdtree
								//the two return values give the indices of the HTuples X/Y/ZVal1/2 which are now matched
									  rotation[0] = HomMat3D_screw[0].D(); rotation[1] = HomMat3D_screw[1].D(); rotation[2] = HomMat3D_screw[2].D(); rotation[3] = HomMat3D_screw[4].D(); rotation[4] = HomMat3D_screw[5].D(); rotation[5] = HomMat3D_screw[6].D(); rotation[6] = HomMat3D_screw[8].D(); rotation[7] = HomMat3D_screw[9].D(); rotation[8] = HomMat3D_screw[10].D();
									  translation[0] = HomMat3D_screw[3].D(); translation[1] = HomMat3D_screw[7].D(); translation[2] = HomMat3D_screw[11].D();
									  //TODO try catch for ICP
									  icp( rotation, translation, model_1xn3, (unsigned int)model_n, data_1xn3, data_weights,
											  (unsigned int)model_n, 50, treeptr, &model_indices , &data_indices);

								//Free allocated memory for the kd-tree.
								free_tree(treeptr->rootptr);
								delete model_1x3n;
								//Free allocated memory for ICP.
								delete model_1xn3;
								delete model_index;
								delete data_1xn3;
								delete data_weights;

								//Update matching
								tuple_select(Rows1, data_indices, &Rows1_icp);
								tuple_select(Columns1, data_indices, &Columns1_icp);
								tuple_select(Rows2, model_indices, &Rows2_icp);
								tuple_select(Columns2, model_indices, &Columns2_icp);
								tuple_sort_index(Rows1_icp, &Index);
								tuple_select(Rows1_icp, Index, &Rows1);
								tuple_select(Columns1_icp, Index, &Columns1);
								tuple_sort_index(Rows2_icp, &Index);
								tuple_select(Rows2_icp, Index, &Rows2);
								tuple_select(Columns2_icp, Index, &Columns2);

								//Reconstruct with ICP matching
									//hom_mat3d_to_pose(HomMat3D1to2_inv, &RelPose1to2);
									//intersect_lines_of_sight(SwissCamParam, SwissCamParam, (((((TransX2to1.Concat(TransY2to1)).Concat(TransZ2to1)).Concat(Rot_X2to1)).Concat(Rot_Y2to1)).Concat(Rot_Z2to1)).Concat(0),
									//      Rows2_icp, Columns2_icp,  Rows1_icp, Columns1_icp, &X_Glass_Rec, &Y_Glass_Rec, &Z_Glass_Rec, &Dist_LinesOfSight);

						   */

								DEBUG_PRINTF("Debug: Reconstructing 3D representation! \n\n");
								try
								{
									//Reconstruction without ICP by intersecting 1 Point of View2 with a Row of View 1
									for(int j=0; j<= Rows2.Num()-1; j++)
									{
										tuple_select(Rows2, j, &Rows2_sel);
										tuple_select(Columns2, j, &Columns2_sel);
										tuple_select(Rows1, j, &Rows1_sel);
										tuple_find(Rows1, Rows1_sel, &Rows1_ind);
										tuple_round(Rows1_ind.Num()/2, &Rows1_len);
										tuple_min(Rows1_ind, &Rows1_ind_min);
										tuple_max(Rows1_ind, &Rows1_ind_max);
										if(RelPoseReal[0].D() < 0)
										{
											if( (j-Rows1_len+1) <= Rows1_ind_min)
											{
												tuple_select_range(Rows1, Rows1_ind_min, j+1, &Rows1_intersect);
												tuple_select_range(Columns1, Rows1_ind_min, j+1, &Columns1_intersect);
												tuple_gen_const( (j - Rows1_ind_min +1+1), Rows2_sel, &Rows2_intersect);
												tuple_gen_const( (j - Rows1_ind_min +1+1), Columns2_sel, &Columns2_intersect);
											}
											else
											{
												if( j <  Rows1_ind_max)
												{
													tuple_select_range(Rows1, j-Rows1_len+1, j+1, &Rows1_intersect);
													tuple_select_range(Columns1, j-Rows1_len+1, j+1, &Columns1_intersect);
													tuple_gen_const(Rows1_len+1, Rows2_sel, &Rows2_intersect);
													tuple_gen_const(Rows1_len+1, Columns2_sel, &Columns2_intersect);
												}
												else
												{
													tuple_select_range(Rows1, j-Rows1_len+1, j, &Rows1_intersect);
													tuple_select_range(Columns1, j-Rows1_len+1, j, &Columns1_intersect);
													tuple_gen_const(Rows1_len, Rows2_sel, &Rows2_intersect);
													tuple_gen_const(Rows1_len, Columns2_sel, &Columns2_intersect);
												}


											}
										}
										else
										{
											if( (j+Rows1_len-1) > Rows1_ind_max)
											{
												tuple_select_range(Rows1, j-1, Rows1_ind_max, &Rows1_intersect);
												tuple_select_range(Columns1, j-1, Rows1_ind_max, &Columns1_intersect);
												tuple_gen_const( (Rows1_ind_max - j +1+1), Rows2_sel, &Rows2_intersect);
												tuple_gen_const( (Rows1_ind_max - j +1+1), Columns2_sel, &Columns2_intersect);
											}
											else
											{
												if( j >  Rows1_ind_min)
												{
													tuple_select_range(Rows1, j-1, j+Rows1_len-1, &Rows1_intersect);
													tuple_select_range(Columns1, j-1, j+Rows1_len-1, &Columns1_intersect);
													tuple_gen_const(Rows1_len+1, Rows2_sel, &Rows2_intersect);
													tuple_gen_const(Rows1_len+1, Columns2_sel, &Columns2_intersect);
												}

												else
												{
													tuple_select_range(Rows1, j, j+Rows1_len-1, &Rows1_intersect);
													tuple_select_range(Columns1, j, j+Rows1_len-1, &Columns1_intersect);
													tuple_gen_const(Rows1_len, Rows2_sel, &Rows2_intersect);
													tuple_gen_const(Rows1_len, Columns2_sel, &Columns2_intersect);
												}

											}
										}
										intersect_lines_of_sight(SwissCamParam, SwissCamParam, RelPose1to2, Rows2_intersect, Columns2_intersect, Rows1_intersect, Columns1_intersect, &X_Rec, &Y_Rec, &Z_Rec, &Dist_LinesOfSight);
											tuple_concat(X_Glass_Rec, X_Rec, &X_Glass_Rec);
											tuple_concat(Y_Glass_Rec, Y_Rec, &Y_Glass_Rec);
											tuple_concat(Z_Glass_Rec, Z_Rec, &Z_Glass_Rec);
									}//endfor
									//intersect_lines_of_sight(SwissCamParam, SwissCamParam, RelPose1to2, Rows2, Columns2, Rows1, Columns1, &X_Glass_Rec, &Y_Glass_Rec, &Z_Glass_Rec, &Dist_LinesOfSight);
									DEBUG_PRINTF("Debug: Reconstruction done! \n\n");
								}
								catch(Halcon::HException ex)
								{
									DEBUG_PRINTF("Error: %s\n", ex.message);
								}
								catch(const char* text)
								{
									DEBUG_PRINTF("%s \n", text);
								}
								catch(...)
								{
									DEBUG_PRINTF("Error intersect_lines of sight \n\n");
								}

								//calculate mean and deviation
								HTuple X_Sort, Y_Sort, Z_Sort;
								tuple_sort(X_Glass_Rec, &X_Sort);
								tuple_sort(Y_Glass_Rec, &Y_Sort);
								tuple_sort(Z_Glass_Rec, &Z_Sort);
								tuple_select_range(X_Sort, round(0.1*X_Sort.Num()) , round(0.9*X_Sort.Num()), &X_Sort);
								tuple_select_range(Y_Sort, round(0.3*Y_Sort.Num()) , round(0.7*Y_Sort.Num()), &Y_Sort);
								tuple_select_range(Z_Sort, round(0.1*Z_Sort.Num()) , round(0.9*Z_Sort.Num()), &Z_Sort);
								//calc
								tuple_mean(X_Sort, &X_Glass_Rec_mean);
								tuple_mean(Y_Sort, &Y_Glass_Rec_mean);
								tuple_mean(Z_Sort, &Z_Glass_Meas_Rec_mean);
								tuple_deviation(X_Sort, &X_Glass_Rec_dev);
								tuple_deviation(Y_Sort, &Y_Glass_Rec_dev);
								tuple_deviation(Z_Sort, &Z_Glass_Meas_Rec_dev);

								//OUTPUT POSE
									FoundGlassMeanDev_Temp[0] = 1; //Glass found -> 1
									tuple_concat(FoundGlassMeanDev_Temp, X_Glass_Rec_mean, &FoundGlassMeanDev_Temp);
									tuple_concat(FoundGlassMeanDev_Temp, Y_Glass_Rec_mean, &FoundGlassMeanDev_Temp);
									tuple_concat(FoundGlassMeanDev_Temp, Z_Glass_Meas_Rec_mean, &FoundGlassMeanDev_Temp);
									tuple_concat(FoundGlassMeanDev_Temp, X_Glass_Rec_dev, &FoundGlassMeanDev_Temp);
									tuple_concat(FoundGlassMeanDev_Temp, Y_Glass_Rec_dev, &FoundGlassMeanDev_Temp);
									tuple_concat(FoundGlassMeanDev_Temp, Z_Glass_Meas_Rec_dev, &FoundGlassMeanDev_Temp);
									DEBUG_PRINTF("Debug: Output vector created \n\n");
									tuple_concat(FoundGlassMeanDev_Temp, 1, &FoundGlassMeanDev_Temp);  //control variable glass found -> 1
									tuple_concat(FoundGlassMeanDev_Temp, 1, &FoundGlassMeanDev_Temp);  //control variable quality = 1

									*FoundGlassMeanDev = FoundGlassMeanDev_Temp;

								//OUTPUT POINTCLOUD
									*PCDX_Rec = X_Glass_Rec;
									*PCDY_Rec = Y_Glass_Rec;
									*PCDZ_Rec = Z_Glass_Rec;

									DEBUG_PRINTF("Info: Transparent Object found in candidate %f \n",i[0].D());
									DEBUG_PRINTF("Info: Leaving Reconstruction Routine! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
									break;
							}//endif Rotations 3D
							else
							{
								DEBUG_PRINTF("Warning: Rotation between reprojected and view2 3D points too low! \n");
								DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
							}
						}//endif rotation Rot_Y from ICP has reight direction
						else
						{
			        		DEBUG_PRINTF("Warning: Direction of y-rotation between view1 and view2 3D points not appropriate! \n");
			        		DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
						}
    		    	}//endif rotation between planar reconstruction and SR4 3D points
    		    	else
    		    	{
    	        		DEBUG_PRINTF("Warning: Rotation between reconstructed and SR4 3D point not big enough, too big or \n not in positive x direction! \n");
    	        		DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
    		    	}
    	    }//endif distance to mean_reconstruct smaller than distance to mean_real
    	    else
    	    {
    	    	DEBUG_PRINTF("Debug: Distance between Rec and SR4 %f \n", Dif_Dist_Rec_Real[0].D());
        		DEBUG_PRINTF("Warning: Reconstructed points not in front of SR4 3D point! \n");
        		DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
    	    }

    	}//endif 2D rotation > 20
    	else
    	{
    		DEBUG_PRINTF("Warning: Rotation between reprojected region and matched region too small! \n");
    		DEBUG_PRINTF("Angle_C[0].D() = %f < 20.0", Angle_C[0].D());
    		DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
    	}


    }//endif dist > Num_Corres*5
    else
    {
    	DEBUG_PRINTF("Warning: Distance between reprojected region and matched region too small! \n");
    	DEBUG_PRINTF("Moving on to next candidate! \n\n");
#ifdef _DEBUG
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
#endif
    }






      DEBUG_PRINTF("Info: Run %f done!\n\n",i[0].D());



  }//END FOR 3D RECONSTRUCTION

  DEBUG_PRINTF("Info: All candidates processed! \n\n");

#ifdef _DEBUG
  if(Fail == 0)
  {
				FILE* FileHandle = fopen("CandidateAttrLog.txt","a");
				char buffer[1000];
				sprintf(buffer, "%ld %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n", CandidatePosID, SumDistances[0].D(), Angle_C[0].D(), Dif_Dist_Rec_Real[0].D(), angle_planes, RelPoseReal[0].D(), RotY_direction[0].D(), RotX_Thresh[0].D(), RotY_Thresh[0].D(), RotZ_Thresh[0].D(), Rot_Thresh[0].D());
				fwrite(buffer, 1, strlen(buffer)*sizeof(*buffer), FileHandle);
				fclose(FileHandle);
  }
#endif

  //DEBUG_PRINTF("FoundGlassMeanDev hat die Werte %f %f %f %f %f %f %f \n\n", FoundGlassMeanDev[0], FoundGlassMeanDev[1],FoundGlassMeanDev[2], FoundGlassMeanDev[3], FoundGlassMeanDev[4], FoundGlassMeanDev[5], FoundGlassMeanDev[6]);



  return;
}





//-----------------------------------------------------------------------------------------------





void ICP_screw_param (Halcon::HTuple X_2, Halcon::HTuple Y_2, Halcon::HTuple Z_2,
    Halcon::HTuple X_1to2, Halcon::HTuple Y_1to2, Halcon::HTuple Z_1to2, Halcon::HTuple *RotX,
    Halcon::HTuple *RotY, Halcon::HTuple *RotZ, Halcon::HTuple *HomMat3D_screw, Halcon::HTuple *RotY_plain, Halcon::HTuple *RotX_plain, Halcon::HTuple *RotZ_plain)
{
  using namespace Halcon;

  // Local control variables
  HTuple  X_2_mean, Y_2_mean, Z_2_mean, X_2_mwfree;
  HTuple  Y_2_mwfree, Z_2_mwfree, X_1to2_mean, Y_1to2_mean;
  HTuple  Z_1to2_mean, X_1to2_mwfree, Y_1to2_mwfree, Z_1to2_mwfree;
  HTuple  Length_XYZ_2, MatrixID_XYZ_2, Length_XYZ_1to2, MatrixID_XYZ_1to2;
  HTuple  Weight, MatrixID_Weights, MatrixTransposedID_W;
  HTuple  MatrixID_2_mw, MatrixID_1to2_mw, MatrixID_2_mw_3xn;
  HTuple  MatrixID_2_dev, MatrixID_1to2_mw_3xn, MatrixID_1to2_dev;
  HTuple  MatrixID_Weights_3xn, MatrixID_2_dev2, MatrixID_1to2_dev2;
  HTuple  MatrixID_2_dev2_T, MatrixID_N, MatrixID_U, MatrixID_S;
  HTuple  MatrixID_V, MatrixID_U_T, MatrixID_R, MatrixID_R_1to2_mw;
  HTuple  MatrixID_T, ValuesR, ValuesT, R02, R35, R68, HomMat3D_1to2_to_2real;
  HTuple  Pose_1to2_to_2real, RotX_icp, RotY_icp, RotZ_icp;

  //free points from centroid
  tuple_mean(X_2, &X_2_mean);
  tuple_mean(Y_2, &Y_2_mean);
  tuple_mean(Z_2, &Z_2_mean);
  tuple_sub(X_2, X_2_mean, &X_2_mwfree);
  tuple_sub(Y_2, Y_2_mean, &Y_2_mwfree);
  tuple_sub(Z_2, Z_2_mean, &Z_2_mwfree);
  tuple_mean(X_1to2, &X_1to2_mean);
  tuple_mean(Y_1to2, &Y_1to2_mean);
  tuple_mean(Z_1to2, &Z_1to2_mean);
  tuple_sub(X_1to2, X_1to2_mean, &X_1to2_mwfree);
  tuple_sub(Y_1to2, Y_1to2_mean, &Y_1to2_mwfree);
  tuple_sub(Z_1to2, Z_1to2_mean, &Z_1to2_mwfree);


  //***********************
  //ICP
  //Calculates the rotational transformation components between two pointclouds
  tuple_length(X_2_mwfree, &Length_XYZ_2);
  create_matrix(3, Length_XYZ_2, ((X_2_mwfree).Concat(Y_2_mwfree)).Concat(Z_2_mwfree),
        &MatrixID_XYZ_2);
  tuple_length(X_1to2_mwfree, &Length_XYZ_1to2);
  create_matrix(3, Length_XYZ_1to2, (X_1to2_mwfree.Concat(Y_1to2_mwfree)).Concat(Z_1to2_mwfree),
      &MatrixID_XYZ_1to2);
  Weight = 1.0/Length_XYZ_2;
  create_matrix(1, Length_XYZ_2, Weight, &MatrixID_Weights);
  //find centroid
  transpose_matrix(MatrixID_Weights, &MatrixTransposedID_W);
  mult_matrix(MatrixID_XYZ_2, MatrixTransposedID_W, "AB", &MatrixID_2_mw);
  mult_matrix(MatrixID_XYZ_1to2, MatrixTransposedID_W, "AB", &MatrixID_1to2_mw);
  //find deviation
  repeat_matrix(MatrixID_2_mw, 1, Length_XYZ_2, &MatrixID_2_mw_3xn);
  sub_matrix(MatrixID_XYZ_2, MatrixID_2_mw_3xn, &MatrixID_2_dev);
  repeat_matrix(MatrixID_1to2_mw, 1, Length_XYZ_2, &MatrixID_1to2_mw_3xn);
  sub_matrix(MatrixID_XYZ_1to2, MatrixID_1to2_mw_3xn, &MatrixID_1to2_dev);
  //apply weights
  repeat_matrix(MatrixID_Weights, 3, 1, &MatrixID_Weights_3xn);
  mult_element_matrix(MatrixID_2_dev, MatrixID_Weights_3xn, &MatrixID_2_dev2);
  mult_element_matrix(MatrixID_1to2_dev, MatrixID_Weights_3xn, &MatrixID_1to2_dev2);
  //create N
  transpose_matrix(MatrixID_2_dev2, &MatrixID_2_dev2_T);
  mult_matrix(MatrixID_1to2_dev, MatrixID_2_dev2_T, "AB", &MatrixID_N);
  svd_matrix(MatrixID_N, "full", "both", &MatrixID_U, &MatrixID_S, &MatrixID_V);
  //Calculate R and T
  transpose_matrix(MatrixID_U, &MatrixID_U_T);
  mult_matrix(MatrixID_V, MatrixID_U_T, "AB", &MatrixID_R);
  mult_matrix(MatrixID_R, MatrixID_1to2_mw, "AB", &MatrixID_R_1to2_mw);
  sub_matrix(MatrixID_2_mw, MatrixID_R_1to2_mw, &MatrixID_T);
  //Create HomMat3D
  get_full_matrix(MatrixID_R, &ValuesR);
  get_full_matrix(MatrixID_T, &ValuesT);
  tuple_select_range(ValuesR, 0, 2, &R02);
  tuple_select_range(ValuesR, 3, 5, &R35);
  tuple_select_range(ValuesR, 6, 8, &R68);
  HomMat3D_1to2_to_2real.Reset();
  HomMat3D_1to2_to_2real.Append(R02);
  HomMat3D_1to2_to_2real.Append(HTuple(ValuesT[0]));
  HomMat3D_1to2_to_2real.Append(R35);
  HomMat3D_1to2_to_2real.Append(HTuple(ValuesT[1]));
  HomMat3D_1to2_to_2real.Append(R68);
  HomMat3D_1to2_to_2real.Append(HTuple(ValuesT[2]));
  //output
  (*HomMat3D_screw) = HomMat3D_1to2_to_2real;
  //calculate angle Threshold
  hom_mat3d_to_pose(HomMat3D_1to2_to_2real, &Pose_1to2_to_2real);
  RotX_icp = Pose_1to2_to_2real[3];
  RotY_icp = Pose_1to2_to_2real[4];
  (*RotY_plain) = RotY_icp;
  (*RotX_plain) = RotX_icp;
  RotZ_icp = Pose_1to2_to_2real[5];
  (*RotZ_plain) = RotZ_icp;
  if (RotX_icp<0)
  {
    RotX_icp = -RotX_icp;
  }
  if (RotX_icp<=180)
  {
    RotX_icp = RotX_icp;
  }
  else if (RotX_icp>180)
  {
    RotX_icp = 360-RotX_icp;
  }
  //
  if (RotY_icp<0)
  {
    RotY_icp = -RotY_icp;
  }
  if (RotY_icp<=180)
  {
    RotY_icp = RotY_icp;
  }
  else if (RotY_icp>180)
  {
    RotY_icp = 360-RotY_icp;
  }
  //
  if (RotZ_icp<0)
  {
    RotZ_icp = -RotZ_icp;
  }
  if (RotZ_icp<=180)
  {
    RotZ_icp = RotZ_icp;
  }
  else if (RotZ_icp>180)
  {
    RotZ_icp = 360-RotZ_icp;
  }
  //
  (*RotX) = RotX_icp;
  (*RotY) = RotY_icp;
  (*RotZ) = RotZ_icp;
  return;
}

//-----------------------------------------------------------------------------------------------





void Searchspace_Reduction (Halcon::HTuple RelPose, Halcon::HTuple CamParam,
	Halcon::Hobject ImageX, Halcon::Hobject ImageY, Halcon::Hobject ImageZ, Halcon::Hobject Candidate,
	Halcon::HTuple *SearchspaceRow, Halcon::HTuple *SearchspaceCol)
{
  using namespace Halcon;

  Halcon::Hobject Region, Rectangle, RectangleDilation;
  Halcon::HTuple HomMat3D, HomMat3DInvert, Rows, Columns, QR, QC;
  Halcon::HTuple Row1, Column1, Row2, Column2, DomainRow, DomainCol;
  Halcon::HTuple XVal, YVal, ZVal, Qx, Qy, Qz, Qx_cal, Qy_cal, Qz_cal;
  Halcon::HTuple Rect_Size;
  Halcon::HTuple Width, Height;
  Halcon::HTuple SelectedQR, SelectedQC, LengthQR;

  DEBUG_PRINTF("Debug: Starting searchspace reduction! \n\n");
  //get X Y Z coordinates and the candidate region
  get_region_points(Candidate, &Rows, &Columns);
  get_grayval(ImageX, Rows, Columns, &XVal);
  get_grayval(ImageY, Rows, Columns, &YVal);
  get_grayval(ImageZ, Rows, Columns, &ZVal);
  //convert the relative pose to a hom_mat3d
  pose_to_hom_mat3d (RelPose, &HomMat3D);
  hom_mat3d_invert (HomMat3D, &HomMat3DInvert);
  //transform the 3D points from view1 to view2
  affine_trans_point_3d(HomMat3DInvert, XVal, YVal, ZVal, &Qx, &Qy, &Qz);
			tuple_select(Qx,1,&SelectedQR);
  			tuple_select(Qy,1,&SelectedQC);
  			DEBUG_PRINTF("Qx = %f Qy = %f /n", SelectedQR[0].D(), SelectedQC[0].D());
  //project the 3D points to obtain the 2D candidate region in view2
  try
  {
	  project_3d_point(Qx, Qy, Qz, CamParam, &QR, &QC);
  }
  catch(Halcon::HException ex)
  {
	  DEBUG_PRINTF("Error: %s\n", ex.message);
	  for(int i = 0; i < Qz.Num(); i++)
	  {
		  if(Qz[i].D() < 0.0001)
		  {
			  Qz[i] = 0.001;
		  }
	  }
  }
  //generate the region and resize it
  tuple_round(QR, &QR);
  tuple_round(QC, &QC);
  gen_region_points (&Region, QR, QC);
  smallest_rectangle1 (Region, &Row1, &Column1, &Row2, &Column2);
  gen_rectangle1 (&Rectangle, Row1, Column1, Row2, Column2);
  //calculate the region size and expand the region into both directions about the same size
  Height[0] = (Row2 - Row1)*2 + 1;
  Width[0] = (Column2 - Column1)*2 + 1;
  DEBUG_PRINTF("Width = %f Height = %f \n\n", Width[0].D(), Height[0].D());
  dilation_rectangle1 (Rectangle, &RectangleDilation, Width, Height);
  get_region_points (RectangleDilation, &DomainRow, &DomainCol);
		/*set_window_attr("background_color","black");
		open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(Candidate, WindowHandle);
		set_window_attr("background_color","black");
		open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(Region, WindowHandle);
		set_window_attr("background_color","black");
		open_window(0,0, 176,144, 0, "","", &WindowHandle);
		set_part(WindowHandle,-1,-1,-1,-1);
		disp_obj(RectangleDilation, WindowHandle);*/

  //generate output
  (*SearchspaceRow) = DomainRow;
  (*SearchspaceCol) = DomainCol;
  return;
}




