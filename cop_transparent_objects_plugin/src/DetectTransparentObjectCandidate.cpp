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


#include "DetectTransparentObjectCandidate.h"
#include "TransparentObjectCandidate.h"
#include "SwissRangerReading.h"
#include "HClusterDetector.h"
#include "SegmentPrototype.h"
#include "BoostUtils.h"

#include <cpp/HalconCpp.h>


#define NORM2(A,B,C) sqrt((A)*(A)+(B)*(B)+(C)*(C))

#ifdef _DEBUG
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF if(false) printf
#endif

using namespace cop;

// Procedure declarations
// Local procedures

DetectTransparentObjectCandidate::DetectTransparentObjectCandidate()
{

}

void DetectTransparentObjectCandidate::SetData(XMLTag* tag)
{
  DEBUG_PRINTF("Loading Algorithm DetectTransparentObjectCandidate\n");
}


DetectTransparentObjectCandidate::~DetectTransparentObjectCandidate(void)
{
  using namespace Halcon;
  clear_all_class_mlp();

}


XMLTag* DetectTransparentObjectCandidate::Save()
{
  XMLTag* tag = new XMLTag(GetName());

  return tag;
}

bool    open1 = false, open2 =  false, open3 = false;
Halcon::HTuple WindowHandle1, WindowHandle2, WindowHandle3;

// Public attribute accessor methods
std::vector<RelPose*> DetectTransparentObjectCandidate::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  using namespace Halcon;
  std::vector<RelPose*> results;
  qualityMeasure = 0;
  Sensor* selected_sensor = NULL;;
  Hobject Intensity1, Distance1, Object_Segmentation1, Object_Segmentation1_Con, Candidate_Sel, ImageX1, ImageY1, ImageZ1, Debug_Region;
  HTuple SRCamParam, HomMat3D_calib, X, Y, Z, I, D, Qx, Qy, Qz, Row, Column, Row_add, Col_add, Row_sgn, Col_sgn, Index_R, Index_C;
  HTuple num_obj, width, height, type, mean, dev, XVals, YVals, ZVals, Xmean, Ymean, Zmean, Last_Set, area, area_row, area_col;

  TransparentObjectCandidate* proto = (TransparentObjectCandidate*)object.GetElement(0, DESCRIPTOR_TRANSPARENTOBJECTCAND);
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      selected_sensor = *it;
      break;
    }
  }
  SwissRangerReading* reading = NULL;
  try
  {
    reading = (SwissRangerReading*)selected_sensor->GetReading(-1);
  }
  catch(const char* text)
  {
    printf("Error in DetectTransparentObjectCandidate: %s (Get data from SR4)\n", text);
    throw text;
  }
  sensor_msgs::PointCloud& pcd = reading->m_image;

  //Generate camera-parameters and calibration-transformation matrix
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
	  DEBUG_PRINTF("View 1: Image acqusition reprojection or affine transformation failed! \n");
	  DEBUG_PRINTF("Fix failed: %s but s_x= %f s_y =%f \n\n", ex.message, SRCamParam[2].D(), SRCamParam[3].D()  );
  }

  try
  {
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
	  gen_image_const(&ImageX1, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&ImageY1, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&ImageZ1, "float", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&Intensity1, "uint2", SRCamParam[6].I(), SRCamParam[7].I());
	  gen_image_const(&Distance1, "float", SRCamParam[6].I(), SRCamParam[7].I());


	  set_grayval(ImageX1, Row.Int(), Column.Int(), X);
	  set_grayval(ImageY1, Row.Int(), Column.Int(), Y);
	  set_grayval(ImageZ1, Row.Int(), Column.Int(), Z);
	  set_grayval(Intensity1, Row.Int(), Column.Int(), I);
	  set_grayval(Distance1, Row.Int(), Column.Int(), D);
  }
  catch(HException ex)
  {
	  DEBUG_PRINTF("Got an exception. Trying to directly acqire images! %s \n", ex.message);
	  for(int i = 0; i < Row.Num(); i++)
	  {
		  if(Row[i].I() >= 0 && Column[i].I() >= 0 &&
				  Row[i].I() < SRCamParam[7].I() + 1 && Column[i].I() < SRCamParam[6].I() + 1)
		  {
			  set_grayval(ImageX1, Row[i].I(), Column[i].I(), X[i]);
			  set_grayval(ImageY1, Row[i].I(), Column[i].I(), Y[i]);
			  set_grayval(ImageZ1, Row[i].I(), Column[i].I(), Z[i]);
			  set_grayval(Intensity1, Row[i].I(), Column[i].I(), I[i]);
			  set_grayval(Distance1, Row[i].I(), Column[i].I(), D[i]);
		  }
		  else
			  DEBUG_PRINTF("Error at: r %d c %d \n x %f y %f  z  %f\n Cam: %d %d (%ld)\n", Row[i].I(), Column[i].I(), X[i].D(), Y[i].D(), Z[i].D(), SRCamParam[6].I(),SRCamParam[7].I(), SRCamParam.Num());
	  }
  }
#ifdef _DEBUG
    printf("Trying to write Images to HDD\n");
    //system("mkdir -p TestOutput");
    try
    {       
      write_image(Intensity1,"tiff",0,"Test_Output/Int1");
      write_image(Distance1,"tiff",0,"Test_Output/Dis1");
      write_image(ImageX1,"tiff",0,"Test_Output/IX1");
      write_image(ImageY1,"tiff",0,"Test_Output/IY1");
      write_image(ImageZ1,"tiff",0,"Test_Output/IZ1");
      DEBUG_PRINTF("Coordinate images for view 1 written to HDD\n\n");
    }
    catch(HException ex)
    {
      DEBUG_PRINTF("Writing to HDD failed failed! %s \n\n",ex.message);
    }
#endif
  try
  {
	  Segmentation_Glasses(Intensity1, Distance1, Row, Column, &Object_Segmentation1);
  }
  catch(HException ex)
  {
	  DEBUG_PRINTF("Segmentation_Glasses failed! %s \n\n",ex.message);
  }

  connection(Object_Segmentation1, &Object_Segmentation1_Con);
  count_obj(Object_Segmentation1_Con, &num_obj);
  DEBUG_PRINTF("num_obj = %d \n\n", num_obj[0].I());
  if(num_obj[0].I() > 0)
  {
	  Last_Set[0] = 0;
	  for(int i=1; i<=min(numOfObjects,num_obj[0].I()); i++)
	  {
		try
		{
			select_obj(Object_Segmentation1_Con, &Candidate_Sel, i);
			area_center(Candidate_Sel, &area, &area_row, &area_col);
			if(area[0].I() == 0)
			{
				continue;
			}
			  //split candidates
			  std::vector<std::pair<Halcon::HTuple, Halcon::HTuple> > Cluster_RC;
			  try
			  {
				  DEBUG_PRINTF("Splitting candidate %d \n\n", i);
				  Segmentation_Glasses_Split(Candidate_Sel, Cluster_RC, SRCamParam);
			  }
			  catch(HException ex)
			  {
				  DEBUG_PRINTF("Split_Candidates failed! %s \n\n",ex.message);
				  continue;
			  }

			  for(size_t k=0; k<Cluster_RC.size(); k++)
			  {
				    HTuple Split_Rows=Cluster_RC[k].first, Split_Columns=Cluster_RC[k].second;

					Hobject Candidate_Sel_split;
					gen_region_points(&Candidate_Sel_split, Split_Rows, Split_Columns);
					concat_obj(Debug_Region, Candidate_Sel_split, &Debug_Region);

					//get_region_points(Candidate_Sel, &Rows, &Columns);
					if(Split_Rows.Num()==0)
					{
						DEBUG_PRINTF("Empty Candidate! Discarding! \n\n");
						continue;
					}
					get_grayval(ImageX1, Split_Rows, Split_Columns, &XVals);
					get_grayval(ImageY1, Split_Rows, Split_Columns, &YVals);
					get_grayval(ImageZ1, Split_Rows, Split_Columns, &ZVals);
					tuple_mean(XVals, &Xmean);
					tuple_mean(YVals, &Ymean);
					tuple_mean(ZVals, &Zmean);
					DEBUG_PRINTF("Got Object's initial position \n\n");

					dev= HTuple();
					dev[0]=dev[1]=dev[2]=dev[3]=dev[4]=dev[5]=0;
					mean=HTuple();
					mean[0]=Xmean[0];
					tuple_concat(mean, Ymean[0], &mean);
					tuple_concat(mean, Zmean[0], &mean);
					tuple_concat(mean, 0, &mean);
					tuple_concat(mean, 0, &mean);
					tuple_concat(mean, 0, &mean);
					tuple_concat(mean, 0, &mean);
					DEBUG_PRINTF("Got mean and dev for TupleToMat \n\n");
					Matrix m(4,4);
					Matrix cov(6,6);
					RelPoseHTuple::TupleToMat(mean, dev, m, cov);

					DEBUG_PRINTF("Reading m_uniqueID %ld \n\n", reading->m_relPose->m_uniqueID);
					cout<<m<<endl<<cov<<endl;
					DEBUG_PRINTF("Out done \n");

					RelPose* Candidate_Pose = NULL;
					try
					{
						Candidate_Pose = RelPoseFactory::FRelPose(reading->m_relPose->m_uniqueID, m, cov);  //TODO 64Bit
					}
					catch(const char* text)
					{
						DEBUG_PRINTF("%s \n", text);
					}

					if(Candidate_Pose != NULL)
					{
						DEBUG_PRINTF("Candidate Pose found \n");
					}
					if(Candidate_Pose == NULL)
					{
						DEBUG_PRINTF("Candidate Pose = NULL\n");
						continue;
					}
					Candidate_Pose = RelPoseFactory::GetRelPose(Candidate_Pose->m_uniqueID, Candidate_Pose->m_parentID);
					if(Candidate_Pose == NULL)
					{
						DEBUG_PRINTF("Candidate Pose = NULL\n");
						continue;
					}
					if(Candidate_Pose != NULL)
					{
						DEBUG_PRINTF("Created Candidate Pose  \n\n");
					}
					//estimate candidate quality by using convexity of the region
					//and the point density
					HTuple convex, density, DevX, DevY, DevZ;
					convexity(Candidate_Sel_split, &convex);
					tuple_deviation(XVals, &DevX);
					tuple_deviation(YVals, &DevY);
					tuple_deviation(ZVals, &DevZ);
					density = Split_Rows.Num()/(DevX*DevY*DevZ);

					DEBUG_PRINTF("Object Density: %f  Object convexity: %f \n\n",density[0].D(), convex[0].D());
					if( convex < 0.5 || density < 1000000)
					{
						DEBUG_PRINTF("Candidate convexity or density too low! \n\n");
						continue;
					}
					qualityMeasure = (convex[0].D()*density[0].D()/50000000);
					Candidate_Pose->m_qualityMeasure = (convex[0].D()*density[0].D()/50000000);

					try
					{
						DEBUG_PRINTF("Saving candidate data. \n\n");
						proto->m_detectedRegions[Candidate_Pose->m_uniqueID] = Candidate_Sel_split;//Object_Segmentation1;
						proto->m_usedIntensities[Candidate_Pose->m_uniqueID] = Intensity1;
						proto->m_usedDistance[Candidate_Pose->m_uniqueID] = Distance1;
						proto->m_usedImageX1[Candidate_Pose->m_uniqueID] = ImageX1;
						proto->m_usedImageY1[Candidate_Pose->m_uniqueID] = ImageY1;
						proto->m_usedImageZ1[Candidate_Pose->m_uniqueID] = ImageZ1;
						if(Last_Set[0].I() == 0)
						{
							proto->SetLastMatchedImage(reading, Candidate_Pose);
							Last_Set[0] = 1;
						}
						DEBUG_PRINTF("pushing back candidate data \n\n");
						results.push_back(Candidate_Pose);
					}
					catch(...)
					{
						DEBUG_PRINTF("push back failed! \n\n");
					}
			  }//end for cluster
#ifdef _DEBUG
				set_window_attr("background_color","black");
				if(!open1)
				{
				  open_window(0,0, 176,144, 0, "","", &WindowHandle1);
				  open1 = true;
				}
				set_part(WindowHandle1,-1,-1,-1,-1);
				set_colored(WindowHandle1, 12);
				disp_obj(Debug_Region, WindowHandle1);
				//HTuple mRow, mCol, mButton;
				//get_mbutton(WindowHandle ,&mRow, &mCol, &mButton);
				write_region(Debug_Region, "Test_Output/Splitting1.tiff");
#endif
		}
		catch(HException ex)
		{
			DEBUG_PRINTF("%s \n",ex.message);
		}
		catch(const char* text)
		{
			DEBUG_PRINTF("%s \n", text);
		}
		catch(...)
		{
			DEBUG_PRINTF("Error returning candidates \n");
		}



	   }//end for numObjects

	  numOfObjects = results.size();
  }
  reading->Free();
  return results;
}

double DetectTransparentObjectCandidate::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{

	DEBUG_PRINTF("DetectTransparentObjectCandidate::CheckSignature \n\n");


  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {

    	DEBUG_PRINTF("SwissRanger found \n\n");

      if(object.GetElement(0, DESCRIPTOR_TRANSPARENTOBJECTCAND) != NULL )
      {
    	  DEBUG_PRINTF("Transparent Object Candidate found! \n\n");
        return 0.5;
      }
      else
        return 0.0;
    }
  }
  return 0.0;
}


// Local procedures
void Segmentation_Glasses (Halcon::Hobject IntensityImage, Halcon::Hobject DistanceImage, Halcon::HTuple DomainRow,
		Halcon::HTuple DomainCol, Halcon::Hobject *CandidateRegion)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  IntEquHisto1, VarThresh, VarThresh1, VarThresh2, RegionUnion, CloseRange1;
  Hobject  CloseGlasses1, IntZoomed1, IntZoomed2, VarZoomed, VarZoomed1, VarZoomed2, ZoomedOpening1;
  Hobject  ZoomedClosing1, ZoomedDilation1, DeZoomedGlasses1, ZoomedConnected, ZoomedRegionTrans, ZoomedRegionUnion, ZoomedSelected;
  Hobject  GlassesRegion1, Glasses_Con1, SelectedGlasses1;
  Hobject  ReUnion1, ObjectSelected1, ClosingObj1, FillUpObj1;
  Hobject  DilationObj1, ReUnion1_Con1, SelectedRegions1, CandidateRegion1;
  Hobject  GlassObjSelected1, GlassObjClosing1, GlassObjFilled1;
  Hobject  GlassObjDil1, CloseRange_compl, CandidateDomain, IntensityImage_Reduced, FullDomain, RegionErosionCon;
  Hobject  RegionClosing, ConnectedRegions, RegionErosion, RegionTrans, RegionTrans1, RegionDilation, SelectedRegions, RegionIntersection;
  Hobject  ConnDiameter, SelDiameter;
  // Local control variables
  HTuple  ZoomHeight1, ZoomWidth1, Number, i, Row, Col, grayvals, grayvals_mean;

  //create domain region
  threshold(DistanceImage, &CloseRange1, 0.2, 1.800);
  //gen_region_points(&FullDomain, DomainRow, DomainCol);
  //intersection(CloseRange1, FullDomain, &CloseRange1);
  /*full_domain(IntensityImage, &FullDomain);*/
  //get_region_points(FullDomain, &Row, &Col);
  //intersection(CandidateDomain, FullDomain, &CandidateDomain);
#ifdef _DEBUG
	  set_window_attr("background_color","black");
	  if(!open2)
	  {
	    open_window(0,0, 176,144, 0, "","", &WindowHandle2);
	    open2 = true;
	  }
	  set_part(WindowHandle2,-1,-1,-1,-1);
	  disp_obj(CloseRange1, WindowHandle2);
      write_region(CloseRange1, "Test_Output/CloseRange.tiff");
#endif

  //Equally distribute grayvalues
  equ_histo_image(IntensityImage, &IntEquHisto1);
  //reduce_domain(IntEquHisto1, CandidateDomain, &IntensityImage_Reduced);

  //average image grayvalue
  get_region_points(CloseRange1, &Row, &Col);
  get_grayval(IntensityImage, Row, Col, &grayvals);
  tuple_median(grayvals, &grayvals_mean);
  //Reduce ROI1 to dark areas
  if(grayvals_mean > 4000)
  {
	  var_threshold(IntEquHisto1, &VarThresh1, 41, 41, 0.1, 5000, "dark");
	  DEBUG_PRINTF("Environment is bright! %f \n\n", grayvals_mean[0].D());
	  //Reduce ROI1 to close Distance
	  intersection(VarThresh1, CloseRange1, &CloseGlasses1);
  }
  else
  {
	  /*var_threshold (IntEquHisto1, &VarThresh2, 15, 15, 0.1, 6000, "dark");
	  var_threshold(IntEquHisto1, &VarThresh1, 41, 41, 0.1, 15000, "light");
	  union2(VarThresh1, VarThresh2, &VarThresh1);
	  opening_circle (VarThresh1, &VarThresh1, 1);*/
	  var_threshold (IntensityImage, &VarThresh, 5, 5, 0.1, 200, "dark");
	  connection (VarThresh, &ConnDiameter);
	  select_shape (ConnDiameter, &SelDiameter, "max_diameter", "and", 0, 100);
	  union1(SelDiameter, &VarThresh);

	  var_threshold (IntEquHisto1, &VarThresh1, 31, 31, 0.2, 10000, "dark");
	  connection (VarThresh1, &ConnDiameter);
	  select_shape (ConnDiameter, &SelDiameter, "max_diameter", "and", 0, 100);
	  union1(SelDiameter, &VarThresh1);

	  var_threshold (IntEquHisto1, &VarThresh2, 41, 41, 0.1, 15000, "light");
	  union2 (VarThresh, VarThresh1, &RegionUnion);
	  union2 (RegionUnion, VarThresh2, &RegionUnion);
	  DEBUG_PRINTF("Environment is dark! \n\n");
	  //Reduce ROI1 to close Distance
	  intersection(RegionUnion, CloseRange1, &CloseGlasses1);
  }

#ifdef _DEBUG
	  /*set_window_attr("background_color","black");
	  open_window(0,0, 176,144, 0, "","", &WindowHandle);
	  set_part(WindowHandle,-1,-1,-1,-1);
	  disp_obj(CloseGlasses1, WindowHandle);*/
#endif

  //Reduce ROI1 to dark areas that also occur in
  //higher Pyramid level
  ZoomHeight1 = 0.8;
  ZoomWidth1 = 0.8;
  zoom_image_factor(IntEquHisto1, &IntZoomed1, ZoomWidth1, ZoomHeight1, "constant");
  if(grayvals_mean > 4000)
  {
	  var_threshold(IntZoomed1, &VarZoomed1, 41, 41, 0.3, 5000, "dark");
	  DEBUG_PRINTF("Environment is bright! \n\n");

	  opening_circle(VarZoomed1, &ZoomedOpening1, 1);
	  closing_circle(ZoomedOpening1, &ZoomedClosing1, 4);
	  dilation_circle(ZoomedClosing1, &ZoomedDilation1, 1.5);
	  zoom_region(ZoomedDilation1, &DeZoomedGlasses1, 1/ZoomWidth1, 1/ZoomHeight1);
	  intersection(DeZoomedGlasses1, CloseGlasses1, &GlassesRegion1);
  }
  else
  {
	  /*var_threshold(IntZoomed1, &VarZoomed1, 41, 41, 0.1    0.3, 10000, "light");
	  var_threshold (IntZoomed1, &VarZoomed2, 15, 15, 0.1, 6000, "dark");
	  union2 (VarZoomed1, VarZoomed2, &VarZoomed1);
	  DEBUG_PRINTF("Environment is dark! \n\n");*/

	  zoom_image_factor (IntensityImage, &IntZoomed2, ZoomWidth1, ZoomHeight1, "constant");
	  var_threshold (IntZoomed2, &VarZoomed, 5, 5, 0.1, 200, "dark");
	  connection (VarZoomed, &ConnDiameter);
	  select_shape (ConnDiameter, &SelDiameter, "max_diameter", "and", 0, 100);
	  union1(SelDiameter, &VarZoomed);

	  var_threshold (IntZoomed1, &VarZoomed1, 31, 31, 0.2, 10000, "dark");
	  connection (VarZoomed1, &ConnDiameter);
	  select_shape (ConnDiameter, &SelDiameter, "max_diameter", "and", 0, 100);
	  union1(SelDiameter, &VarZoomed1);

	  var_threshold (IntZoomed1, &VarZoomed2, 41, 41, 0.1, 15000, "light");
	  union2 (VarZoomed, VarZoomed1, &VarZoomed1);
	  union2 (VarZoomed1, VarZoomed2, &VarZoomed2);
	  closing_circle (VarZoomed2, &ZoomedClosing1, 2);
	  opening_circle (ZoomedClosing1, &ZoomedOpening1, 2);
	  connection (ZoomedOpening1, &ZoomedConnected);
	  shape_trans (ZoomedConnected, &ZoomedRegionTrans, "convex");
	  select_shape(ZoomedRegionTrans, &ZoomedSelected, "area", "and", 10, 5000);
	  dilation_circle (ZoomedSelected, &ZoomedDilation1, 2);
	  union1 (ZoomedDilation1, &ZoomedRegionUnion);
	  zoom_region (ZoomedRegionUnion, &DeZoomedGlasses1, 1/ZoomWidth1, 1/ZoomHeight1);
	  intersection (DeZoomedGlasses1, CloseGlasses1, &GlassesRegion1);

	  closing_circle (GlassesRegion1, &RegionClosing, 2);
	  connection (RegionClosing, &ConnectedRegions);
	  erosion_circle (ConnectedRegions, &RegionErosion, 4);
	  union1(RegionErosion, &RegionErosion);
	  connection (RegionErosion, &RegionErosionCon);
	  shape_trans (RegionErosionCon, &RegionTrans, "convex");
	  dilation_circle (RegionTrans, &RegionDilation, 4);
	  select_shape (RegionDilation, &SelectedRegions, "area", "and", 50, 5000);
	  intersection (SelectedRegions, GlassesRegion1, &RegionIntersection);
	  shape_trans (RegionIntersection, &RegionTrans1, "convex");
	  union1(RegionTrans1, &GlassesRegion1);
  }
#ifdef _DEBUG
	  //set_window_attr("background_color","black");
	  //open_window(0,0, 176,144, 0, "","", &WindowHandle);
	  //set_part(WindowHandle,-1,-1,-1,-1);
	  //disp_obj(GlassesRegion1, WindowHandle);
#endif

  //Resize every single Object
  connection(GlassesRegion1, &Glasses_Con1);
  select_shape(Glasses_Con1, &SelectedGlasses1, "area", "and", 50, 10000);
  count_obj(SelectedGlasses1, &Number);
  gen_empty_region(&ReUnion1);
  for (i=1; i<=Number; i+=1)
  {
    select_obj(SelectedGlasses1, &ObjectSelected1, i);
    closing_circle(ObjectSelected1, &ClosingObj1, 2);
    fill_up(ClosingObj1, &FillUpObj1);
    dilation_rectangle1(FillUpObj1, &DilationObj1, 2, 2);
    union2(DilationObj1, ReUnion1, &ReUnion1);
  }
#ifdef _DEBUG
	  set_window_attr("background_color","black");
	  if(!open3)
	  {
	    open_window(0,0, 176,144, 0, "","", &WindowHandle3);
	    open3 = true;
	  }
	  set_part(WindowHandle3,-1,-1,-1,-1);
	  disp_obj(ReUnion1, WindowHandle3);
#endif

  //Resize every object
  connection(ReUnion1, &ReUnion1_Con1);
  select_shape(ReUnion1_Con1, &SelectedRegions1, "area", "and", 100, 99999);
  count_obj(SelectedRegions1, &Number);
  gen_empty_region(&CandidateRegion1);
  for (i=1; i<=Number; i+=1)
  {
    select_obj(SelectedRegions1, &GlassObjSelected1, i);
    closing_circle(GlassObjSelected1, &GlassObjClosing1, 2);
    fill_up(GlassObjClosing1, &GlassObjFilled1);
    union2(GlassObjFilled1, CandidateRegion1, &CandidateRegion1);
  }
#ifdef _DEBUG
	  set_window_attr("background_color","black");
	  if(!open1)
	  {
	    open_window(0,0, 176,144, 0, "","", &WindowHandle1);
	    open1 = true;
	  }
	  set_part(WindowHandle1,-1,-1,-1,-1);
	  disp_obj(CandidateRegion1, WindowHandle1);
#endif
  copy_obj(CandidateRegion1, &(*CandidateRegion), 1, 1);
  return;
}


//------------------------------------------------------------------------------------------------------
void Segmentation_Glasses_Split(Halcon::Hobject CandidateRegion, std::vector<std::pair<Halcon::HTuple, Halcon::HTuple> > &Cluster_RC, Halcon::HTuple CamParam)
{
	using namespace Halcon;

  Halcon::Hobject CandidateRegion_Con, ObjectSelected, ImageX, ImageY, ImageZ;
  Halcon::HTuple Number, index, Rows, Columns, Yreg, MeanY, Xreg, MeanX, Zreg, MeanZ, SizeCluster;
  Halcon::HTuple Mean_X, Mean_Y, Mean_Z, ROW, COL, Z;
  double min_object_size_for_split = 12;
  int test_near_temp;

	  //generate Images with Row and Col values as grayvalues
	  for (int m=0; m<CamParam[6].I(); m++)
	  {
		  tuple_concat(COL, m, &COL);
	  }

	  gen_image_const(&ImageX, "float", CamParam[6].I(), CamParam[7].I());
	  for (int m=0; m<CamParam[7].I(); m++)
	  {
		  tuple_gen_const(CamParam[6].I(), m, &ROW);
		  set_grayval(ImageX, ROW, COL, COL);
	  }
	  gen_image_const(&ImageY, "float", CamParam[6].I(), CamParam[7].I());
	  for (int m=0; m<CamParam[7].I(); m++)
	  {
		  tuple_gen_const(CamParam[6].I(), m, &ROW);
		  set_grayval(ImageY, ROW, COL, ROW);
	  }
	  gen_image_const(&ImageZ, "float", CamParam[6].I(), CamParam[7].I());
	  tuple_gen_const(CamParam[6].I(), 0, &Z);
	  for (int m=0; m<CamParam[7].I(); m++)
	  {
		  tuple_gen_const (CamParam[6].I(), m, &ROW);
	   	  set_grayval (ImageZ, ROW, COL, Z);
	  }

	  connection(CandidateRegion, &CandidateRegion_Con);
	  count_obj(CandidateRegion_Con, &Number);
	  DEBUG_PRINTF("\n\n%d regions to search for clusters\n\n", Number[0].I());
	      for (index=1; index<=Number[0].I(); index+=1)
	      {
	    	  //for every Candidate from the Segmentation routine
				DEBUG_PRINTF("enter region %d\n", index[0].I());
				select_obj(CandidateRegion_Con, &ObjectSelected, index);
				get_region_points(ObjectSelected, &Rows, &Columns);
				get_grayval(ImageY, Rows, Columns, &Yreg);
				tuple_mean(Yreg, &MeanY);

				HTuple max, histo, histo_abs, split_y, size_y;

				//find maxima and check their environment
				tuple_max(Yreg - MeanY, &max);
				if(max > 2*min_object_size_for_split)
				{
				  DEBUG_PRINTF("Trying split in Y\n");
				  splitDim(ImageY, ObjectSelected, MeanY, min_object_size_for_split, split_y, size_y);
				}
				else
				  split_y = MeanY;

				for(int ysplit = 0; ysplit < split_y.Num(); ysplit++)
				{
				  HTuple RowY, ColY;
				  double lower_bound = -1000.0;
				  double upper_bound = 1000.0;

				  if(ysplit != split_y.Num() -1)
				  {
				   upper_bound = (split_y[ysplit + 1] * size_y[ysplit + 1] +
										  split_y[ysplit] * size_y[ysplit])
										  / (size_y[ysplit + 1] + size_y[ysplit]);
				  }
				  if(ysplit != 0)
				  {
					lower_bound =  (split_y[ysplit] * size_y[ysplit] +
														 split_y[ysplit - 1] * size_y[ysplit - 1])
													  / (size_y[ysplit - 1] + size_y[ysplit]) ;
				  }
				  for (int rows = 0; rows < Rows.Num(); rows++)
				  {
					if( lower_bound < Yreg[rows].D() &&  Yreg[rows].D() <  upper_bound && Yreg[rows].D() != 0.0)
					{
					  RowY.Append(Rows[rows]);
					  ColY.Append(Columns[rows]);
					}
				  }
				  DEBUG_PRINTF("ysplit %d left %ld / %ld\n", ysplit, RowY.Num(), Rows.Num());
				  DEBUG_PRINTF("decision y: ");
				  DEBUG_PRINTF( "%f < y < %f\n", lower_bound, upper_bound);
				  if( RowY.Num() < 100)
				  {
					  DEBUG_PRINTF("Reject this region as too small\n");
					  continue;
				  }
				  HTuple split_x, size_x;
				  HTuple RowXY, ColXY;

				  get_grayval(ImageX, RowY, ColY, &Xreg);
				  tuple_mean(Xreg, &MeanX);
				  tuple_max(Xreg - MeanX, &max);

				  if(max > 2*min_object_size_for_split)
				  {
					DEBUG_PRINTF("Trying split in X\n");
					splitDim(ImageX, ObjectSelected, MeanX, min_object_size_for_split, split_x, size_x);
				  }
				  else
					split_x = MeanY;
				  for(int xsplit = 0; xsplit < split_x.Num(); xsplit++)
				  {
					RowXY = HTuple();
					ColXY = HTuple();
					double lower_bound_x = -1000.0;
					double upper_bound_x = 1000.0;
					if(xsplit != split_x.Num() -1)
					{
					 upper_bound_x = (split_x[xsplit + 1] * size_x[xsplit + 1] +
											split_x[xsplit] * size_x[xsplit])
											/ (size_x[xsplit + 1] + size_x[xsplit]);
					}
					if(xsplit != 0)
					{
					  lower_bound_x =  (split_x[xsplit] * size_x[xsplit] +
														   split_x[xsplit - 1] * size_x[xsplit - 1])
														/ (size_x[xsplit - 1] + size_x[xsplit]) ;
					}


					for (int rows = 0; rows < RowY.Num(); rows++)
					{
					  if(lower_bound_x < Xreg[rows].D() && Xreg[rows].D() < upper_bound_x && Xreg[rows].D() != 0.0)
					  {
						RowXY.Append(RowY[rows]);
						ColXY.Append(ColY[rows]);
					  }
					}
					DEBUG_PRINTF("xsplit: %d () xsplit: %d size: %ld / %ld\n", xsplit, xsplit, RowXY.Num(),  RowY.Num());
					DEBUG_PRINTF("decision x: ");
					DEBUG_PRINTF( "%f < x < %f\n",lower_bound_x,upper_bound_x);

					if(RowXY.Num() < 100)
					{
					  DEBUG_PRINTF("Reject this region as too small (2nd split)\n");
					  continue;
					}
					HTuple YregInner, XregInner, z_off, x_off, y_off, mean_x_inner, mean_y_inner;
					get_grayval(ImageX, RowXY, ColXY, &XregInner);
					get_grayval(ImageY, RowXY, ColXY, &YregInner);
					get_grayval(ImageZ, RowXY, ColXY, &Zreg);

					tuple_mean(XregInner, &mean_x_inner);
					tuple_mean(YregInner, &mean_y_inner);
					tuple_mean(Zreg, &MeanZ);

					z_off = Zreg - MeanZ;
					y_off = YregInner - mean_y_inner;
					x_off = XregInner - mean_x_inner;
					tuple_abs(x_off, &x_off);
					tuple_abs(y_off, &y_off);
					tuple_abs(z_off, &z_off);
					HTuple x_off_sorted, y_off_sorted, z_off_sorted, indices;
					tuple_sort_index(x_off, &x_off_sorted);
					tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.05).Int(), HTuple(x_off_sorted.Num()*0.95).Int(), &x_off_sorted);

					tuple_sort_index(y_off, &y_off_sorted);
					tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.05).Int(), HTuple(y_off_sorted.Num()*0.95).Int(), &y_off_sorted);

					tuple_sort_index(z_off, &z_off_sorted);
					tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.05).Int(), HTuple(z_off_sorted.Num()*0.95).Int(), &z_off_sorted);

					tuple_concat(x_off_sorted,y_off_sorted, &indices);
					tuple_concat(indices,z_off_sorted, &indices);

					tuple_select(x_off, indices, &x_off);
					tuple_select(y_off, indices, &y_off);
					tuple_select(z_off, indices, &z_off);

					//Test if there is already a similar cluster
					//This can happen if the axis aligned split cut an object in half
					bool rejected = false;
					for(int test_near = 0; test_near < Mean_X.Num(); test_near++)
					{
					  if(NORM2((Mean_X[test_near].D() - mean_x_inner[0].D()),
							   (Mean_Y[test_near].D() - mean_y_inner[0].D()),
							  (Mean_Z[test_near].D() - MeanZ[0].D())) <
							  2*min_object_size_for_split)
					  {
						Mean_X[test_near] = (Mean_X[test_near].D()*(SizeCluster[test_near].I())
											   + mean_x_inner*x_off.Num())
												/ (SizeCluster[test_near].I()+x_off.Num());
						Mean_Y[test_near] = (Mean_Y[test_near].D()*(SizeCluster[test_near].I())
											   + mean_y_inner*x_off.Num())
												/ (SizeCluster[test_near].I()+x_off.Num());
						Mean_Z[test_near] = (Mean_Z[test_near].D()*(SizeCluster[test_near].I())
											   + MeanZ*x_off.Num())
												/ (SizeCluster[test_near].I()+x_off.Num());
						SizeCluster[test_near] = SizeCluster[test_near].I() + x_off.Num();
						DEBUG_PRINTF("Fused two clusters, which were splitted before\n");
						Cluster_RC[test_near].first.Append(RowXY);
						Cluster_RC[test_near].second.Append(ColXY);

						rejected = true;
						test_near_temp = test_near;
						break;
					  }
					  DEBUG_PRINTF("Accept distance %f between %d and new mean (%f %f %f)\n", NORM2((Mean_X[test_near].D() - mean_x_inner[0].D()),
										   (Mean_Y[test_near].D() - mean_y_inner[0].D()),
															   (Mean_Z[test_near].D() - MeanZ[0].D())),
															   test_near, mean_x_inner[0].D(),
															   mean_y_inner[0].D(),  MeanZ[0].D());
					}//end for
					if(rejected)
					continue;

					//save data
					std::pair<HTuple, HTuple> New_Cluster_RC;
					New_Cluster_RC.first = RowXY;
					New_Cluster_RC.second = ColXY;
					Cluster_RC.push_back(New_Cluster_RC);
					Mean_X.Append(mean_x_inner);
					Mean_Y.Append(mean_y_inner);
					Mean_Z.Append(MeanZ);
					SizeCluster.Append(x_off.Num());

				  }//for xsplit
				}//for ysplit

	      }//for candidates

	return;
}


//------------------------------------------------------------------------------------------------------

void splitDim(Halcon::Hobject Image, Halcon::Hobject Region, const Halcon::HTuple &Mean, double min_object_size_for_split, Halcon::HTuple &split_x, Halcon::HTuple &size_x)
{
   using namespace Halcon;
  HTuple histo_abs, histo, Min1, Max1, Range, Min, Max, Function, SmoothedFunction, Y1;

    gray_histo(Region, Image, &histo_abs, &histo);
    min_max_gray(Region, Image, 0, &Min1, &Max1, &Range);
    create_funct_1d_array(histo, &Function);
    smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);

    local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
    get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
    DEBUG_PRINTF("%ld maxima in split dimension\n", Y1.Num());
    if(Y1.Num() > 0)
    {
      double curr = (Min1+((HTuple(Max[0])*(Max1-Min1))/256))[0].D();
      split_x = curr;
      double curr_scale = Y1[0].D();
      DEBUG_PRINTF("Curr center in split dimension: %f\n", curr);
      for(int j = 0; j < Y1.Num() - 1; j++)
      {
        double next = (Min1+((HTuple(Max[j+1])*(Max1-Min1))/256))[0].D();
        if(fabs(curr - next) > min_object_size_for_split)
        {
          split_x[split_x.Num() - 1] = split_x[split_x.Num() - 1].D();
          split_x.Append(next);
          size_x.Append(curr_scale);
          curr_scale = Y1[j+1].D();
          DEBUG_PRINTF("Additional center in in split dimension: %f\n", next);
        }
        else
        {
          split_x[split_x.Num() - 1] = (split_x[split_x.Num() - 1].D() + next) / 2;
          curr_scale += Y1[j+1].D();

        }
        curr = next;
      }
      size_x.Append(curr_scale);
    }
    else
    {
      split_x = Mean;
      size_x = 1.0;
    }

}
