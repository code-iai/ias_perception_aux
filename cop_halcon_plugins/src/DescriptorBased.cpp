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


/************************************************************************
                        DescriptorBased.cpp - Copyright klank


**************************************************************************/
#ifdef DESCRIPTOR_AVAILABLE
#include "DescriptorBased.h"
#include "XMLTag.h"
#include "PointDescrModel.h"
#include "Camera.h"
#include "ShapeModel.h"


#include "cpp/HalconCpp.h"
#include "HCPPdescriptor3d.h"
//#include "HCPPplanar_pose_estimation.h" /*< Removed cause code is very closed source*/
using namespace Halcon;


using namespace cop;


// Constructors/Destructors
//

DescriptorBased::DescriptorBased () :
	LocateAlgorithm (),
	m_row(NULL),
	m_col(NULL)

	{
}

DescriptorBased::DescriptorBased (XMLTag* tag) :
	LocateAlgorithm (),
	m_row(NULL),
	m_col(NULL)
{
}


DescriptorBased::~DescriptorBased ( ) { }

//
// Methods
//
XMLTag* DescriptorBased::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_DESCRIPTORBASEDALG);
	return tag;
}

void HCheckDoubleRandRecMy(int* q, int i, int maxcompares)
{
	int k = 0;
	for(; k < maxcompares; k++)
	{
		if(q[i] == q[k])
		{
			q[i]++;
			HCheckDoubleRandRecMy(q, i, k);
		}
	}
}

void HMakeDoubleRandMy(int* q, int numRands, int numMax)
{
	int i = 1;
	for(int j = 0; j < numRands; j++)
	{
		q[j] = (int)(((double)rand()/RAND_MAX)*(numMax - (j+1)));
	}
	for(; i < numRands; i++)
	{
		HCheckDoubleRandRecMy(q, i, i);
	}
}


#define NUM_OF_RAND_POINTS 5
Halcon::HTuple RansacPose(Halcon::HTuple sx, Halcon::HTuple sy, Halcon::HTuple sz, Halcon::HTuple dx, Halcon::HTuple dy, Halcon::HTuple camparam)
{
	int q[NUM_OF_RAND_POINTS];
	int numofRands = NUM_OF_RAND_POINTS;
	int score = NUM_OF_RAND_POINTS;
	Halcon::HTuple final;
	for(int i = 0; i < 1000; i++)
	{
		Halcon::HTuple pose, hommat3d;
		Halcon::HTuple sxSel, sySel, szSel, dxSel,dySel, x,y,z, row, col, sxSel2, sySel2, szSel2, dxSel2,dySel2;
		/*Disable double selections*/
		HMakeDoubleRandMy(q, numofRands, sx.Num());
		int tempscore = 0;
		for(int j = 0; j< numofRands; j++)
		{
			sxSel.Append(sx[q[j]].D());
			sySel.Append(sy[q[j]].D());
			szSel.Append(sz[q[j]].D());
			dxSel.Append(dx[q[j]].D());
			dySel.Append(dy[q[j]].D());
		}
		try
		{
			//Halcon::vector_to_pose(sxSel, sySel, szSel, dxSel,dySel, camparam, "schweighofer", &pose);
		}
		catch(Halcon::HException ex)
		{
			printf("Error: %s\n", ex.message);
			continue;
		}
		Halcon::pose_to_hom_mat3d(pose, &hommat3d);
		Halcon::affine_trans_point_3d(hommat3d, sx,sy,sz, &x, &y, &z);
		Halcon::project_3d_point(x, y, z, camparam, &row, &col);
		for(int j = 0; j < dx.Num(); j++)
		{
			double distr = fabs(row[j].D() - dy[j].D());
			if(distr < 2)
			{
				double distc = fabs(col[j].D() - dx[j].D());
				if(distc < 2)
				{
					sxSel2.Append(sx[j].D());
					sySel2.Append(sy[j].D());
					szSel2.Append(sz[j].D());
					dxSel2.Append(dx[j].D());
					dySel2.Append(dy[j].D());
					tempscore++;
				}
			}
		}
		if(tempscore > score)
		{
			score = tempscore;
			final = pose;
			try
			{
				//Halcon::vector_to_pose(sxSel2, sySel2, szSel2, dxSel2,dySel2, camparam, "schweighofer", &pose);
			}
			catch(Halcon::HException ex)
			{
				printf("Error: %s\n", ex.message);
				continue;
			}
			Halcon::pose_to_hom_mat3d(pose, &hommat3d);
			Halcon::affine_trans_point_3d(hommat3d, sx,sy,sz, &x, &y, &z);
			Halcon::project_3d_point(x, y, z, camparam, &row, &col);
			for(int j = 0; j < dx.Num(); j++)
			{
				double distr = fabs(row[j].D() - dy[j].D());
				if(distr < 2)
				{
					double distc = fabs(col[j].D() - dx[j].D());
					if(distc < 2)
					{
						tempscore++;
					}
				}
			}
		}
	}
	return final;
}


std::vector<RelPose*> DescriptorBased::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose,
Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
	if(cam.size() > 0)
	{
		Image* img = cam[0]->GetImage(-1);
		RelPose* camPose = cam[0]->m_relPose;
		Calibration* calib = &(cam[0]->m_calibration);
        if(img != NULL && camPose != NULL && calib  != NULL)
        {
		    return Inner(img, camPose, calib, lastKnownPose, object, numOfObjects, qualityMeasure);
        }
	}
	return result;
}

std::vector<RelPose*> DescriptorBased::Inner(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
	HTuple camparam = calib->CamParam();
	HTuple cammatrix = calib->CamMatrix();

	if(img->GetType() == HALCONIMAGE)
	{
		bool trackPossible = TrackingPossible(*img, object, lastKnownPose);
		int n = 4;
		double Partly = 1.0 - (0.3* (n-1));
		HTuple empty;
		HTuple  matches,
				xDescriptorSource,yDescriptorSource,zDescriptorSource,
                rowDescriptorTarget,colDescriptorTarget,
                xFinalSource,yFinalSource,zFinalSource,
                rowFinalTarget,colFinalTarget;
		printf("Get Descriptor Model\n");
		PointDescrModel* sm = (PointDescrModel*)(object.GetElement(0, DESCRIPTOR_FEATURE ));
		Halcon::HTuple hommat, pose, cov;
		try
		{
			Hobject* imgs = img->GetHImage();
			int handle = sm->GetDescriptorHandle();
			Halcon::find_descriptor_model_3d(*imgs, handle,
										empty, empty, empty, empty,
										42, 1, cammatrix,
										&hommat,&pose, &cov, &matches,
										&xDescriptorSource,&yDescriptorSource,&zDescriptorSource,
										&rowDescriptorTarget,&colDescriptorTarget,
										&xFinalSource,&yFinalSource,&zFinalSource,
										&rowFinalTarget,&colFinalTarget);
			//printf("Descriptor Target\n");
			img->Free();
			//pose = RansacPose(xDescriptorSource, yDescriptorSource, zDescriptorSource, rowDescriptorTarget, colDescriptorTarget, camparam);
			/*printf("Point-correspondences after Ransac (blue)\n");
			for(int i = 0; i < rowFinalTarget.Num(); i++)
			{
				printf("2d: (%f, %f) -> 3d (%f, %f, %f)\n", rowFinalTarget[i].D(), colFinalTarget[i].D(), xFinalSource[i].D(),yFinalSource[i].D(),zFinalSource[i].D());
			}*/
			if(matches.Num() > 0 && matches[0].ValType() == 1)
			{
				//Halcon::convert_pose_type(pose,"Rp+T", "gba", "coordinate_system",&pose);
				printf("\n");
				printf("Estimated Pose:\n");
				for(int i = 0; i < pose.Num() -1; i++)
				{
					printf("%f, ", pose[i].D());
				}
				printf("%d\n", pose[pose.Num() -1].I());
				printf("\n");
				if(object.GetObjectPose() != NULL && object.GetObjectPose()->m_uniqueID != ID_WORLD)
                                  result.push_back(RelPoseFactory::FRelPose(pose, cov, camPose, object.GetObjectPose()->m_uniqueID));
				else
                                  result.push_back(RelPoseFactory::FRelPose(pose, cov, camPose));
				numOfObjects = matches[0].I();
				qualityMeasure = 0.99;
                                sm->SetLastMatchedImage(img, result[0]);
			}
			else
			{
			   img->Free();
			   numOfObjects = 0;
			   qualityMeasure = 0.99;
		           if(object.GetObjectPose() != NULL)
                            result.push_back(object.GetObjectPose());
			}
			return result;

		}
		catch(Halcon::HException ex)
		{
			printf("Error  in DescriptorBased: %s\n", ex.message);
			qualityMeasure = 0.0;
		}
	}
	return result;
}


double DescriptorBased::CheckSignature(Signature& Object)
{
	if(Object.GetElement(0,DESCRIPTOR_FEATURE))
		return 1.0;
	else
		return 0.0;
}

void DescriptorBased::Show(Camera* cam)
{

	try
	{
	if(m_row != NULL && m_col != NULL)
	{
		static int i = 6;
		Halcon::HWindow* win =  cam->GetWindow();
		win->DispCross(*m_row, *m_col, i, 0);
		i--;
		if (i < 2)
			i = 8;
	}
	}
	catch(Halcon::HException ex)
	{
		printf("Showing not possible: %s\n", ex.message);
	}

}
#endif /*DESCRIPTOR_AVAILABLE*/
