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


#include "PRAShapeVsPointDescr.h"
#include "PointDescrModel.h"
#include "ShapeModel.h"
#include "ShapeBased3D.h"
#include "DescriptorBased.h"
#include "XMLTag.h"
#include "RelPoseFactory.h"

using namespace cop;

PRAShapeVsPointDescr::PRAShapeVsPointDescr(void)
{
}
PRAShapeVsPointDescr::PRAShapeVsPointDescr(XMLTag*)
{
}

PRAShapeVsPointDescr::~PRAShapeVsPointDescr(void)
{
}

ImprovedPose PRAShapeVsPointDescr::Perform(std::vector<Sensor*> cam, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
#ifdef DESCRIPTOR_AVAILABLE
	PointDescrModel* p1 =  (PointDescrModel*)sig.GetElement(0,DESCRIPTOR_FEATURE);
	ShapeModel* p2 =  (ShapeModel*)sig.GetElement(0,DESCRIPTOR_SHAPE);
	//TODO hinder this algorithm of changing the signature at all..
	Image* img = NULL;
	RelPose* pose1 = NULL;
  std::vector<RelPose*> pose2;

	if(p1->date() > p2->date() && p1->GetLastMatchedImage() != NULL)
	{
		img = p1->GetLastMatchedImage();
		Calibration* calib = p1->GetCurCalibration();
  try
  {
     pose1 = RelPoseFactory::CloneRelPose(p1->GetLastMatchedPose());
  }
  catch(...)
  {
    printf("Tying to copy a singular position\n");
    pose1 = RelPoseFactory::FRelPoseWorld();
  }

        if(img != NULL && pose1 != NULL && calib  != NULL)
        {
		    img->Hold();
		    ShapeBased3D shape3d;
        pose2 = shape3d.Inner(img, RelPoseFactory::FRelPose(pose1->m_parentID) , calib, pose, sig, numOfObjects, qualityMeasure);
        }
	}
	else
	{
		img  = p2->GetLastMatchedImage();
        if(img == NULL)
          throw "Nothing to Evaluate";
        ShapeModelParamSet* pm = p2->GetParamSet();
        Calibration* calib = pm->m_calib;
    try
    {
		  pose1 = RelPoseFactory::CloneRelPose(p2->GetLastMatchedPose());
    }
    catch(...)
    {
      printf("Tying to copy a singular position\n");
      pose1 = RelPoseFactory::FRelPoseWorld();
    }
        if(img != NULL && pose1 != NULL && calib  != NULL)
        {
		    img->Hold();
		    DescriptorBased db;
		    pose2 = db.Inner(img, RelPoseFactory::FRelPose(pose1->m_parentID), calib, pose, sig, numOfObjects, qualityMeasure);
        }
	}

#ifdef _DEBUG
  if(pose1 != NULL && pose2.size() > 0)
  {
	  pose1->Print();
	  pose2[0]->Print();
  }
#endif

  if(pose1 != NULL && pose2.size() > 0)
  {
	  double ret = pose1->CompareLo(*pose2[0]);
	  delete pose1;

	  p1->Evaluate(ret);
	  p2->Evaluate(ret);
	  return ImprovedPose(p1, ret);
  }
#endif
  return ImprovedPose(NULL, 0.0);
}

double PRAShapeVsPointDescr::CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
{
	if(sig.GetElement(0,DESCRIPTOR_FEATURE) != NULL && sig.GetElement(0,DESCRIPTOR_SHAPE) != NULL)
	{
		return 1.0;
	}
	return 0.0;
}

XMLTag* PRAShapeVsPointDescr::Save()
{
	XMLTag *tag = new XMLTag(XML_NODE_PRASP);
	return tag;
}

