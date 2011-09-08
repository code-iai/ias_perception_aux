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


#include "FindCalTab.h"
#include "CalTab.h"
#include "XMLTag.h"
#include "RelPoseHTuple.h"


#undef NATIVE_COP_CALIB

#ifndef NATIVE_COP_CALIB
#include "HalconCalib.h"
#endif


#include "cpp/HalconCpp.h"
using namespace Halcon;


using namespace cop;


FindCalTab::FindCalTab()
{
}

FindCalTab::FindCalTab(XMLTag* tag)
{
}

FindCalTab::~FindCalTab(void)
{
}

XMLTag* FindCalTab::Save()
{
  return new XMLTag(XML_NODE_FINDCALTAB);
}

std::vector<RelPose*> FindCalTab::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Calibration* calib = &(cam->m_calibration);
    Image* img = cam->GetImage(-1);
    /** TODO: TOREMOVE **/
    /*cam->Show();*/
    RelPose* camPose = cam->m_relPose;
    if(img != NULL && camPose != NULL)
      result = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
  }
  return result;
}

double FindCalTab::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensor)
{
  if(object.GetElement(0,DESCRIPTOR_CALTAB) != NULL)
    return 1.0;
  else
    return 0.0;
}

std::vector<RelPose*> FindCalTab::Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, CalTab* cm, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  if(img->GetType() == ReadingType_HalconImage)
  {
    HTuple    x, y, z;    // calibration table parameter
    HTuple    Row, Col;     // detected marker image positions
    HTuple    StartPose;      // estimated start pose
    HTuple    FinalPose;      // calibrated external pose
    HTuple    Errors;
      try
      {
      HTuple campar_start = calib->CamParam();
      HTuple campar_final;
      caltab_points(cm->m_stCalTabDescriptionFile.c_str(), &x,&y,&z);
      if(!FindCaltab(campar_start, *(img->GetHImage()), cm->m_stCalTabDescriptionFile.c_str(),
               Row, Col,  StartPose, cm))
       {
          img->Free();
          numOfObjects = 0;
          qualityMeasure = 0.0;
          return result;
        }
        printf("StartPose: %f %f %f\n", StartPose[0].D(),  StartPose[1].D(), StartPose[2].D());
        camera_calibration(x, y, z, Row, Col, campar_start, StartPose, "pose", &campar_final, &FinalPose, &Errors);
        printf("FinalPose: %f %f %f\n", FinalPose[0].D(),  FinalPose[1].D(), FinalPose[2].D());
      }
      catch(HException ex)
      {
          img->Free();
          numOfObjects = 0;
          qualityMeasure = 0.0;
          return result;
      }
    HTuple cov(6, 0.02);
    cov[2] = 0.4;
    cov[3] = 0.4;
    cov[4] = 0.4;
    cov[5] = 0.4;

    result.push_back(RelPoseHTuple::FRelPose(FinalPose, cov,  camPose));

    numOfObjects = 1;
    qualityMeasure = 0.9;
    img->Free();
    return result;
  }
  img->Free();
  numOfObjects = 0;
  qualityMeasure = 0.6;
  return result;
}

std::vector<RelPose*> FindCalTab::Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;

#ifdef NATIVE_COP_CALIB
  if(img->GetType() == HALCONIMAGE)
  {
    HTuple  rc, cc, startPose, x,y ,z, finalpose, errors;
    HTuple campar = calib->CamParam();
    /*printf("Get Shape Model\n");*/
    CalTab* cm = (CalTab*)(object.GetElement(0, DESCRIPTOR_CALTAB ));
    Hobject reg;
    try
    {
	    //you can put Jan's function here - take the for-loop, baby - to determine these parameters
      int sizegauss = 3;
      int MarkThres = 200; //lower for dark images
      int minDiamMsrks = 5;
      int StartThres = 128; //similar to MarkThresh
      int DeltaThres = 10; // step size for decreasing the threshold
      int MinThres = 18; //minimum threshold
      double Alpha = 0.9;//for edge detection
      int MinContLength =15; //length of the circumference of the markers
      int MaxDiamMarks = 100; //
      find_caltab(*img->GetHImage(), &reg, cm->m_stCalTabDescriptionFile.c_str(), 3, 112, 5);
      find_marks_and_pose(*img->GetHImage(), reg, cm->m_stCalTabDescriptionFile.c_str(),campar , StartThres,
        DeltaThres , MinThres, Alpha, MinContLength, MaxDiamMarks, &rc, &cc, &startPose);
      caltab_points(cm->m_stCalTabDescriptionFile.c_str(), &x,&y,&z);
      camera_calibration(x,y,z, rc, cc, campar, startPose, "pose", &campar, &finalpose, &errors);

    }
    catch(HException ex)
    {
      printf("Error in FinCalTab: %s\n", ex.message);
      img->Free();
      numOfObjects = 0;
      qualityMeasure = 0.0;
      return result;
    }
    HTuple cov(6, 0.02);
    cov[2] = 0.4;
    cov[3] = 0.4;
    cov[4] = 0.4;
    cov[5] = 0.4;
    if(object.GetObjectPose() != NULL && object.GetObjectPose()->m_uniqueID!=ID_WORLD)
        {
            result.push_back(RelPoseHTuple::FRelPose(finalpose, cov, camPose, object.GetObjectPose()->m_uniqueID));
            cerr<<"In const. 4"<<endl;
        }
    else
        {
            result.push_back(RelPoseHTuple::FRelPose(finalpose, cov,  camPose));
            cerr<<"In const. 3"<<endl;
        }
    numOfObjects = 1;
    qualityMeasure = 0.9;
    img->Free();
    return result;
  }
#else

  if(img->GetType() == ReadingType_HalconImage)
  {
    HTuple    x, y, z;    // calibration table parameter
    HTuple    Row, Col;     // detected marker image positions
    HTuple    StartPose;      // estimated start pose
    HTuple    FinalPose;      // calibrated external pose
    HTuple    Errors;
    try
    {
      HTuple campar_start = calib->CamParam();
      HTuple campar_final;
      CalTab* cm = (CalTab*)(object.GetElement(0, DESCRIPTOR_CALTAB ));

      caltab_points(cm->m_stCalTabDescriptionFile.c_str(), &x,&y,&z);
      Hobject img_eq, *obj = img->GetHImage();
      equ_histo_image(*obj, &img_eq);
      if(!FindCaltab(campar_start, img_eq, cm->m_stCalTabDescriptionFile.c_str(),
                Row, Col,  StartPose, cm))
      {
        img->Free();
        numOfObjects = 0;
        qualityMeasure = 0.0;
        return result;
      }
      printf("StartPose: %f %f %f\n", StartPose[0].D(),  StartPose[1].D(), StartPose[2].D());
      camera_calibration(x, y, z, Row, Col, campar_start, StartPose, "pose", &campar_final, &FinalPose, &Errors);
      printf("FinalPose: %f %f %f\n", FinalPose[0].D(),  FinalPose[1].D(), FinalPose[2].D());
    }
    catch(HException ex)
    {
      img->Free();
      numOfObjects = 0;
      qualityMeasure = 0.0;
      return result;
    }
    HTuple cov(6, 0.02);
    cov[2] = 0.4;
    cov[3] = 0.4;
    cov[4] = 0.4;
    cov[5] = 0.4;

    if(object.GetObjectPose() != NULL && object.GetObjectPose()->m_uniqueID!=ID_WORLD)
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
    qualityMeasure = 0.9;
    img->Free();
    return result;
  }
#endif /* NATIVE_COP_CALIB*/
  img->Free();
  numOfObjects = 0;
  qualityMeasure = 0.6;
  return result;
}
