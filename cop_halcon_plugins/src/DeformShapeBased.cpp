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


/*#ifdef DEFORMSHAPE_AVAILABLE*/

#include "DeformShapeBased.h"
#include "DeformShapeModel.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RegionOI.h"
#include "RelPoseHTuple.h"


#include "cpp/HalconCpp.h"
using namespace Halcon;


using namespace cop;


DeformShapeBased::DeformShapeBased()
{
}


DeformShapeBased::DeformShapeBased(XMLTag* tag)
{

}

DeformShapeBased::~DeformShapeBased(void)
{

}

void DeformShapeBased::SetData(XMLTag* tag)
{
  printf("Loading Algorithm DeformShapeBased\n");
}

/**
  Action function, prepares images
*/
std::vector<RelPose*> DeformShapeBased::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Image* img = cam->GetImage(-1);
    RelPose* camPose = cam->m_relPose;
    if(img != NULL && camPose != NULL )
    {
      result = Inner(img, camPose, cam, lastKnownPose, object, numOfObjects, qualityMeasure, cam->GetSensorID());
    }
  }
  return result;
}
/**
  Final algorithm, for call for special images
*/
std::vector<RelPose*> DeformShapeBased::Inner(Image* img,RelPose* camPose,Camera* cam, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure, std::string stSensorName)
{
  std::vector<RelPose*> result;

  HTuple camparam = cam->m_calibration.CamParam();
  HTuple cammatrix = cam->m_calibration.CamMatrix();

  if(img->GetType() == ReadingType_HalconImage)
  {
    bool trackPossible = TrackingPossible(*img, object, lastKnownPose);
    //int n = 4;
    //double Partly = 1.0 - (0.3* (n-1));
    HTuple empty;
    HTuple  rotationAngleStart, rotationAngleExtend,
        scaleCMin, scaleCMax, scaleRMin, scaleRMax, minScore,
        numMatches, maxOverlap, numLevels, greediness, paramName, paramValue;
    HTuple  score,hommat,hommatfirst,pose, cov, error;


    DeformShapeModel* dsm = (DeformShapeModel*)(object.GetElement(0, DESCRIPTOR_DEFORMSHAPE ));
    if(dsm != NULL)
    {
      //hommatfirst = *dsm->GetHomMatFirst();
      try
      {
        Hobject imgs = *img->GetHImage();
        int handle = dsm->GetDeformShapeHandle(stSensorName);
        if(handle  == -1)
        {
          printf("\nIncompatible model\n");
          DeformShapeModel::SensorSpecificDeformModel& model = dsm->GetDeformShapeModel();
          if(!model.IsCompatible(cam))
          {
            camparam[0] = model.m_calib.focal_length;
            camparam[2] = model.m_calib.pix_size_x;
            camparam[3] = model.m_calib.pix_size_y;
            camparam[4] = model.m_calib.proj_center_x;
            camparam[5] = model.m_calib.proj_center_y;

            printf("\nAdapting image to %d x %d ->  %f x %f \n", camparam[6].I(), camparam[7].I(), model.m_calib.width, model.m_calib.height);

            camparam[6] = model.m_calib.width;
            camparam[7] = model.m_calib.height;
            zoom_image_size (imgs, &imgs, camparam[6], camparam[7], "constant");
          }
          handle = model.m_handle;
        }
        else
          printf("\nGot handle for Sensor %s: %d\n\n", stSensorName.c_str(), handle);
        numMatches = numOfObjects;
        numOfObjects = 0;
        HTuple pose, cov, area_roi = 0;
        Hobject roi_obj;
        if(lastKnownPose != NULL)
        {
            RegionOI roi(lastKnownPose, img->GetPose()->m_uniqueID, &cam->m_calibration);
            try
            {
              HTuple rowtmp, coltmp;
              dilation_circle(roi.GetRegion(), &roi_obj, 150);
              area_center(roi_obj, &area_roi, &rowtmp, &coltmp );
            }
            catch(HException ex)
            {
              printf("DeformShapeBased::Perform: Error creating region: %s\n", ex.message);
              area_roi = 0;
            }
        }
        if(!trackPossible)
        {
          printf("Deform: Not tracking\n");
          rotationAngleStart = -1.0;
          rotationAngleExtend = 1.0;
          minScore = 0.6;
          maxOverlap = 1.0;
          numLevels = 0;
          greediness = 0.8;
          scaleCMin = 1.0;
          scaleCMax = 1.0;
          scaleRMin = 1.0;
          scaleRMax = 1.0;
          paramName = "subpixel";
          paramValue = "least_squares_very_high";
          if(area_roi[0].I() > 1000)
          {
            Hobject img_reduced;
            reduce_domain(imgs, roi_obj, &img_reduced);
            find_planar_calib_deformable_model(img_reduced, handle,rotationAngleStart, rotationAngleExtend,
              scaleRMin, scaleRMax, scaleCMin, scaleCMax, minScore,
              numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
              &pose, &cov, &score);


          }
          else
          {
            find_planar_calib_deformable_model(imgs, handle,rotationAngleStart, rotationAngleExtend,
              scaleRMin, scaleRMax, scaleCMin, scaleCMax, minScore,
              numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
              &pose, &cov, &score);
          }
        }
        else
        {
          printf("Reduced search possible\n");
          rotationAngleStart = -0.5;
          rotationAngleExtend = 0.5;
          minScore = 0.6;
          maxOverlap = 1.0;
          numLevels = 0;
          greediness = 0.9;
          scaleCMin = 1.0;
          scaleCMax = 1.0;
          scaleRMin = 1.0;
          scaleRMax = 1.0;
          paramName = "subpixel";
          paramValue = "least_squares_very_high";
          if(area_roi[0].I() > 1000)
          {
            Hobject img_reduced;
            reduce_domain(imgs, roi_obj, &img_reduced);
            find_planar_calib_deformable_model(img_reduced, handle,rotationAngleStart, rotationAngleExtend,
              scaleRMin, scaleRMax, scaleCMin, scaleCMax, minScore,
              numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
              &pose, &cov, &score);

          }
          else
          {
            find_planar_calib_deformable_model(imgs, handle,rotationAngleStart, rotationAngleExtend,
              scaleRMin, scaleRMax, scaleCMin, scaleCMax, minScore,
              numMatches, maxOverlap, numLevels, greediness, paramName, paramValue,
              &pose, &cov, &score);
          }
          printf("here\n");
        }
        printf("Finished Search (%ld results)\n", score.Num());
        if(score.Num() > 0)
        {
          printf("Estimated Pose:\n");
          int i;
          for(i = 0; i < 6; i++)
          {
            printf("%f, ", pose[i].D());
          }
          printf("%d\n", pose[6].I());
          printf("\n");
          HTuple pose_sel, cov_sel;
          for(i = 0; i < score.Num(); i++)
          {
            try
            {
              Halcon::HTuple hom, hom_new;
              tuple_select_range(pose, 0 + 7*i, 6 + 7*i, &pose_sel);
              tuple_select_range(cov, 0+ 6*i, 5+ 6*i, &cov_sel);
              /*Halcon::pose_to_hom_mat3d(pose_sel, &hom);
              Halcon::hom_mat3d_rotate_local(hom, -M_PI, "y", &hom_new);
              Halcon::hom_mat3d_to_pose(hom_new, &pose_sel);*/
              pose_to_hom_mat3d (pose_sel, &hom);

              cout << "Compensation" << endl <<  dsm->m_planarAssumtionCompensation << endl;
              hom_mat3d_compose(hom,RelPoseHTuple::m2HT(dsm->m_planarAssumtionCompensation), &hom_new);
              hom_mat3d_to_pose (hom_new, &pose_sel);

              for(int k = 0; k< 6; k++)printf("%f ", pose_sel[k].D());
              printf("\n");
              
              for(int k = 0; k< 6; k++)
                cov_sel[k] = cov_sel[k].D() + dsm->m_covInherit.element(k,k);
              
              RelPose* pose = RelPoseHTuple::FRelPose(pose_sel, cov_sel, img->GetPose());
              
              cout<< endl << "CovDetected" <<endl<< pose->GetCovarianceMatrix(0) << endl <<endl;
              
              if(pose != NULL)
              {
                pose->m_qualityMeasure = score[i];
                result.push_back(pose);
                numOfObjects++;
              }
            }
            catch(HException ex)
            {
              printf("Error in DeformShapeBased %s\n", ex.message);
            }
          }
          qualityMeasure = score[0].D();
          if(numOfObjects > 0)
          {
            /* dsm->SetLastMatchedImage(img, result[0]);*/
          }
          else
          {
            printf("Rejected all solutions\n");
          }
        }
        else
        {

          numOfObjects = 0;
          qualityMeasure = 0.0;
        }
        img->Free();
        return result;
      }
      catch(Halcon::HException ex)
      {
        printf("Error in DeformShapeBased: %s\n", ex.message);
        qualityMeasure = 0.0;
        numOfObjects = 0;
      }
      catch(const char* text)
      {
        printf("No valid model for this sensor: %s\n", text);
        qualityMeasure = 0.0;
        numOfObjects = 0;
      }
    }
  }
  img->Free();

  return result;
}

/**
  Test if the already there are models learned
*/
double DeformShapeBased::CheckSignature(const Signature& object, const std::vector<Sensor*> &sens)
{
  if(object.GetElement(0,DESCRIPTOR_DEFORMSHAPE))
    return 1.0;
  else
    return 0.0;
}



/**
  Save the parameters or any
*/
XMLTag* DeformShapeBased::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_DEFORMSHAPEBASEDALG);
  return tag;
}
/*#endif*/ /*DEFORMSHAPE_AVAILABLE*/


