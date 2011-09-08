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
                        ShapeBased3D.cpp - Copyright klank

**************************************************************************/

#include "ShapeBased3D.h"
#include "XMLTag.h"
#include "RelPoseHTuple.h"


#include "cpp/HalconCpp.h"
#include "SupportingPlaneDescriptor.h"
#include "SupportingPlaneDetector.h"
using namespace Halcon;

#include "ShapeModel.h"

#define XML_ATTRIBUTE_MINSCORE "MinScore"
#define XML_ATTRIBUTE_GREEDY   "Greediness"
#define XML_ATTRIBUTE_LEVELS   "Levels"

using namespace cop;



/**
* DEFAULT Values
*/
#define DEFAULT_MINSCORE 0.40
#define DEFAULT_GREEDINESS 0.80
#define DEFAULT_LEVELS 0
/**
 *  Macro for assigning standard values to the parameter list
*/
#define ASSIGN_DEFAULT_PARAM_LIST(m_paramNameList)   \
       (*m_paramNameList)[0] = "longitude_min"; \
      (*m_paramNameList)[1] = "longitude_max";  \
      (*m_paramNameList)[2] = "latitude_min";   \
      (*m_paramNameList)[3] = "latitude_max";   \
      (*m_paramNameList)[4] = "cam_roll_min";   \
      (*m_paramNameList)[5] = "cam_roll_max";   \
      (*m_paramNameList)[6] = "dist_min";       \
      (*m_paramNameList)[7] = "dist_max";       \
      (*m_paramNameList)[8] = "num_matches"; \

// Constructors/Destructors
//
ShapeBased3D::ShapeBased3D () :
  LocateAlgorithm(),
  m_minScore(DEFAULT_MINSCORE),
  m_greediness(DEFAULT_GREEDINESS),
  m_levels(DEFAULT_LEVELS)
{

      m_paramList = new HTuple(10, 0.0);
      m_paramNameList = new HTuple(10, "");
      tuple_gen_const(9, 0.0, m_paramList);
      tuple_gen_const(9, "", m_paramNameList);
  ASSIGN_DEFAULT_PARAM_LIST(m_paramNameList)
      (*m_paramNameList)[9] = "pose_refinement";
      (*m_paramList)[9] = "none";


}


void ShapeBased3D::SetData (XMLTag* tag)
{
  if(tag != NULL)
  {
    m_minScore = tag->GetPropertyDouble(XML_ATTRIBUTE_MINSCORE, DEFAULT_MINSCORE);
    if(m_minScore < 0.2 || m_minScore > 0.99)
    {
      m_minScore = DEFAULT_MINSCORE;
    }
    m_greediness = tag->GetPropertyDouble(XML_ATTRIBUTE_GREEDY, DEFAULT_GREEDINESS);
    if(m_greediness < 0.0 || m_greediness > 0.99)
    {
      m_greediness = DEFAULT_GREEDINESS;
    }
    m_levels = tag->GetPropertyInt(XML_ATTRIBUTE_LEVELS,DEFAULT_LEVELS);
    if(m_levels < 0 || m_levels > 7)
    {
      m_levels = DEFAULT_LEVELS;
    }
  }
  m_paramList = new HTuple(9, 0.0);
  m_paramNameList = new HTuple(9, "");
  ASSIGN_DEFAULT_PARAM_LIST(m_paramNameList);
  (*m_paramNameList)[9] = "pose_refinement";
  (*m_paramList)[9] = "none";

}


XMLTag* ShapeBased3D::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  tag->AddProperty(XML_ATTRIBUTE_MINSCORE, m_minScore);
  tag->AddProperty(XML_ATTRIBUTE_GREEDY, m_greediness);
  tag->AddProperty(XML_ATTRIBUTE_LEVELS, m_levels);
  return tag;
}


ShapeBased3D::~ShapeBased3D ( )
{

  delete m_paramList;
  delete m_paramNameList;

}


/**
*   Given an valid last-pose of the object we want to try to track it.
*    TODO: make sure, that we do not track on search süace changes.
*/
void GuidedPoseRestiction(Halcon::HTuple* paramList,Halcon::HTuple* paramNameList, Hlong model, double fraction, RelPose* pose, Halcon::HTuple camparam,Halcon::Hobject* reg, ShapeModelParamSet* pm)
{
  Halcon::HTuple poseout, hpose, hommat3d, hommat3dInv, poserefabg, longitude, latitude, distance, camroll, row, col;
  /* generate Tuple with the pose */
  RelPoseHTuple::GetPose(pose, &hpose);
  /* TOCHECK: what happens here*/
  Halcon::trans_pose_shape_model_3d(model,hpose, "model_to_ref", &poseout);
  /* Invert the pose*/
  Halcon::pose_to_hom_mat3d(poseout, &hommat3d);
  Halcon::hom_mat3d_invert(hommat3d, &hommat3dInv);
  /* Calc cartesian */
  Halcon::convert_point_3d_cart_to_spher(hommat3dInv[3].D(), hommat3dInv[7].D(),hommat3dInv[11].D(), "-y", "-z", &longitude, &latitude, &distance);
  /* Coordinates for region */
  Halcon::project_3d_point(hommat3d[3].D(), hommat3d[7].D(), hommat3d[11].D(), camparam, &row, &col);
#ifdef _DEBUG
  printf("Tracked object in the image at %f %f (lon: %f lat %f dist %f) \n", row[0].D(), col[0].D(), longitude[0].D(),latitude[0].D(),distance[0].D());
#endif
  Halcon::gen_circle(reg, row, col, 100);
  Halcon::convert_pose_type(poseout, "Rp+T", "abg", "point", &poserefabg);
  camroll = - (((poserefabg[5].D()) / 360) * 2 * M_PI);
  double dlong = pm->m_longitudeMin * fraction; /*model->GetShapeModel3dParams((*paramNameList)[0].S())[0].D() * fraction;*/
  double dlat = pm->m_latitudeMin * fraction; /*model->GetShapeModel3dParams((*paramNameList)[2].S())[0].D() * fraction;*/
  double ddist = pm->m_distMin * fraction;; /*model->GetShapeModel3dParams((*paramNameList)[6].S())[0].D() * fraction;*/
  (*paramList)[0] = longitude - dlong;  /*long min*/
  (*paramList)[1] = longitude + dlong;	/*long max*/
  (*paramList)[2] = latitude - dlat;		 /*lat min*/
  (*paramList)[3] = latitude + dlat;		 /*lat max*/
  (*paramList)[4] = pm->m_camRollMin;  /*model->GetShapeModel3dParams((*paramNameList)[4].S());		 cam roll min*/
  (*paramList)[5] = pm->m_camRollMax; /*model->GetShapeModel3dParams((*paramNameList)[5].S());		 cam roll max*/
  (*paramList)[6] = distance - ddist;		 /*dist min*/
  (*paramList)[7] = distance + ddist;		 /*dist max*/
}

/**
* Test a pose if it is inside the search space.
*/
bool CheckPose(Halcon::HTuple pose, ShapeModelParamSet* pm, Hlong model)
{
  try
  {
  Halcon::HTuple poseout, hommat3d, hommat3dInv, poserefabg, longitude, latitude, distance, camroll;
  /* generate Tuple with the pose */
  /* TOCHECK: what happens here*/
  Halcon::trans_pose_shape_model_3d(model,pose, "model_to_ref", &poseout);
  /* Invert the pose*/
  Halcon::pose_to_hom_mat3d(poseout, &hommat3d);
  Halcon::hom_mat3d_invert(hommat3d, &hommat3dInv);
  Halcon::convert_point_3d_cart_to_spher(hommat3dInv[3].D(), hommat3dInv[7].D(),hommat3dInv[11].D(), "-y", "-z", &longitude, &latitude, &distance);
  /* Coordinates for region */
  Halcon::convert_pose_type(poseout, "Rp+T", "abg", "point", &poserefabg);
  camroll = - (((poserefabg[5].D()) / 360) * 2 * M_PI);

  if(/*pm->m_longitudeMin < longitude + 0.05 &&
     pm->m_longitudeMax > longitude - 0.05 &&
     pm->m_latitudeMin < latitude + 0.05 &&
     pm->m_latitudeMax > latitude - 0.05 && */
     pm->m_distMin < distance + 0.1&&
     pm->m_distMax > distance - 0.1 /*&&
     pm->m_camRollMin < camroll + 0.3 &&
     pm->m_camRollMax > camroll - 0.3*/)
    return true;
  return false;
  }
  catch(...)
  {
    printf("Check Pose failed,\n");
    return true;
  }
}


//
// Methods
//
int ShapeBased3D::FindBestSupportedMatch(int &numOfObjects, std::vector<std::vector< RelPose*> > poses, std::vector<double> quality)
{
  double max_quali = -1.0;
  int max_index = 0;
  //int numOfObjectsLeft = numOfObjects;
  for(unsigned int i = 0; i < quality.size(); i++)
  {
    if(quality[i] > max_quali)
    {
      if(poses[i].size() > 0)
      {
          max_quali = quality[i];
          max_index = i;
          /**TODO check hypothesis by overlapping with others*/
      }
    }
  }
  return max_index;
}

std::vector<RelPose*>  ShapeBased3D::PerformForAll(Image* img, RelPose* camPose,Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  double max_eval = 0.0;
  double max_initlevel = 0.0;
  ShapeModel* shape = NULL;
  std::vector< std::vector<RelPose*> > result;
  std::vector< double  > quality;
  int i = 0;
  while((shape = (ShapeModel*)object.GetElement(i++, DESCRIPTOR_SHAPE )) != NULL)
  {
    if(max_eval + max_initlevel < shape->GetQuality() + shape->m_initializationLevel)
    {
      max_eval = shape->GetQuality();
      max_initlevel = shape->m_initializationLevel;
    }
  }
  max_eval -= 0.1;
  max_initlevel -= 0.1;
  printf("Level to execute set to\neval: %f\ninit %f\n", max_eval, max_initlevel);
  i = 0;
  while((shape = (ShapeModel*)object.GetElement(i, DESCRIPTOR_SHAPE )) != NULL)
  {
    if(max_eval < shape->GetQuality() && max_initlevel  < shape->m_initializationLevel )
    {
      try
      {
        int numOfObjectsInner = numOfObjects;
        double qualityInner = qualityMeasure;
        std::vector<RelPose*> vec = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjectsInner, qualityInner, i);
        result.push_back(vec);
        quality.push_back(qualityInner);
        shape->m_showNow = false;
      }
      catch(const char* text)
      {
        shape->Evaluate(-1, 1.0);
        printf("ShapeBased3D::PerformForAll: Error in one of the models: %s\n", text);
      }
      catch(Halcon::HException ex)
      {
        shape->Evaluate(-1, 1.0);
        printf("ShapeBased3D::PerformForAll: Error in one of the models: %s\n", ex.message);
      }
      catch(...)
      {
        shape->Evaluate(-1, 1.0);
        printf("ShapeBased3D::PerformForAll: Error in one of the models.\n");
      }
    }
    i++;
  }
  int index_best = FindBestSupportedMatch(numOfObjects, result, quality);
#ifdef _DEBUG
  printf("ShapeBased3D: Selected match of model %d as the best\n", index_best);
#endif
  shape = ((ShapeModel*)object.GetElement(index_best, DESCRIPTOR_SHAPE ));
  if(shape != NULL)
  {
    shape->m_showNow = true;
    if(shape->m_initializationLevel < 0.3)
    {
        SupportingPlaneDetector detector;

        double qualPlane = qualityMeasure;
        int  numOfObjectsInner = numOfObjects;
        /**TODO incooperate usage of index*/
#ifdef _DEBUG
        printf("ShapeBased3D: Trying to improve the Match (bad init Value: %f ) by projecting it on a supporting plane\n Searching...\n", shape->m_initializationLevel);
#endif
        std::vector<RelPose*> poses = detector.Inner(img, camPose, calib, lastKnownPose, object, numOfObjectsInner, qualPlane, 0);
        if(poses.size() > 0)
        {
#ifdef _DEBUG
          printf("Found a supporting plane\n");
#endif
        /**TODO incooperate usage of index*/
          if(result.size() != 0  && result[index_best].size() > 0)
          {
            shape->IntersectWithSupportingPlane(poses[0], result[index_best][0], camPose, calib);
          }

        }
    }
  }
  if(result.size() == 0 || shape == NULL)
      return std::vector<RelPose*>();
  numOfObjects = result.size();
  qualityMeasure = quality[index_best];
  return  result[index_best];
}
std::vector<RelPose*> ShapeBased3D::Perform(std::vector<Sensor*> sensors, RelPose* lastKnownPose_in, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  printf("GetFirstCam");

  Camera* cam = Camera::GetFirstCamera(sensors);
#ifdef _DEBUG
  printf("Got Camera: %s\n", cam != NULL ? cam->GetName().c_str() : "None");
#endif // _DEBUG
  try
  {
    if(cam != NULL)
    {
      Calibration* calib = &(cam->m_calibration);
      Image* img = cam->GetImage(-1);
      RelPose* camPose = img->GetPose();
      RelPose* lastKnownPose = RelPoseFactory::GetRelPose(lastKnownPose_in->m_uniqueID, camPose->m_uniqueID);
      if(object.CountElems() > 1)
      {
        result = PerformForAll(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
      }
      else
      {
        if(img != NULL && camPose != NULL)
          result = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
      }
      RelPoseFactory::FreeRelPose(&lastKnownPose);
    }
    else
    {
      numOfObjects = 0;
      qualityMeasure = 0.0;
    }
  }
  catch(const char *text)
  {
    /** If the model is not too small or too big in one of the views it might be good to check in another view*/
    ROS_INFO("cop: ShapeBased3D has most probably selected the wrong sensor (%s), trying again with a different\n", cam->GetSensorID().c_str());
    int num_cam = 0;
    cam = NULL;
    for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
    {
      if((*it)->IsCamera())
      {
        if(num_cam++ > 1)
        {
          cam = (Camera*)(*it);
        }
      }
    }
    if(cam != NULL)
    {
      Calibration* calib = &(cam->m_calibration);
      Image* img = cam->GetImage(-1);
      RelPose* camPose = img->GetPose();
      RelPose* lastKnownPose = RelPoseFactory::GetRelPose(lastKnownPose_in->m_uniqueID, camPose->m_uniqueID);
      if(object.CountElems() > 1)
      {
        result = PerformForAll(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
      }
      else
      {
        if(img != NULL && camPose != NULL)
          result = Inner(img, camPose, calib, lastKnownPose,  object, numOfObjects, qualityMeasure);
      }
      RelPoseFactory::FreeRelPose(&lastKnownPose);
    }
    else
    {
      numOfObjects = 0;
      qualityMeasure = 0.0;
    }
  }
  return result;
}

std::vector<RelPose*> ShapeBased3D::Inner(Image* img, RelPose* camPose,Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure, int i)
{
  std::vector<RelPose*> result;
  if(img->GetType() == ReadingType_HalconImage)
  {
    HTuple  ViewPose, pose, CovPose, Score;

    /*printf("Get Shape Model\n");*/
    ShapeModel* sm = (ShapeModel*)(object.GetElement(i, DESCRIPTOR_SHAPE ));
    if(sm == NULL)
    {
      printf("Shape Model not found\n");
      return result;
    }
#ifdef _DEBUG
/*    RelPoseHTuple::Print(lastKnownPose);*/
#endif

    bool trackPossible = TrackingPossible(*img, object, lastKnownPose);
#ifdef _DEBUG
    printf("TrackingPossible result: %s\n", trackPossible ? "true" : "false");
#endif
    trackPossible = false; /*TODO debug tracking*/
    if(!trackPossible)
    {
      sm->SetShapeModelParamSet(lastKnownPose, calib);
    }
    ShapeModelParamSet* pm = sm->GetParamSet();
    /*sm->m_showNow = true;
    sm->Show(lastKnownPose, NULL);*/
    double scale;
    long model = sm->GetShapeModel(scale);
    if(model == -1)
    {
      printf("BigProblem with shape models\n");
    }
    else
    {
      /*printf("Start Shape model detection\n");*/
      qualityMeasure = 0.0;

      Halcon::HTuple pointer, t, w, h;
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::get_image_pointer1(*obj, &pointer, &t, &w, &h);
      if(scale != 1.0)
      {
        Halcon::Hobject* zoomed = new Halcon::Hobject();
        Halcon::zoom_image_size(*obj, zoomed, (int)(w[0].I()*scale), (int)(h[0].I()*scale), "constant");
        obj = zoomed;
      }
      else
      {
        /*if(w != calib->m_width)
          printf("nix gut");
        Halcon::HTuple tup;
        Halcon::count_channels(*obj, &tup);
        if(tup[0].I() == 3)
        {
            Halcon::rgb1_to_gray(*obj, obj);
            printf("RGB > Gray\n");
        }*/
      }
      Halcon::get_image_pointer1(*obj, &pointer, &t, &w, &h);

      (*m_paramList)[8] = numOfObjects;

        if(trackPossible)
        {
          Halcon::Hobject region;
          GuidedPoseRestiction(m_paramList, m_paramNameList, model, 0.1, object.GetObjectPose(), calib->CamParam(scale),&region, pm);
          /*Halcon::reduce_domain(*obj, region, obj);*/
        }
        else
        {
        /* Restriction of Search array (TODO derive it from estimated position)*/
          (*m_paramList)[0] = pm->m_longitudeMin;/*long min*/
          (*m_paramList)[1] = pm->m_longitudeMax;/*model->GetShapeModel3dParams((*m_paramNameList)[1].S()) * Partly;		 *long max*/
          (*m_paramList)[2] = pm->m_latitudeMin;/*model->GetShapeModel3dParams((*m_paramNameList)[2].S()) * Partly;		 *lat min*/
          (*m_paramList)[3] = pm->m_latitudeMax;/*model->GetShapeModel3dParams((*m_paramNameList)[3].S()) * Partly;		 *lat max*/
          (*m_paramList)[4] = pm->m_camRollMin;/*model->GetShapeModel3dParams((*m_paramNameList)[4].S());		 *cam roll min*/
          (*m_paramList)[5] = pm->m_camRollMax;/*model->GetShapeModel3dParams((*m_paramNameList)[5].S());		 *cam roll max*/
          (*m_paramList)[6] = pm->m_distMin;/*model->GetShapeModel3dParams((*m_paramNameList)[6].S());		 *dist min*/
          (*m_paramList)[7] = pm->m_distMax;/*model->GetShapeModel3dParams((*m_paramNameList)[7].S());		 *dist max*/
        }
  #ifdef _DEBUG
        printf("Parameter for detection:\n%f %f // %f %f \n %f %f // %f %f\n", /*long min:*/(*m_paramList)[0].D(),
          /*long max:*/(*m_paramList)[1].D(),
              /*lat min:*/(*m_paramList)[2].D(),
              /*lat max:*/(*m_paramList)[3].D(),
              /*cam roll min:*/(*m_paramList)[4].D(),
              /*cam roll max:*/(*m_paramList)[5].D(),
              /*dist min:*/(*m_paramList)[6].D(),
              /*dist max:*/(*m_paramList)[7].D());
        //printf("Run FindShapeModel with minscore: %f \nlevels %d\ngreediness: %f\n", minScore, level, m_greediness);
  #endif
        try
        {
          if(pm->m_region != NULL)
          {
              /*assert(pm->m_region->GetSize() > 0);*/
              HTuple tuple_temp1,tuple_temp2, empty_t;
              get_shape_model_3d_params(model, (*m_paramNameList)[0].S(), &tuple_temp1);
              get_shape_model_3d_params(model, (*m_paramNameList)[1].S(), &tuple_temp2);
  #ifdef _DEBUG
              printf("Start search Model: %ld . lonmin %f lonmax %f (minscore %f)\n", model, tuple_temp1[0].D(),tuple_temp2[0].D(), m_minScore);
  #endif
              Hobject img_tmp;
              reduce_domain(*obj, pm->m_region->GetRegion(scale), &img_tmp);
              find_shape_model_3d(img_tmp, model, /*< Image here*/
              m_minScore, m_greediness, m_levels, /*< Min Score, Greediness, Levels*/
              *m_paramNameList, *m_paramList, /*< Generic Params (gp), gp Values*/
                  &pose, &CovPose, &Score);/*< Cov,Score */
         }
          else
          {
            /*Halcon::HTuple level(2, 5);*/
          /*  level[1] = 3;*/
  #ifdef _DEBUG
            printf("Start search Model: %ld .\n", model);
  #endif
            find_shape_model_3d(*obj, model, /*< Image here*/
                 m_minScore, m_greediness, m_levels/*level*/, /*< Min Score, Greediness, Levels*/
                  *m_paramNameList, *m_paramList, /*< Generic Params (gp), gp Values*/
                  &pose, &CovPose, &Score);/*< Cov,Score */
           }
        }
        catch(Halcon::HException ex)
        {
          printf("Finding failed: %s  (img %d, %d)\n", ex.message, w[0].I(), h[0].I());
        }
  #ifdef _DEBUG
        printf("Finished detection with %ld object%s found\n", Score.Num(), Score.Num()==1?"":"s");
  #endif
        if(Score.Num() > 0)
        {
  #ifdef _DEBUG
          printf("Found %ld objects with max score %f\nPose: %f,%f,%f, // %f,%f,%f // %d\n", Score.Num(), Score[0].D(), pose[0].D(), pose[1].D(), pose[2].D(), pose[3].D(), pose[4].D(), pose[5].D(), pose[6].I());
  #endif
          qualityMeasure = Score[0].D();
          numOfObjects = Score.Num();
          HTuple pose_sel, cov_sel;
          for(int i = 0; i < Score.Num(); i++)
          {
            tuple_select_range(pose, 0 + 7*i, 6 + 7*i, &pose_sel);
            tuple_select_range(CovPose, 0+ 6*i, 5+ 6*i, &cov_sel);
            if(CheckPose(pose_sel, pm, model))
            {
              if(false && object.GetObjectPose() != NULL && i == 0 && object.GetObjectPose()->m_uniqueID!=ID_WORLD)
              {
                result.push_back(RelPoseHTuple::FRelPose(pose_sel, cov_sel, camPose, object.GetObjectPose()->m_uniqueID));
                result.back()->m_qualityMeasure = Score[i].D();
              }
              else
              {
                result.push_back(RelPoseHTuple::FRelPose(pose_sel, cov_sel, camPose));
                result.back()->m_qualityMeasure = Score[i].D();
              }
            }
            else
            {
              numOfObjects --;
              /*qualityMeasure -= Score[i].D();*/
  #ifdef _DEBUG
            printf("One object out of search space! Removed\n");
  #endif
            }
          }
          if(numOfObjects > 0)
          {
             sm->SetLastMatchedImage(img, result[0]);
  #ifdef _DEBUG
            printf("Return from inner of ShapeBased3D (success)\n");
  #endif
          }
          img->Free();
        }
        else
        {
          numOfObjects = 0;
  #ifdef _DEBUG
          printf("Return from inner of ShapeBased3D (no match)\n");
  #endif
        }
        return result;
    }
  }
  img->Free();
  return result;
}

double ShapeBased3D::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  //TODO
  printf("ShapeBased3D::CheckSignature: Obj %ld, num elems %ld\n", object.m_ID, object.CountElems());
  if(object.GetElement(0,DESCRIPTOR_SHAPE) != NULL)
    return 1.0;
  else
    return 0.0;
}

// Accessor methods
//

// Other methods
//


