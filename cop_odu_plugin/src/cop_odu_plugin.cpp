/*
 * Copyright (c) 2010, Ulrich Klank, Dejan Pangercic
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cop_odu_plugin.h"
#include "XMLTag.h"

#include <iostream>
#include <cstdio>

/*  . */
#include "IplImageReading.h"
/* */

#include "NamedDescriptor.h"

#include <std_msgs/String.h>

#define XML_ATTRIBUTE_COMMAND                             "cascade_name"
#define XML_ATTRIBUTE_DATABASE_LOCATION                   "nested_cascade_name"
#define XML_ATTRIBUTE_IMAGES_DIRECTORY                    "scale"
#define XML_ATTRIBUTE_IMAGES_FOR_VISUALIZATION_DIRECTORY "images_for_visualization_directory"
#define XML_ATTRIBUTE_VOTES_COUNT                         "votes_count"
#define XML_ATTRIBUTE_TREE_K                              "tree_k"
#define XML_ATTRIBUTE_TREE_LEVELS                          "tree_levels"
#define XML_ATTRIBUTE_MIN_CLUSTER_SIZE                    "min_cluster_size"
#define XML_ATTRIBUTE_UNKNOWN_OBJECT_THRESHOLD            "unknown_object_threshold"
#define XML_ATTRIBUTE_SLIDING_WINDOW_SIZE                 "sliding_window_size"
#define XML_ATTRIBUTE_ENABLE_CLUSTERING                   "enable_clustering"
#define XML_ATTRIBUTE_ENABLE_INCREMENTAL_LEARNING         "enable_incremental_learning"
#define XML_ATTRIBUTE_ENABLE_VISUALIZATION                "enable_visualization"
#define XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MIN             "radius_adaptation_r_min"
#define XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MAX             "radius_adaptation_r_max"
#define XML_ATTRIBUTE_RADIUS_ADAPTATION_A                 "radius_adaptation_A"
#define XML_ATTRIBUTE_RADIUS_ADAPTATION_K                 "radius_adaptation_K"

namespace cop
{
  CREATE_NAMED_DESCRIPTOR(ObjectOfDailyUseDescriptor, "ObjectOfDailyUseDescriptor", "NoClass", DESCRIPTOR_NAMEDCLASS, std_msgs::String)
}
using namespace cop;


CopOduRefine::CopOduRefine() : m_odufinder(NULL)
{
  printf("CopOduLocalization created\n");
}

CopOduRefine::~CopOduRefine(void)
{
}


XMLTag* CopOduRefine::Save()
{
  XMLTag* tag = new XMLTag(GetName());


  tag->AddProperty(XML_ATTRIBUTE_COMMAND, m_odufinder->command);
  tag->AddProperty(XML_ATTRIBUTE_DATABASE_LOCATION, m_odufinder->database_location);
  tag->AddProperty(XML_ATTRIBUTE_IMAGES_DIRECTORY, m_odufinder->images_directory);
  tag->AddProperty(XML_ATTRIBUTE_IMAGES_FOR_VISUALIZATION_DIRECTORY, m_odufinder->images_for_visualization_directory);


  tag->AddProperty(XML_ATTRIBUTE_VOTES_COUNT, m_odufinder->votes_count);
  tag->AddProperty(XML_ATTRIBUTE_TREE_K, m_odufinder->tree_k);
  tag->AddProperty(XML_ATTRIBUTE_TREE_LEVELS, m_odufinder->tree_levels);
  tag->AddProperty(XML_ATTRIBUTE_MIN_CLUSTER_SIZE, m_odufinder->min_cluster_size);
  tag->AddProperty(XML_ATTRIBUTE_UNKNOWN_OBJECT_THRESHOLD, m_odufinder->unknown_object_threshold);
  tag->AddProperty(XML_ATTRIBUTE_SLIDING_WINDOW_SIZE, m_odufinder->sliding_window_size);

  tag->AddProperty(XML_ATTRIBUTE_ENABLE_CLUSTERING, m_odufinder->enable_clustering);
  tag->AddProperty(XML_ATTRIBUTE_ENABLE_INCREMENTAL_LEARNING, m_odufinder->enable_incremental_learning);
  tag->AddProperty(XML_ATTRIBUTE_ENABLE_VISUALIZATION, m_odufinder->enable_visualization);

  tag->AddProperty(XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MIN, m_odufinder->radius_adaptation_r_min);
  tag->AddProperty(XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MAX, m_odufinder->radius_adaptation_r_max);
  tag->AddProperty(XML_ATTRIBUTE_RADIUS_ADAPTATION_A, m_odufinder->radius_adaptation_A);
  tag->AddProperty(XML_ATTRIBUTE_RADIUS_ADAPTATION_K, m_odufinder->radius_adaptation_K);

  return tag;
}


void CopOduRefine::SetData(XMLTag* tag)
{
  this->m_odufinder = new ODUFinder();
  if(tag != NULL)
  {
    m_odufinder->command = tag->GetProperty(XML_ATTRIBUTE_COMMAND, std::string("/load"));
    m_odufinder->database_location = tag->GetProperty(XML_ATTRIBUTE_DATABASE_LOCATION, std::string("germandeli_big/"));
    m_odufinder->images_directory = tag->GetProperty(XML_ATTRIBUTE_IMAGES_DIRECTORY, std::string("data_big/"));
    m_odufinder->images_for_visualization_directory = tag->GetProperty(XML_ATTRIBUTE_IMAGES_FOR_VISUALIZATION_DIRECTORY, std::string("data_big/"));
    m_odufinder->votes_count = tag->GetPropertyInt(XML_ATTRIBUTE_VOTES_COUNT, 10);
    m_odufinder->tree_k = tag->GetPropertyInt(XML_ATTRIBUTE_TREE_K,5);
    m_odufinder->tree_levels = tag->GetPropertyInt(XML_ATTRIBUTE_TREE_LEVELS,5);
    m_odufinder->min_cluster_size = tag->GetPropertyInt(XML_ATTRIBUTE_MIN_CLUSTER_SIZE,30);
    m_odufinder->unknown_object_threshold = tag->GetPropertyDouble(XML_ATTRIBUTE_UNKNOWN_OBJECT_THRESHOLD,0.3);
    m_odufinder->sliding_window_size = tag->GetPropertyInt(XML_ATTRIBUTE_SLIDING_WINDOW_SIZE,10);

    m_odufinder->enable_clustering = tag->GetPropertyInt(XML_ATTRIBUTE_ENABLE_CLUSTERING,0);
    m_odufinder->enable_incremental_learning = tag->GetPropertyInt(XML_ATTRIBUTE_ENABLE_INCREMENTAL_LEARNING,0);
    m_odufinder->enable_visualization = tag->GetPropertyInt(XML_ATTRIBUTE_ENABLE_VISUALIZATION,0);

    m_odufinder->radius_adaptation_r_min = tag->GetPropertyDouble(XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MIN,200.0);
    m_odufinder->radius_adaptation_r_max = tag->GetPropertyDouble(XML_ATTRIBUTE_RADIUS_ADAPTATION_R_MAX,600.9);
    m_odufinder->radius_adaptation_A = tag->GetPropertyDouble(XML_ATTRIBUTE_RADIUS_ADAPTATION_A,800.0);
    m_odufinder->radius_adaptation_K = tag->GetPropertyDouble(XML_ATTRIBUTE_RADIUS_ADAPTATION_K,0.02);
  }
  this->m_odufinder->start();
}


// Public attribute accessor methods
//
Descriptor*  CopOduRefine::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  ObjectOfDailyUseDescriptor*  namedClass = NULL;
  /*SegmentPrototype* segment = (SegmentPrototype*)object.GetElement(0, DESCRIPTOR_SEGMPROTO);*/
  try
  {
    ScopedCvMat_t image (sensors, ReadingType_IplImage);

    RelPose* sensor_pose = image.sensor_pose_at_capture_time;

    IplImage gray_ipl;
    int r1 = 0, r2 = image.calib.height, c1 = 0, c2 = image.calib.width;
    RelPoseToRect(pose, sensor_pose->m_uniqueID, &(image.calib), r1, c1, r2, c2);
    printf("Converted pose to rect: %d %d %d %d\n", r1, c1, r2, c2);
    CvMat *q1, q1stub, in;
    in =  (*image);
    q1 = cvGetSubRect( &in, &q1stub, cvRect(c1,r1,c2-c1,r2-r1) );
    if((*image).channels() > 1)
    {
      cv::Mat gray;
      cv::Mat test(&q1stub, false);
      cvtColor( test, gray, CV_BGR2GRAY );
      gray_ipl = gray;
    }
    else
    {
       gray_ipl = cv::Mat(&q1stub, false);
    }

    printf("Call odu finder with %d %d\n", gray_ipl.height, gray_ipl.width);
    std::string result = m_odufinder->process_image(&gray_ipl);
    if(result.length() > 0)
    {
      namedClass = new ObjectOfDailyUseDescriptor();
      std_msgs::String res;
      res.data = result;
      namedClass->SetContent(res);
      namedClass->SetClass(result);
    }
  }
  catch(const char* exception)
  {
    printf("CopOduRefine failed: %s\n", exception);
  }
  return namedClass;
}

double CopOduRefine::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  printf("CopOduRefine::CheckSignature\n");
  if(object.GetElement(0, DESCRIPTOR_NAMEDCLASS) == NULL)
  {
    printf("It has no Named Class\n");
    for(size_t i = 0; i < sensors.size(); i++)
    {
      printf("Is Sensor %s a camera?", sensors[i]->GetSensorID().c_str());
      if(sensors[i]->IsCamera())
      {
        printf("Yes\n");
        return 1.0;
      }
    }
  }
  printf("No\n");
  return 0.0;
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_REGISTER_CLASS(ObjectOfDailyUseDescriptor, cop::ObjectOfDailyUseDescriptor, Descriptor);
PLUGINLIB_REGISTER_CLASS(CopOduRefine, cop::CopOduRefine, RefineAlgorithm);

