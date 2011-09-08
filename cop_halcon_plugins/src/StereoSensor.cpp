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


#include "Camera.h"
#include "StereoSensor.h"
#include "XMLTag.h"
#include "cpp/HalconCpp.h"
#include <sensor_msgs/PointCloud.h>

#define XML_PROPERTY_SENSORNAME_RIGHT "SensorName_CamRight"
#define XML_PROPERTY_SENSORNAME_LEFT "SensorName_CamLeft"
#define XML_PROPERTY_RELPOSEFILENAME "RelPoseFileName"

extern volatile bool g_stopall;


// Procedures
void binocular_xyz_proc (Halcon::Hobject ImageL, Halcon::Hobject MapL, Halcon::Hobject ImageR,
    Halcon::Hobject MapR, Halcon::Hobject *X, Halcon::Hobject *Y, Halcon::Hobject *Z,
    Halcon::Hobject *RegionZ, Halcon::HTuple CamParamRect1, Halcon::HTuple CamParamRect2,
    Halcon::HTuple RelPoseRect)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  ImageMappedL, ImageMappedR, Rectangle;
  Hobject  ImageReducedR, ImageReducedL, ImageEquHistoR, ImageEquHistoL;
  Hobject  Disparity, Score, RegionComplement;

  map_image(ImageL, MapL, &ImageMappedL);
  map_image(ImageR, MapR, &ImageMappedR);
  gen_rectangle1(&Rectangle, 200, 200, 1600, 2200);
  reduce_domain(ImageMappedR, Rectangle, &ImageReducedR);
  reduce_domain(ImageMappedL, Rectangle, &ImageReducedL);
  equ_histo_image(ImageReducedR, &ImageEquHistoR);
  equ_histo_image(ImageReducedL, &ImageEquHistoL);
  binocular_disparity_mg(ImageEquHistoL, ImageEquHistoR, &Disparity, &Score, 1, 30,
      1, -100, "false", "default_parameters", "fast");
  disparity_image_to_xyz(Disparity, &(*X), &(*Y), &(*Z), CamParamRect1, CamParamRect2,
      RelPoseRect);
  threshold((*Z), &(*RegionZ), 0.01, 10);
  complement((*RegionZ), &RegionComplement);
  overpaint_region((*Z), RegionComplement, 0, "fill");
  overpaint_region((*Y), RegionComplement, 0, "fill");
  overpaint_region((*X), RegionComplement, 0, "fill");
  return;
}

using namespace cop;

StereoSensor::StereoSensor() :
  m_camRight(NULL),
  m_nameRight("uninit_1234679_right"),
  m_camLeft(NULL),
  m_nameLeft("uninit_1234679_left"),
  m_mapInitialized(false),
  m_advertised(false),
  m_lastReading(NULL)
{

}


StereoSensor::~StereoSensor()
{
  printf("Crashing?");
}

bool StereoSensor::Start()
{
  return true;
}

XMLTag* StereoSensor::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  Sensor::SaveTo(tag);
  tag->AddProperty(XML_PROPERTY_SENSORNAME_RIGHT, m_nameRight);
  tag->AddProperty(XML_PROPERTY_SENSORNAME_LEFT, m_nameLeft);
  return tag;
}
void StereoSensor::SetData(XMLTag* tag)
{
  m_nameRight = tag->GetProperty(XML_PROPERTY_SENSORNAME_RIGHT, m_nameRight);
  m_nameLeft = tag->GetProperty(XML_PROPERTY_SENSORNAME_LEFT, m_nameLeft);
  std::string relPoseFileName = tag->GetProperty(XML_PROPERTY_RELPOSEFILENAME, "");
  try
  {
    Halcon::read_pose(relPoseFileName.c_str(),  &m_relPoseLeft2Right);
  }
  catch(...)
  {
    printf("StereoSensor::SetData: Error reading relaitive pose  %s -> %s\n", m_nameLeft.c_str(), m_nameRight.c_str());
  }
}


bool StereoSensor::RequiresSensorList()
{
  if(m_camRight == NULL || m_camLeft == NULL)
    return true;
  else
    return false;
}
/**
*  SetSensorList
*  @brief Stereo works as a composition of names sensors
*/
void StereoSensor::SetSensorList(std::vector<Sensor*> cameras)
{
  m_mapInitialized = false;
  for(std::vector<Sensor*>::const_iterator it = cameras.begin();
            it !=  cameras.end(); it++)
  {
    if((*it)->IsCamera() )
    {
      if( (*it)->GetSensorID().compare(m_nameLeft) == 0 )
      {
        m_camLeft = (Camera*)(*it);
      }
      if( (*it)->GetSensorID().compare(m_nameRight) == 0 )
      {
        m_camRight = (Camera*)(*it);
      }
    }
  }
}

unsigned long GetNanoSecAge(Halcon::Hobject img_l)
{
  Halcon::HTuple msec, sec, min, hour, day, yday, month, year;
  Halcon::get_image_time(img_l, &msec, &sec, &min, &hour, &day, &yday, &month, &year);
  return msec[0].I()  * 10000000 + sec[0].I() * 1000000000;
}

/**
* GetReading
* @param Frame frame number, to specify an offset or a specific file
* @throws char* with an error message in case of failure
*/
Reading* StereoSensor::GetReading(const long &Frame)
{
  Reading* result = NULL;
  /** TODO buffer!: this only implements the creation on query*/
  if(m_camRight != NULL && m_camLeft != NULL && !g_stopall)
  {
    Image* rightImg = (Image*)m_camRight->GetReading(Frame);
    Image* leftImg = (Image*)m_camLeft->GetReading(Frame);
    try
    {
      if(!m_mapInitialized)
      {
        Halcon::HTuple leftCalibBef = m_camLeft->m_calibration.CamParam();
        Halcon::HTuple  rightCalibBef = m_camRight->m_calibration.CamParam();
        Halcon::gen_binocular_rectification_map(&m_mapLeft, &m_mapRight,
                                                leftCalibBef, rightCalibBef,
                                                 m_relPoseLeft2Right,
                                                 1, "geometric", "bilinear",
                                           &m_calibLeft, &m_calibRight, &camposerectLeft,
                                            &camposerectRight, &m_relPoseRect);
        m_mapInitialized = true;
      }
      Halcon::Hobject X,Y,Z, region, img_l;
      Halcon::HTuple num_channels, h,w,pr,pg,pb,t, px, py, pz;
      img_l = *rightImg->GetHImage();
      binocular_xyz_proc(*leftImg->GetHImage(), m_mapLeft, img_l, m_mapRight, &X, &Y, &Z, &region, m_calibLeft, m_calibRight, m_relPoseRect);
      Halcon::count_channels(img_l, &num_channels);

      Halcon::get_image_pointer1(X,&px,&h,&w,&t);
      Halcon::get_image_pointer1(Y,&py,&h,&w,&t);
      Halcon::get_image_pointer1(Z,&pz,&h,&w,&t);

      if(num_channels[0].I() == 3)
        Halcon::get_image_pointer3(img_l,&pr,&pg,&pb, &h,&w,&t);
      else
        Halcon::get_image_pointer1(img_l,&pr,&h,&w,&t);
      m_lastReading = new SwissRangerReading();

      m_lastReading->m_image.header.stamp = ros::Time::now();
      m_lastReading->m_image.header.frame_id = "/base_link";

      // resize channels and copy their names
      m_lastReading->m_image.channels.resize (num_channels[0].I());
      for (unsigned int i = 0; i < m_lastReading->m_image.channels.size(); i++)
        m_lastReading->m_image.channels[i].name = i == 0 ? "R" : i == 1 ? "G" : "B";
      unsigned char* image_pointer_red = (unsigned char*)pr[0].L();
      unsigned char* image_pointer_green = NULL;
      unsigned char* image_pointer_blue = NULL;
      if(num_channels[0].I() == 3)
      {
        image_pointer_green = (unsigned char*)pg[0].L();
        image_pointer_blue = (unsigned char*)pb[0].L();
      }
      float* image_pointer_x = (float*)px[0].L();
      float* image_pointer_y = (float*)py[0].L();
      float* image_pointer_z = (float*)pz[0].L();
      // go through all points
      Halcon::HTuple r, c;
      Halcon::get_region_points(region, &r, &c);
      int num = r.Num();
      int num_final = num;
      for (int iter_p = 0; iter_p < num; iter_p++)
      {
          // select all points that are within in the specified points
          unsigned int index = r[iter_p].I()*w[0].I() + c[iter_p].I();
          geometry_msgs::Point32 point;
          point.x = image_pointer_x[index];
          point.y = image_pointer_y[index];
          point.z = image_pointer_z[index];
          if(point.z < 0.001 || point.z > 2.50)
          {
          num_final--;
            continue;
          }
          m_lastReading->m_image.points.push_back (point);

          m_lastReading->m_image.channels[0].values.push_back (image_pointer_red[index]);
          if(num_channels[0].I() == 3)
          {
            m_lastReading->m_image.channels[1].values.push_back (image_pointer_green[index]);
            m_lastReading->m_image.channels[2].values.push_back (image_pointer_blue[index]);
          }
      }
      PushBack(m_lastReading);
      result = m_lastReading;
      /**  TODO: fill result
        result = new SwissRangerReading, DisparityBla, PointCloud, ...();
      */
    }
    catch(...)
    {
      printf("Error in StereoSensor::GetReading !\n\n");
    }
    rightImg->Free();
    leftImg->Free();
  }
  return result;
}

void StereoSensor::Show(const long frame)
{
  if(!m_advertised)
  {
    ros::NodeHandle nh;
    m_cloud_pub = nh.advertise<sensor_msgs::PointCloud> ("cop_debug_out_stereo_sensor", 1);
    m_advertised = true;
  }
  if(m_lastReading == NULL)
  {
    GetReading(-1);
  }
  m_cloud_pub.publish(m_lastReading->m_image);
  /** TODO
        publish results on a topic?,
        static topic?
        topic name as parameter / rosparameter?
        + concept of rosparam <-> XMLTag
  */

}


bool StereoSensor::CanSee(RelPose &pose) const
{
  if(m_camRight != NULL && m_camLeft != NULL && !g_stopall)
    return m_camRight->CanSee(pose) && m_camLeft->CanSee(pose);
  else
    return false;
}


