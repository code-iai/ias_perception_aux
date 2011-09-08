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
                        CameraDriver.cpp - Copyright klank

**************************************************************************/

#include "CameraDriver.h"
#include "cpp/HalconCpp.h"
#include "XMLTag.h"

#include "BoostUtils.h"



using namespace cop;

extern volatile bool g_stopall;

// Constructors/Destructors
//

#define XML_ATTRIBUTE_HASPTU "HasPTU"
#define XML_ATTRIBUTE_ISSTOC "IsSTOC"
#define XML_ATTRIBUTE_ISYUV "IsYUV"
#define XML_ATTRIBUTE_GRABBERNAME "GrabberName"
#define XML_ATTRIBUTE_CAMERATYPE "CameraType"
#define XML_ATTRIBUTE_CAMCOLOR "CamColor"
#define XML_ATTRIBUTE_IMGWIDTH "SetImageWidth"
#define XML_ATTRIBUTE_IMGHEIGHT "SetImageHeight"
#define XML_ATTRIBUTE_PANTOTILTHEIGHT "PanToTiltHeight"
#define XML_ATTRIBUTE_PANTOTILTWIDTH  "PanToTiltWidth"
#define XML_ATTRIBUTE_TILTTOENDHEIGHT "TiltToEndHeight"
#define XML_ATTRIBUTE_TILTTOENDWIDTH  "TiltToEndWidth"
#define XML_ATTRIBUTE_CAMPORT         "CamPort"
#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"
#define XML_ATTRIBUTE_HRESOLUTION     "Hresolution"
#define XML_ATTRIBUTE_VRESOLUTION     "Vresolution"
#define XML_ATTRIBUTE_STARTROW        "Startrow"
#define XML_ATTRIBUTE_STARTCOLUMN     "Startcolumn"
#define XML_ATTRIBUTE_FIELD           "Field"
#define XML_ATTRIBUTE_BITSPERCHANNEL  "Bitsperchannel"
#define XML_ATTRIBUTE_COLORSPACE       "Colospace"
#define XML_ATTRIBUTE_GENERIC         "Generic"
#define XML_ATTRIBUTE_DEVICE          "Device"
#define XML_ATTRIBUTE_EXTERNALTRIGGER "Externaltrigger"
#define XML_ATTRIBUTE_LINEIN          "Linein"

CameraDriver::CameraDriver()
{
}

void CameraDriver::SetData( XMLTag* ConfigFile)
{
  Camera::SetData(ConfigFile);
  m_hasPTU = (false);
  m_isSTOC = (false);
#ifdef PTU_USED
  m_ptuClient = (NULL);
#endif
  m_port = (0);
  m_imageWidth = (0);
  m_imageHeight = (0);
  m_fg = (NULL);
  m_grabbing = (false);
  m_isYUV = (false);


#ifdef _DEBUG
  printf("In CameraDriverConstructor:\n");
#endif
  try
  {
    if(ConfigFile != NULL)
    {
      try
      {
        XMLTag* tag = ConfigFile->GetChild(XML_NODE_RELPOSE);
        if(tag != NULL)
        {
          m_relPose = RelPoseFactory::FRelPose(tag);
          if(m_relPose != NULL)
          {
#ifdef _DEBUG
          printf("  CD: Camera is localized at position %ld\n", m_relPose->m_uniqueID);
#endif
          }
#ifdef _DEBUG
          else
            printf("  CD: Camera has no location\n");
#endif
        }
        else
        {
#ifdef _DEBUG
          printf("  CD: Camera has no location\n");
#endif
        }
      }
      catch(...)
      {
        printf("  CD: error Reading RelPose\n");
      }
      m_hasPTU = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HASPTU) != 0;
      printf("HASPTU:=%s\n",ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HASPTU) ? "true" : "false");
      m_isSTOC =ConfigFile->GetPropertyInt(XML_ATTRIBUTE_ISSTOC) != 0;
      printf("IsStoc:=%s\n",ConfigFile->GetPropertyInt(XML_ATTRIBUTE_ISSTOC) ? "true" : "false");
      m_isYUV = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_ISYUV) != 0;
      if(m_hasPTU)
      {
#ifdef PTU_USED
#ifdef _DEBUG
        printf("Init PTU Client\n");
#endif
        try
        {
          /* PTU Client init: the first four parameter are the metrics of the PTU, TODO get those from configFile*/
          double heightPanToTilt = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_PANTOTILTHEIGHT);
          double lengthPanToTilt = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_PANTOTILTWIDTH);
          double heightTiltToEnd = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_TILTTOENDHEIGHT);
          double lengthTiltToEnd = ConfigFile->GetPropertyDouble(XML_ATTRIBUTE_TILTTOENDWIDTH);
          m_ptuClient = new PTUClient(heightPanToTilt,lengthPanToTilt,
                        heightTiltToEnd,lengthTiltToEnd ,
                        RelPoseFactory::FRelPose(ConfigFile->GetChild(XML_NODE_RELPOSE)),
                        "roboradig30", 6665, 0);
          printf("Make a relpose from PTU\n");
          m_relPose = RelPoseFactory::FRelPose(*m_ptuClient);
        }
        catch(...)
        {
          printf("PTU Driver could not instantiated\n");
          m_ptuClient = NULL;
        }
  #endif
      }
#ifdef PTU_USED
      if(m_ptuClient == NULL)
  #endif
      {
        m_relPose = RelPoseFactory::FRelPose(ConfigFile->GetChild(XML_NODE_RELPOSE));
      }
      m_grabberName = ConfigFile->GetProperty(XML_ATTRIBUTE_GRABBERNAME);
      if(m_grabberName.length() == 0)
      {
        m_grabberName = "1394IIDC";
      }
      m_stCameraType = ConfigFile->GetProperty(XML_ATTRIBUTE_CAMERATYPE, "default");
      m_isColor = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_CAMCOLOR) != 0;
      m_imageWidth = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_IMGWIDTH);
      m_imageHeight = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_IMGHEIGHT);
      m_port = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_CAMPORT, -1);
      m_hresolution     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_HRESOLUTION    , 1);
      m_vresolution     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_VRESOLUTION    , 1);
      m_startRow        = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_STARTROW       );
      m_startColumn     = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_STARTCOLUMN    );
      m_field           = ConfigFile->GetProperty(XML_ATTRIBUTE_FIELD          , "default");
      m_BitsPerChannel  = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_BITSPERCHANNEL , -1);
      m_colorSpace      = ConfigFile->GetProperty(XML_ATTRIBUTE_COLORSPACE     , m_isColor ? "yuv" : "default");
      m_generic            = ConfigFile->GetProperty(XML_ATTRIBUTE_GENERIC           , "-1");
      m_externalTrigger = ConfigFile->GetProperty(XML_ATTRIBUTE_EXTERNALTRIGGER, "default");
      m_device          = ConfigFile->GetProperty(XML_ATTRIBUTE_DEVICE, "default");
      m_lineIn          = ConfigFile->GetPropertyInt(XML_ATTRIBUTE_LINEIN         , -1);

    }
  }
  catch(char const* text)
  {
    printf("Error Loading CameraDriver: %s\n", text);
  }
#ifdef _DEBUG
  printf("Successfully initialized Camera Driver\n");
#endif
}

void CameraDriver::threadfunc()
{
  int i = 0;
  int nErrorCount = 0;
  boost::xtime t;
  boost::xtime_get(&t, boost::TIME_UTC);
  /*#define GRAY_IMAGE 0
    #define RGB_IMAGE 1
    #define YUV_IMAGE 2
    #define HSV_IMAGE 4
    #define GRAY_DISPARITY_IMAGE 8
    #define YUV_DISPARITY_IMAGE 16*/
  int image_type = m_isYUV ? (m_isSTOC ? YUV_DISPARITY_IMAGE : YUV_IMAGE ): (m_isSTOC ? GRAY_DISPARITY_IMAGE : GRAY_IMAGE);
  if( m_colorSpace.compare("default") != 0)
  {
        if(m_colorSpace.compare("rgb") != 0)
            image_type = RGB_IMAGE;
        else if(m_colorSpace.compare("yuv") != 0)
            image_type = YUV_IMAGE;
        else if(m_colorSpace.compare("gray") != 0)
            image_type = GRAY_IMAGE;
  }
  /* Special case: handles for diorect show are unique for a thread ...*/
  try
  {
  if(this->m_grabberName.compare("DirectShow") == 0)
  {
        printf("open_frame_grabber(%s, %d, %d, %d, %d, %d, %d, %s, %d, %s, %s , %s, %s, %s, %d, %d)",m_grabberName.c_str(),
                                   m_hresolution,
                                   m_vresolution,
                                   m_imageWidth,
                                   m_imageHeight,
                                   m_startRow,
                                   m_startColumn,
                                   m_field.c_str(),
                                   m_BitsPerChannel,
                                   m_colorSpace.c_str(),
                                   m_generic.c_str(),
                                   m_externalTrigger.c_str(),
                                   m_stCameraType.c_str(),
                                   m_device.c_str(),
                                   m_port,
                                   m_lineIn);
        //Halcon::set_system("do_low_error", "true");
        m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                   m_hresolution,
                                   m_vresolution,
                                   m_imageWidth,
                                   m_imageHeight,
                                   m_startRow,
                                   m_startColumn,
                                   m_field.c_str(),
                                   m_BitsPerChannel,
                                   m_colorSpace.c_str(),
                                   ((m_generic.compare("-1") == 0) ? Halcon::HTuple(-1) : Halcon::HTuple(m_generic.c_str())),
                                   m_externalTrigger.c_str(),
                                   m_stCameraType.c_str(),
                                   m_device.c_str(),
                                   m_port,
                                   m_lineIn);
  }

    Halcon::grab_image_start (m_fg->GetHandle(), -1);
  }
  catch(Halcon::HException ex)
  {
    printf("CameraDriver failed in threadfunc: %s\n Restart Camera\n", ex.message);
    ROS_ERROR("Camera %s must be restarted, failed to grab images\n", GetSensorID().c_str());
    m_grabbing = false;
    return;
  }
  catch(...)
  {
    printf("CameraDriver failed in threadfunc.\n Restart Camera\n");
    ROS_ERROR("Camera %s must be restarted, failed to grab images\n", GetSensorID().c_str());
    m_grabbing = false;
    return;
  }
  while(m_grabbing && !g_stopall)
  {
    if(m_fg != NULL)
    {
      Halcon::Hobject* obj = new Halcon::Hobject();
      if(obj == NULL)
      {

      }
      //printf("Grabbing\n");
      try
      {
        try
        {
          Halcon::grab_image_async(obj, m_fg->GetHandle(), 50);
        }
        catch(Halcon::HException ex)
        {
          ROS_WARN("Error grabbing images: %s, retrying once in a few ms\n", ex.message);
          if(m_grabbing == false)
            return;
          Sleeping(5);
          try
          {
            Halcon::grab_image_async(obj, m_fg->GetHandle(), -1);
          }
          catch(...)
          {
           if(m_grabbing == false)
             return;
           ROS_ERROR("Major issue with camera %s", GetSensorID().c_str());
           delete m_fg;
           m_fg = NULL;
           ROS_INFO("Restarting CameraDriver of  %s in a while", GetSensorID().c_str());
           Sleeping(50);
           ROS_INFO("Restarting now");
           m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_generic.compare("-1") == 0 ? Halcon::HTuple(-1) : Halcon::HTuple(m_generic.c_str()),
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);
           if(m_grabberName.compare("GigEVision") == 0)
           {
            m_fg->SetFramegrabberParam("GtlGVSPDiscardIncompleteBuffers", "enable");
            m_fg->SetFramegrabberParam("AcquisitionFrameRateAbs", 10); //TODO framerate

           }

            Halcon::grab_image_start (m_fg->GetHandle(), -1);
            continue;

          }
        }
        if(m_isSTOC)
        {
          Halcon::Hobject tmp1, tmp2;
          Halcon::decompose3(*obj, obj, &tmp1, &tmp2);
        }
        if(m_calibration.m_radialDistortionHandling)
        {
          //Halcon::HTuple chan, width, height, pointer;
          //Halcon::count_channels(*m_calibration.m_radialDistMap, &chan);
          //printf("Undistortion img: \n");
          //Halcon::disp_obj(*(m_calibration.m_radialDistMap), GetWindow()->WindowHandle());
           try
           {
             Halcon::map_image(*obj, *m_calibration.m_radialDistMap, obj);
           }
           catch(Halcon::HException ex)
           {
             ROS_ERROR("Error acquiring images: %s\n", ex.message);
             delete obj;
             if(nErrorCount > 5)
             {
               delete m_fg;
               m_fg = NULL;
               if(m_grabbing != false)
                  Start();
               return;
             }
             nErrorCount++;
             continue;
           }
          //GetWindow()->Click();
          //Halcon::disp_obj(*(m_calibration.m_radialDistMap), GetWindow()->WindowHandle());
        }
        i++;
      }
      catch(Halcon::HException ex)
      {
        ROS_ERROR("Error in Grabbing: %s\n", ex.message);
        break;
      }
      Image* img = new Image(obj, image_type, m_relPose);
      PushBack(img);
    }
    while(m_images.size() > m_max_cameraImages)
    {
      if(DeleteReading())
        continue;
      else
      {
        ROS_DEBUG("Camera Driver: Could not delete an image!\n");
        break;
      }
    }
    Sleeping(10);
  }
  m_grabbing = false;
  ROS_INFO("End Camera Thread\n");
}

CameraDriver::~CameraDriver ( )
{
  Stop();
  delete m_fg;
  m_fg = NULL;
#ifdef PTU_USED
  delete m_ptuClient;
#endif
}

//
// Methods
//
Reading* CameraDriver::GetReading(const long &Frame)
{
  if(!m_grabbing)
  {
    Start();
    BOOST(boost::xtime t);
    BOOST(boost::xtime_get(&t, boost::TIME_UTC));
    BOOST(t.sec += 1);
    BOOST(boost::thread::sleep(t));
  }
  if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
  {
    if(m_grabbing)
    {
      while(m_grabbing && ((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0))
      {
        BOOST(boost::xtime t);
        BOOST(boost::xtime_get(&t, boost::TIME_UTC));
        BOOST(t.sec += 1);
        BOOST(boost::thread::sleep(t));
        printf("waiting for %s to start grabbing (Grabbing: %s, NumImages: %ld)\n", GetSensorID().c_str(), m_grabbing ? "true" : false, m_images.size());
      }
      printf("Got a new image: %d\n", (int)m_images.size());
    }
    if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
    {
      printf("unexpected error\n");
      throw "Asking for images from a camera that has no images";
    }
  }
  if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
  {
    return GetReading_Lock(m_images.size() -1);
    /*return m_images[m_images.size() -1];*/
  }
  return GetReading_Lock(Frame - m_deletedOffset);
  /*return m_images[Frame - m_deletedOffset];*/
}

void CameraDriver::SetExposure(int exposure)
{
  if(m_grabbing == true && m_fg != NULL && exposure > 5000 && exposure < 100000)
  {
    try
    {
    m_fg->SetFramegrabberParam("ExposureTimeAbs", exposure);
    }
    catch(Halcon::HException ex)
    {
      ROS_ERROR("Exceptiuon during setting of new exposure: %s in camera %s", ex.message, m_device.c_str());
    }
  }
  else
  {
    ROS_ERROR("Ignoring newly set exposure in Camera: %s", m_device.c_str());
  }
}


#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/*void Camera::PublishImageOnRosTopic(const Image& img)
{
  sensor_msgs::Image msg;

}*/
bool CameraDriver::Start()
{
  if(m_grabberName.compare("1394IIDC") == 0 || m_grabberName.compare("GigEVision") == 0)
  {
#ifdef _DEBUG

    if(m_grabberName.compare("1394IIDC") == 0)
      printf("Trying to open 1394-Camera\n");
    else
      printf("Trying to open GigEVision-Camera\n");
#endif
    if(m_stCameraType.length() == 0)
      m_stCameraType = "7:0:0";
#ifdef _DEBUG
    printf("Camera Type: %s\n", m_stCameraType.c_str());
#endif
    try
    {
      printf("Entering Start\n");
      m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_generic.compare("-1") == 0 ? Halcon::HTuple(-1) : Halcon::HTuple(m_generic.c_str()),
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);
      printf("Opened\n");
      if(m_grabberName.compare("GigEVision") == 0)
      {
         m_fg->SetFramegrabberParam("GtlGVSPDiscardIncompleteBuffers", "enable");
         m_fg->SetFramegrabberParam("AcquisitionFrameRateAbs", 10); //TODO framerate

      }
      /*if(m_stCameraType.c_str()[0] == '7')
      {
        m_fg->SetFramegrabberParam("packet_size", 8192);
        if(m_imageWidth > 0)
        {
          printf("Force Image Width to %d\n", m_imageWidth);
          m_fg->SetFramegrabberParam("horizontal_resolution",	m_imageWidth	);
        }
        if(m_imageHeight > 0)
        {
          printf("Force Image Height to %d\n", m_imageHeight);
          m_fg->SetFramegrabberParam("vertical_resolution",		m_imageHeight	);
        }
        Halcon::HTuple tup = m_fg->GetFramegrabberParam("frame_rate");
        Halcon::HTuple tup2 = m_fg->GetFramegrabberParam("packet_size");
        printf("... done: (%f fps in %d byte) \n", tup[0].D(), tup2[0].L());
      }
      else
        printf("... done \n");*/
    }
    catch(Halcon::HException &ex)
    {
      ROS_ERROR("Error opening camera: %s, DeviceName: %s\n", ex.message, m_device.c_str());
      m_grabbing = false;
      return false;
    }
    catch(...)
    {
      m_fg = NULL;
      ROS_ERROR("Camera not connected, or errors in driver configuration\n");
      m_grabbing = false;
      return false;
    }
    //mfg.OpenFramegrabber();
  }
  else if(m_grabberName.compare("LeutronVision") == 0)
  {
    //TODO
  }
  else if(m_grabberName.compare("DirectShow") == 0)
  {
    /* Windows handle are not usable in different threads? so build m_fg in the thread*/
  }
  else
  {
    try
    {
      printf("open_frame_grabber(%s, %d, %d, %d, %d, %d, %d, %s, %d, %s, %s, %s, %s, %s, %d, %d)",m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_generic.c_str(),
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);

      m_fg = new Halcon::HFramegrabber(m_grabberName.c_str(),
                                       m_hresolution,
                                       m_vresolution,
                                       m_imageWidth,
                                       m_imageHeight,
                                       m_startRow,
                                       m_startColumn,
                                       m_field.c_str(),
                                       m_BitsPerChannel,
                                       m_colorSpace.c_str(),
                                       m_generic.compare("-1") == 0 ? Halcon::HTuple(-1) :Halcon::HTuple( m_generic.c_str()),
                                       m_externalTrigger.c_str(),
                                       m_stCameraType.c_str(),
                                       m_device.c_str(),
                                       m_port,
                                       m_lineIn);
    }
    catch(Halcon::HException &ex)
    {
      printf("Error opening camera: %s, DeviceName: %s\n", ex.message, m_device.c_str());
      m_grabbing = false;
      return false;
    }

  }
  m_grabbing = true;
  m_grabbingThread = new thread(bind(&CameraDriver::threadfunc, this)) ;
  return true;
}
bool CameraDriver::Stop()
{
  m_grabbing = false;
  printf("\nTrying to Stop Cemaera Grabbing Thread (Sleeping 1s)\n");
  Sleeping(1000);
  printf("\nJoining Camera Grabbing Thread \n");
  m_grabbingThread->join();
  Sensor::Stop();
  delete m_fg;
  //Halcon::close_all_framegrabbers();
  delete m_grabbingThread;
  return false;
}

XMLTag* CameraDriver::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  Camera::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_CALIBFILE, m_stCalibName);
  tag->AddProperty(XML_ATTRIBUTE_HASPTU, m_hasPTU ? 1 : 0);
  tag->AddProperty(XML_ATTRIBUTE_GRABBERNAME, m_grabberName);
  tag->AddProperty(XML_ATTRIBUTE_CAMERATYPE, m_stCameraType);
  tag->AddProperty(XML_ATTRIBUTE_CAMCOLOR, (int)m_isColor);
  tag->AddProperty(XML_ATTRIBUTE_IMGWIDTH, m_imageWidth);
  tag->AddProperty(XML_ATTRIBUTE_IMGHEIGHT, m_imageHeight);
  tag->AddProperty(XML_ATTRIBUTE_CAMPORT, m_port);

  tag->AddProperty(XML_ATTRIBUTE_HRESOLUTION    ,m_hresolution);
  tag->AddProperty(XML_ATTRIBUTE_VRESOLUTION    ,m_vresolution);
  tag->AddProperty(XML_ATTRIBUTE_STARTROW       ,m_startRow);
  tag->AddProperty(XML_ATTRIBUTE_STARTCOLUMN    ,m_startColumn);
  tag->AddProperty(XML_ATTRIBUTE_FIELD          ,m_field);
  tag->AddProperty(XML_ATTRIBUTE_BITSPERCHANNEL ,m_BitsPerChannel);
  tag->AddProperty(XML_ATTRIBUTE_COLORSPACE     ,m_colorSpace);
  tag->AddProperty(XML_ATTRIBUTE_GENERIC        ,m_generic);
  tag->AddProperty(XML_ATTRIBUTE_DEVICE         ,m_device);
  tag->AddProperty(XML_ATTRIBUTE_EXTERNALTRIGGER,m_externalTrigger);
  tag->AddProperty(XML_ATTRIBUTE_LINEIN         ,m_lineIn);

  return tag;
}


void CameraDriver::Show(const long frame)
{
  try
  {
    if(m_win == NULL)
      GetWindow();
    Image* img = GetImage(frame);
    if(img != NULL)
    {
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::HTuple p,t,height, width, chan;
      Halcon::get_image_pointer1(*obj, &p,&t,&width, &height);
      #ifdef _DEBUG
      printf("Showing with: height %d width %d\n", height[0].I(), width[0].I());
      #endif
      if(width[0].I() < 700)
        m_win->SetWindowExtents(20,10,(int)(width[0].I() * 1.5 ) + 10, (int)(height[0].I()* 1.5 ) + 20);
      else
        m_win->SetWindowExtents(20,10,(int)(width[0].I() * 0.5 ) + 10, (int)(height[0].I()* 0.5 ) + 20);

      m_win->SetPart(0,0,height, width);
      //TODO Check if this is not colored!


      Halcon::count_channels(*obj, &chan);
      if(chan[0].I() == 3 && m_isYUV)
      {
        Halcon::Hobject obj_temp1,obj_temp2,obj_temp3;
        Halcon::decompose3(*obj, &obj_temp1,&obj_temp2,&obj_temp3);
        Halcon::trans_to_rgb(obj_temp1,obj_temp2,obj_temp3,&obj_temp1,&obj_temp2,&obj_temp3, "yuv");
        Halcon::compose3(obj_temp1,obj_temp2,obj_temp3,&obj_temp1);
        Halcon::disp_obj(obj_temp1, m_win->WindowHandle());
      }
      else
        Halcon::disp_obj(*obj, m_win->WindowHandle());
      img->Free();
    }
  }
  catch(Halcon::HException ex)
  {
    printf("Showing not possible: %s \n", ex.message);
  }
}


sensor_msgs::Image  CameraDriverRelay ::ConvertData(Reading* img)
{
  Halcon::HTuple chan;
  Halcon::Hobject* obj;
  sensor_msgs::Image image;
  try
  {
    Image* img_cast = (Image*)img;
    obj = img_cast->GetHImage();
    Halcon::count_channels(*obj, &chan);
    image.header.stamp = ros::Time::now();
    if(m_relPose != NULL && m_relPose->m_mapstring.length() > 0)
      image.header.frame_id = m_relPose->m_mapstring;
    else
      image.header.frame_id = "/map";
  }
  catch(...)
  {
    throw "Error in initialization of CameraDriverRelay::CameraDriverRelay";
  }
  if(chan[0].I() == 3)
  {
    try
    {
      Hlong ptr_r, ptr_g, ptr_b, width, height;
      char type[100];
      Halcon::get_image_pointer3(*obj, &ptr_r, &ptr_g, &ptr_b, type, &width, &height);
      HBYTE* pbr = (HBYTE*)ptr_r;
      HBYTE* pbg = (HBYTE*)ptr_g;
      HBYTE* pbb = (HBYTE*)ptr_b;
      image.data.resize((size_t)(width * height * 3));
      image.step = 3*width;
      image.width = width;
      image.height = height;
      image.encoding = "rgb8";
      for(size_t row = 0; row < (unsigned)height; row++)
      {
        size_t index_ipl = row * width * 3;
        size_t index_himg = row * width;
        for(size_t col = 0; col <  (unsigned)width; col++)
        {
          size_t index_ipl_inner = index_ipl + col * 3;
          size_t index_himg_inner = index_himg + col;
          image.data[index_ipl_inner    ] = pbr[index_himg_inner];
          image.data[index_ipl_inner + 1] = pbg[index_himg_inner];
          image.data[index_ipl_inner + 2] = pbb[index_himg_inner];
        }
      }
    }
    catch(...)
    {
      throw "Error converting data in CameraDriverRelay::CameraDriverRelay";

    }
  }
  else
  {
    throw "Not yet implemented at <-- sensor_msgs::Image  CameraDriverRelay ::ConvertType(Reading* img); --> case: number of channels == 1";
  }
  return image;
}

XMLTag* CameraDriverRelay::Save()
{
  XMLTag* tag = CameraDriver::Save();
  tag->SetName(GetName());
  tag->AddProperty(XML_ATTRIBUTE_TOPICNAME, m_stTopic);
  tag->AddProperty(XML_ATTRIBUTE_RATE, m_rate);
  return tag;
}

void CameraDriverRelay::ModeCallback(const boost::shared_ptr<const vision_msgs::cop_camera_mode> &modestring)
{
  m_stEnvMode = (*modestring).mode;
  printf("Got a requests for mode %s on camer %s\n", GetSensorID().c_str(), m_stEnvMode.c_str());
  if(m_stEnvMode.compare(vision_msgs::cop_camera_mode::VERY_DARK_ENVIRONMENT) == 0)
  {
    printf("Setting Exposure time to %d microseconds\n", 50000);
    SetExposure(50000);
  }
  else if(m_stEnvMode.compare(vision_msgs::cop_camera_mode::DARK_ENVIRONMENT) == 0)
  {
    printf("Setting Exposure time to %d microseconds\n", 30000);
    SetExposure(30000);
  }
  else if(m_stEnvMode.compare(vision_msgs::cop_camera_mode::AVERAGE_ENVIRONMENT) == 0)
  {
    printf("Setting Exposure time to %d microseconds\n", 15000);
    SetExposure(15000);
  }
  else if(m_stEnvMode.compare(vision_msgs::cop_camera_mode::BRIGHT_ENVIRONMENT) == 0)
  {
    printf("Setting Exposure time to %d microseconds\n", 10000);
    SetExposure(10000);
  }
  else
  {
    SetExposure(atoi(m_stEnvMode.c_str()));
  }
}

void CameraDriverRelay::SetData(XMLTag* tag)
{
 try
 {
    CameraDriver::SetData(tag);
    m_stTopic = tag->GetProperty(XML_ATTRIBUTE_TOPICNAME);
    m_rate = tag->GetPropertyInt(XML_ATTRIBUTE_RATE);
    ros::NodeHandle nh;
    m_pub = nh.advertise<sensor_msgs::Image>(m_stTopic, 5);



    int n = m_stTopic.find("/camera");
    if(n == -1)
    {
      ROS_ERROR("Error in topic name of cop camera");
    }
    else
    {
     std::string temp1 = m_stTopic, temp2 = m_stTopic;
     std::string stTemp = temp1.replace(n, std::string("/camera").length(), "/camera_info");
     m_pubCamInfo = nh.advertise<sensor_msgs::CameraInfo>(stTemp, 5);


     std::string stModeTopic= temp2.replace(n, std::string("/camera").length(), "/camera_mode");
     ros::NodeHandle nh;
     printf("Subscribe to %s\n", stModeTopic.c_str());
     m_modeSubscriber = nh.subscribe<vision_msgs::cop_camera_mode>(
                                stModeTopic, 2,
                                boost::bind(&CameraDriverRelay::ModeCallback, this, _1));

      Halcon::HTuple camMatrix = m_calibration.CamMatrix();
      m_cameraInfoMessage.width = m_calibration.m_width;
      m_cameraInfoMessage.height = m_calibration.m_height;

      m_cameraInfoMessage.distortion_model = "plumb_bob";
      for(int i = 0; i < 5; i++)
      {
         m_cameraInfoMessage.D.push_back(0.0f);
      }

      for(int i = 0; i < 9; i++)
      {
        m_cameraInfoMessage.K[i] = camMatrix[i].D();
      }
      m_cameraInfoMessage.P[0] = camMatrix[0].D();
      m_cameraInfoMessage.P[1] = camMatrix[1].D();
      m_cameraInfoMessage.P[2] = camMatrix[2].D();
      m_cameraInfoMessage.P[3] = 0.0;

      m_cameraInfoMessage.P[4] = camMatrix[3].D();
      m_cameraInfoMessage.P[5] = camMatrix[4].D();
      m_cameraInfoMessage.P[6] = camMatrix[5].D();
      m_cameraInfoMessage.P[7] = 0.0;

      m_cameraInfoMessage.P[8] = camMatrix[6].D();
      m_cameraInfoMessage.P[9] = camMatrix[7].D();
      m_cameraInfoMessage.P[10] = camMatrix[8].D();
      m_cameraInfoMessage.P[11] = 1.0;
      m_bCameraInfo = true;
    }
  }
  catch(const char *test)
  {
    ROS_ERROR("Error with cam param in cop camera");
    m_bCameraInfo = false;
  }
  catch(Halcon::HException ex)
  {
    ROS_ERROR("Error with cam param in cop camera");
    m_bCameraInfo = false;
  }
}

