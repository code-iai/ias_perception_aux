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


/*********************************************************************
*  Camera.cpp Copyright klank
*********************************************************************/


#include "CameraDriver.h"
#include "SimulatedCamera.h"

#include "ROSCamera.h"


/** Self Filter */
/*#include "camera_self_filter/robotMeshModel.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
*/
#include "XMLTag.h"
#include "cpp/HalconCpp.h"



#define XML_ATTRIBUTE_CALIBFILE       "CalibFileName"


using namespace Halcon;

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#define BOOST(A) A

using namespace cop;

extern volatile bool g_stopall;

// Constructors/Destructors
//

Camera* Camera::GetFirstCamera(const std::vector<Sensor*> &sensors)
{
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->IsCamera())
    {
      return (Camera*)(*it);
    }
  }
  return NULL;
}



Calibration::~Calibration()
{
  if(m_radialDistortionHandling)
  {
    delete m_radialDistMap;
    delete m_camdist;
  }
}

HTuple Calibration::CamParam() const
{
  if(m_radialDistortionHandling)
    return *m_camdist;
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
    {
      printf("Error in  Calibration::CamParam()\n");
      throw "No Camera Calibration available!\n";
    }
    for(int i = 0; i < 6; i++)
      par[i] = m_calibrationmatrix[i];
    par[6] = m_width;
    par[7] = m_height;
    return par;
  }
}

HTuple Calibration::CamParam(int width, int height)
{
  if(m_radialDistortionHandling)
  {
    HTuple tup = *m_camdist;
    tup[4] = tup[4].D() * width / m_width;
    tup[5] = tup[5].D() * height / m_height;
    tup[6] = width;
    tup[7] = height;
    //TODO support width height
    return tup;
  }
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
      throw "No Camera Matrix available for Projection\n";
    for(int i = 0; i < 4; i++)
      par[i] = m_calibrationmatrix[i];
    par[4] = m_calibrationmatrix[4] * width / m_width;
    par[5] = m_calibrationmatrix[5] * height / m_height;
    par[6] = width;
    par[7] = height;
    return par;
  }
}

HTuple Calibration::CamParam(double scale_image)
{
  if(m_radialDistortionHandling)
  {
    HTuple tup = *m_camdist;
    tup[2] = tup[2].D() / scale_image;
    tup[3] = tup[3].D() / scale_image;
    tup[4] = tup[4].D() * scale_image;
    tup[5] = tup[5].D() * scale_image;
    tup[6] = tup[6].D() * scale_image;
    tup[7] = tup[7].D() * scale_image;
    //TODO support width height
    return tup;
  }
  else
  {
    HTuple par;
    tuple_gen_const(8, 0, &par);
    if(m_calibrationmatrix.size() < 6)
      throw "No Camera Matrix available for Projection\n";
    par[0] = m_calibrationmatrix[0];
    par[1] = m_calibrationmatrix[1];
    par[2] = m_calibrationmatrix[2] / scale_image;
    par[3] = m_calibrationmatrix[3] / scale_image;
    par[4] = m_calibrationmatrix[4] * scale_image;
    par[5] = m_calibrationmatrix[5] * scale_image;
    par[6] = m_width  * scale_image;
    par[7] = m_height * scale_image;
    return par;
  }
}


HTuple Calibration::CamMatrix()
{
  HTuple param = CamParam();
  HTuple matrix, height, width;
  Halcon::cam_par_to_cam_mat(param, &matrix, &height, &width);
  return matrix;
}

Calibration::Calibration(XMLTag* tag) :
  m_radialDistortionHandling(false)
{
  if(tag != NULL)
  {
    XMLTag* tagName = tag->GetChild(XML_NODE_CALIBRATION);
    if(tagName != NULL)
      m_calibrationmatrix = XMLTag::Load(tagName, &m_calibrationmatrix);
    tagName = tag->GetChild(XML_NODE_WIDTH);
    if(tagName != NULL)
      m_width = XMLTag::Load(tagName, &m_width);
    tagName = tag->GetChild(XML_NODE_HEIGHT);
    if(tagName != NULL)
      m_height = XMLTag::Load(tagName, &m_height);
    if(m_calibrationmatrix.size() != 0)
    {
    if(m_calibrationmatrix[1] != 0.0)
    {
      printf("Radial Distortion Handling necessary!\n");
      Halcon::HTuple param = this->CamParam();
      m_radialDistortionHandling = true;
      m_radialDistMap = new Hobject();
      m_camdist = new HTuple();
      Halcon::change_radial_distortion_cam_par("fixed", param, 0.0, m_camdist);
      Halcon::gen_radial_distortion_map(m_radialDistMap, param, *m_camdist, "bilinear");
                        m_radialDistortionHandling = true;
    }
    }
  }
}

void Calibration::SetCamParam(HTuple& param)
{
  m_calibrationmatrix.clear();
  if(param.Num() > 1 && param[1].D() != 0.0)
  {
    m_camdist = new HTuple();
    Halcon::change_radial_distortion_cam_par("fixed", param, 0.0, m_camdist);
    m_radialDistMap = new Hobject();
    Halcon::gen_radial_distortion_map(m_radialDistMap, param, *m_camdist, "bilinear");
                m_radialDistortionHandling = true;

  }
  for(int i = 0; i < param.Num() - 2; i++)
  {
    m_calibrationmatrix.push_back(param[i].D());
  }
  m_width = param[param.Num() - 2].I();
  m_height = param[param.Num() - 1].I();
}

void Calibration::SaveTo(XMLTag* tag)
{
  tag->AddChild(XMLTag::Tag(m_calibrationmatrix, XML_NODE_CALIBRATION));
  tag->AddChild(XMLTag::Tag(m_width, XML_NODE_WIDTH));
  tag->AddChild(XMLTag::Tag(m_height, XML_NODE_HEIGHT));
}


typedef boost::mutex::scoped_lock scoped_lock;
boost::mutex g_display_data_mutex;
std::pair<char*, RobotMeshModel*> g_p_displayData;
Camera* g_curr_sensor;
bool g_Glinited = false;


Camera::~Camera()
{
  delete m_win;
  delete m_image_data;
  {
    scoped_lock(g_display_data_mutex);
    if(g_Glinited)
    {
      g_Glinited = false;
      //delete m_self_filter;
    }
  }
}


void Camera::ReadCamParam(std::string filename)
{
  HTuple t;
  try
  {
    Halcon::read_cam_par(filename.c_str(), &t);
    m_calibration.SetCamParam(t);
  }
  catch(Halcon::HException ex)
  {
    printf("Error reading Camera Param File: %s\n", ex.message);
    throw "Camera can not be used uncalibrated, (errror during reading calibration)";
  }
}

void Camera::SaveTo(XMLTag* tag)
{
  m_calibration.SaveTo(tag);
  Sensor::SaveTo(tag);
}

Camera::Camera() :
  m_image_data(NULL),
  m_bselffilterActive(false)
{
}

/*
void initializeGL ()
{
    glClearColor(0, 0, 0, 0);
//  glEnable(GL_LIGHTING);
//  glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                     GL_LINEAR_MIPMAP_LINEAR);
}


#define HCkGlP(A) A
void SetGLCamera(const HTuple &cam_gl)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  HCkGlP(glMatrixMode(GL_PROJECTION));
  HCkGlP(glLoadIdentity());
  HCkGlP(glFrustum(-(cam_gl[4].D() * cam_gl[2].D()),
            ((cam_gl[6].D()-cam_gl[4].D()) * cam_gl[2].D()) ,
           -((cam_gl[7].D()-cam_gl[5].D()) * cam_gl[3].D()) ,
            (cam_gl[5].D() * cam_gl[3].D()),
            cam_gl[0].D(), 6.0));
  HCkGlP(gluLookAt(0, 0,0,0, 0, 1,  0,-1,0));
  HCkGlP(glMatrixMode(GL_MODELVIEW));
  HCkGlP(glViewport(0,0,cam_gl[6].D(),cam_gl[7].D()));
}

void display() {

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   if(!g_stopall)
   {
     //g_p_displayData.second->setCamera();
     //g_p_displayData.second->setCameraPose();
     SetGLCamera(g_curr_sensor->m_calibration.CamParam());
     g_p_displayData.second->paintRobot();
     glReadPixels(0, 0, g_p_displayData.second->cam_info_->width, g_p_displayData.second->cam_info_->height , GL_RGBA, GL_UNSIGNED_BYTE, g_p_displayData.first);


   }
   glutSwapBuffers();
}
*/

Reading* Camera::ApplySelfFilter(Reading* read)
{
  /*
  if(m_bselffilterActive)
  {
    Halcon::Hobject region;
    GetSelfFilterMask(&region);
    printf("Apply Self filter in %s\n", GetSensorID().c_str());
    write_image(*((Image*)read)->m_image, "tiff", 0, "image_before.tiff");
    reduce_domain( *((Image*)read)->m_image, region, ((Image*)read)->m_image);
    write_region(region, "image_after_domain.reg");
    write_image(*((Image*)read)->m_image, "tiff", 0, "image_after.tiff");
  }*/
  return read;
}

void Camera::CreateSelfFilter()
{
 /*   char* value =  getenv ("DISPLAY");
    printf("Read Env Display: %s\n", value);
    if(!(value && strlen(value) > 0))
       return;

    printf("Create Filter\n");

    scoped_lock(g_display_data_mutex);

    if(!g_Glinited)
    {
      try
      {
        int argc = 0;
        char  argv[10];
        char * tmp = argv;
        glutInit(&argc, &tmp);

        m_self_filter = new RobotMeshModel(false);
        g_p_displayData.second = this->m_self_filter;

        boost::shared_ptr<sensor_msgs::CameraInfo> info (new sensor_msgs::CameraInfo);

        Halcon::HTuple camMatrix = m_calibration.CamMatrix();
        (*info).width = m_calibration.m_width;
        (*info).height = m_calibration.m_height;
        for(int i = 0; i < 9; i++)
        {
          (*info).K[i] = camMatrix[i].D();
        }
        (*info).P[0] = camMatrix[0].D();
        (*info).P[1] = camMatrix[1].D();
        (*info).P[2] = camMatrix[2].D();
        (*info).P[3] = 0.0;

        (*info).P[4] = camMatrix[3].D();
        (*info).P[5] = camMatrix[4].D();
        (*info).P[6] = camMatrix[5].D();
        (*info).P[7] = 0.0;

        (*info).P[8] = camMatrix[6].D();
        (*info).P[9] = camMatrix[7].D();
        (*info).P[10] = camMatrix[8].D();
        (*info).P[11] = 1.0;

        g_p_displayData.second->cam_info_ =  info;
        if(m_relPose != NULL)
        {
          g_p_displayData.second->cameraframe_ = m_relPose->m_mapstring;
          g_p_displayData.second->modelframe_ = m_relPose->m_mapstring;


        }
        else
        {
          g_p_displayData.second->cameraframe_ =  "/RightEyeCalc";
        }

        g_Glinited = true;
        glutInitWindowSize (m_self_filter->cam_info_->width, m_self_filter->cam_info_->height);
        glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
        glutCreateWindow ("dummy");
        glutHideWindow();
        initializeGL();
        glutDisplayFunc (display);
        m_bselffilterActive = true;
      }
      catch(...)
      {
         ROS_ERROR("Error Initializing self filter\n");
         m_bselffilterActive = false;
         g_Glinited = false;
      }
    }
    else
    {
        boost::shared_ptr<sensor_msgs::CameraInfo> info (new sensor_msgs::CameraInfo);

        Halcon::HTuple camMatrix = m_calibration.CamMatrix();
        (*info).width = m_calibration.m_width;
        (*info).height = m_calibration.m_height;
        for(int i = 0; i < 9; i++)
        {
          (*info).K[i] = camMatrix[i].D();
        }
        (*info).P[0] = camMatrix[0].D();
        (*info).P[1] = camMatrix[1].D();
        (*info).P[2] = camMatrix[2].D();
        (*info).P[3] = 0.0;

        (*info).P[4] = camMatrix[3].D();
        (*info).P[5] = camMatrix[4].D();
        (*info).P[6] = camMatrix[5].D();
        (*info).P[7] = 0.0;

        (*info).P[8] = camMatrix[6].D();
        (*info).P[9] = camMatrix[7].D();
        (*info).P[10] = camMatrix[8].D();
        (*info).P[11] = 1.0;

        g_p_displayData.second->cam_info_ =  info;
        if(m_relPose != NULL)
          g_p_displayData.second->cameraframe_ = m_relPose->m_mapstring;
        else
          g_p_displayData.second->cameraframe_ =  "/RightEyeCalc";


       this->m_self_filter = g_p_displayData.second;
       m_bselffilterActive = true;
    }
    m_image_data = new char[4*m_self_filter->cam_info_->width*m_self_filter->cam_info_->height];
*/
    // Register glut callbacks:
//    glutReshapeFunc (reshape);

}


void Camera::GetSelfFilterMask(Halcon::Hobject* region)
{
 /* scoped_lock(g_display_data_mutex);
  if(!g_Glinited)
  {
    if(g_stopall)
      return;

  return;
    CreateSelfFilter();
  }

  g_curr_sensor = this;
  g_p_displayData.first = this->m_image_data;

  g_p_displayData.second->cameraframe_ = m_relPose->m_mapstring;
  g_p_displayData.second->modelframe_ = m_relPose->m_mapstring;
  m_self_filter->updateRobotLinks(ros::Time::now());

  glutPostRedisplay();
  glutMainLoopEvent();


  try
  {
    Halcon::Hobject obj;
    Halcon::gen_image_interleaved(&obj, (Hlong)g_p_displayData.first ,"rgbx",
        (int)g_p_displayData.second->cam_info_->width, (int)g_p_displayData.second->cam_info_->height,
        1, "byte",  (int)g_p_displayData.second->cam_info_->width, (int)g_p_displayData.second->cam_info_->height, 0,0,-1,0);
    for(int i = 0; i < g_p_displayData.second->cam_info_->height*g_p_displayData.second->cam_info_->width*4; i++)
    {
      if(g_p_displayData.first[i] > 0)
        printf("adkl nwakldnwalkn\n\n\n\n\n\n");
    }
    write_image(obj, "tiff", 0, "test_image.tiff");

    Halcon::threshold(obj, region, Halcon::HTuple(0).Append(0).Append(0), Halcon::HTuple(127).Append(127).Append(127));
  }
  catch(Halcon::HException ex)
  {
    printf("Error getting selffilter: %s\n", ex.message);
  }*/
}


void Camera::SetData(XMLTag* tag)
{
  m_relPose = NULL;
  Sensor::SetData(tag);
  m_self_filter = NULL;
  m_calibration = Calibration(tag);
  m_stCalibName  = tag->GetProperty(XML_ATTRIBUTE_CALIBFILE);
  m_win = (NULL);
  if(m_relPose == NULL)
    m_relPose = RelPoseFactory::FRelPoseWorld();
  else
    printf("Camera Pose: %s %ld\n", m_relPose->m_mapstring.c_str(), m_relPose->m_uniqueID);
  if(m_stCalibName.length() > 0)
  {
    ReadCamParam(m_stCalibName);
  }
  /*CreateSelfFilter();*/
};


Halcon::HWindow* Camera::GetWindow()
{
  if(m_win == NULL)
  {
#ifdef _DEBUG
    printf("Creating a new window\n");
#endif
    m_win = new Halcon::HWindow(10,10,650,490, 0, "", "");
  }
  if(m_win == NULL)
    throw("failed to allocate a window");
  //m_win->SetShape("rectangle1");
  return m_win;
}
void Camera::DeleteWindow()
{
  delete m_win; m_win = NULL;
}

void Camera::WriteToFile(std::string fileName, const long& Frame)
{
  printf("Write to %s\n", fileName.c_str());
  Image* img = GetImage(Frame);
  if(img != NULL)
  {
    Halcon::Hobject* obj = img->GetHImage();
    if(img->GetType() == YUV_IMAGE)
    {
      Halcon::Hobject img1, img2, img3, imgr, imgg, imgb;
      Halcon::decompose3(*obj, &img1, &img2, &img3);
      Halcon::trans_to_rgb(img1, img2, img3, &imgr, &imgg, &imgb, "yuv");
      Halcon::compose3(imgr, imgg, imgb, &img1);
      Halcon::write_image(img1, "jpg", 0, fileName.c_str());
    }
    else
      Halcon::write_image(*obj, "jpg", 0, fileName.c_str());
    img->Free();
  }
}

std::pair<std::string, std::vector<double> > Camera::GetUnformatedCalibrationValues() const
{
  std::pair<std::string, std::vector<double> > ret;
  ret.first = "RECTHALCONCALIB";
  Halcon::HTuple tup = m_calibration.CamParam();
  ret.second.push_back( tup[0].D());
  ret.second.push_back( tup[2].D());
  ret.second.push_back( tup[3].D());
  ret.second.push_back( tup[4].D());
  ret.second.push_back( tup[5].D());
  ret.second.push_back( tup[6].D());
  ret.second.push_back( tup[7].D());
  return ret;
}


bool  Camera::CanSee (RelPose &pose) const
{
  if(m_relPose == NULL)
     return false;
  if(pose.m_uniqueID == m_relPose->m_uniqueID) /*lazy people just search in front of the camera, allow it*/
    return true;


  RelPose* pose_rel = RelPoseFactory::GetRelPose(pose.m_uniqueID, m_relPose->m_uniqueID);
  if(pose_rel != NULL)
  {
    Matrix m = pose_rel->GetMatrix(0);
    RelPoseFactory::FreeRelPose(&pose_rel);
    double x = m.element(0,3);
    double y = m.element(1,3);
    double z = m.element(2,3);
    if(z > 0.0)
    {

      try
      {
        Halcon::HTuple R, C;
        Halcon::project_3d_point(x,y,z, m_calibration.CamParam(), &R , &C);
        if(R >= 0 && R < m_calibration.m_height
          && C >= 0 && C < m_calibration.m_width)
          return true;
      }
      catch(Halcon::HException ex)
      {
        printf("Error: %s\n", ex.message);
      }
    }
  }
  else
  {
    printf("Queried position does not exist (or position of the camera), so better assume the camera can see this position\n");
    return true;
  }
  return false;
}

void Camera::ProjectPoint3DToSensor(const double &x, const double &y,
             const double &z, double &row, double &column)
{
  Halcon::HTuple R, C;
  double r, c;
  Halcon::project_3d_point(x,y,z, m_calibration.CamParam(), &R , &C);
  Sensor::ProjectPoint3DToSensor(x,y,z,r, c);
  Halcon::set_color(m_win->WindowHandle(), "green");
  Halcon::disp_cross(m_win->WindowHandle(), R, C, 8, 0);
  Halcon::set_color(m_win->WindowHandle(), "red");
  Halcon::disp_cross(m_win->WindowHandle(), r, c, 8, 0.7);
  printf("Ros: %f == %f Col %f == %f\n",  R[0].D(), r,  C[0].D(), c);
  row = R[0].D();
  column = C[0].D();
}


void Camera::Show(const long frame)
{
  try
  {
    printf("Try to show Camera %s\n", GetSensorID().c_str());
    if(m_win == NULL)
    {
      GetWindow();
      printf("Opened Window\n");
      if(m_win == NULL)
      {
        ROS_ERROR("Failed to open a window\n");
        return;
      }
    }
    Image* img = GetImage(frame);
    if(img != NULL)
    {
      printf("Got image\n");

      Halcon::Hobject* obj = img->GetHImage();
      Halcon::HTuple p,t,height, width, chan;
      Halcon::get_image_pointer1(*obj, &p,&t,&width, &height);

      //printf("Showing with: height %d width %d\n", height[0].I(), width[0].I());//
      m_win->SetWindowExtents(20,10,(width[0].I() ) + 10, (height[0].I()) + 20);
      m_win->SetPart(0,0,height, width);
      //TODO Check if this is not colored!
      if(img->GetType() == YUV_IMAGE)
      {
        Halcon::Hobject img1, img2, img3, imgr, imgg, imgb;
        Halcon::decompose3(*obj, &img1, &img2, &img3);
        Halcon::trans_to_rgb(img1, img2, img3, &imgr, &imgg, &imgb, "yuv");
        Halcon::compose3(imgr, imgg, imgb, &img1);
        Halcon::disp_obj(img1, m_win->WindowHandle());
      }
      else if(img->GetType() == GRAY_DISPARITY_IMAGE)
      {
        Halcon::Hobject img1, img2, img3;
        Halcon::decompose3(*obj, &img1, &img2, &img3);
        Halcon::disp_obj(img1, m_win->WindowHandle());
      }
      else
      {
        printf("Try to show normal image\n");
        Halcon::disp_obj(*obj, m_win->WindowHandle());
      }
      img->Free();
    }
    else{
      printf("Image from camera == NULL (%s)\n", GetName().c_str());
    }
  }
  catch(Halcon::HException ex)
  {
    printf("Showing not possible (Sensor: %s): %s \n", ex.message, GetSensorID().c_str());
  }
  catch(...)
  {
     printf("Showing not possible! Unknown Exception\n");
  }
}


//#endif
