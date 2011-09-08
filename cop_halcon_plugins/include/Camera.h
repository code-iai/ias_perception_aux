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


/**
*	klank 31.10.2007
*/

#ifndef CAMERA_H
#define CAMERA_H

#include "Image.h"
#include "Sensor.h"

#include <vector>
#include "RelPoseFactory.h"



namespace Halcon
{
    class HWindow;
    class HTuple;
    class Hobject;
}

class RobotMeshModel;

#define XML_NODE_CALIBRATION "CalibMatrix"
#define XML_NODE_WIDTH "Width"
#define XML_NODE_HEIGHT "Height"
#define XML_NODE_CAMERA "Camera"

namespace cop
{
  /**
  *   Class Calibration
  *   @brief stores the calibration and provides converions to different formats, stores a radial distortion map
  */
  class Calibration
  {
  public:
      Calibration():m_radialDistortionHandling(false){};
      Calibration(XMLTag* tag);
      ~Calibration();
      std::vector<double> m_calibrationmatrix;
      int m_width;
      int m_height;
      bool m_radialDistortionHandling;
      void SetCamParam(Halcon::HTuple& t);
      /**
      * CamParam
      *
      * @throw char* with an error message in case of failure
      */
      Halcon::HTuple CamParam() const;
      /**
      * CamParam
      * @param width specifies the wished images width for which the parameters should be converted, not yet tested
      * @param height specifies the wished images height for which the parameters should be converted, not yet tested
      * @throw char* with an error message in case of failure
      */
      Halcon::HTuple CamParam(int width, int height);

      /**
      * CamParam
      * @param scale_image isotropic image scale apllied to the camparams by this factor
      * @throw char* with an error message in case of failure
      */
      Halcon::HTuple CamParam(double scale_image);
      /**
      * CamMatrix
      *
      * @throw char* with an error message in case of failure
      **/
      Halcon::HTuple CamMatrix();

      Halcon::HTuple* m_camdist;
      Halcon::Hobject* m_radialDistMap;
      void SaveTo(XMLTag* tag);
      //TODO usw
  };

  /**
  *   Class Camera
  *   @brief Provides an interface for camera usage
  */
  class Camera : public Sensor
  {
  public:
      /***
      *   @brief Constructor with pose, initializes parameters, sets the camera pose
      */
      Camera(RelPose* pose) :
         Sensor(pose)
         ,m_win(NULL)
      {}
      /**
      *   The destructor is virtual
      */
      virtual ~Camera();
  protected:
      /**
      *   @brief Load camera parameter from xml
      *   Only derivatives of Camera can use this constructor to initialize values
      */
      Camera();

  public:
      /**
      *   CameraFactory
      *   @brief static function to create cameras
      *   @param tag contains the specification of the camera
      */
      static Camera*	CamFactory(XMLTag* tag);
      void			SaveTo(XMLTag* tag);

      /**
      *  SetData
      *  @brief initializes the class from an xml file, virtual, shoul dbe called by all derivatives
      */
      virtual void SetData(XMLTag* tag);
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const{return XML_NODE_CAMERA;};

      Calibration		m_calibration;
      int				LastAskedFrame;

      Image* GetImage(const long &Frame){return (Image*)GetReading(Frame);}

      virtual bool CanSee(RelPose& pose) const;

      virtual void ProjectPoint3DToSensor(const double &x, const double &y,
             const double &z, double &row, double &column);


      /**
      *  GetUnformatedCalibrationValues
      *  @return a pair of a fomrat string describing the content and a list of doubles
      */
      virtual std::pair<std::string, std::vector<double> > GetUnformatedCalibrationValues() const;

      /**
      *   @brief select first camera
      *   @param sensors List of sensors that are available
      *   @return the first camera in the list of sensors
      */
      static Camera* GetFirstCamera(const std::vector<Sensor*> &sensors);

      virtual Reading* ApplySelfFilter(Reading* read);


      virtual long GetCurFrame(){return -1;}
      virtual void NextFrame(){}

      virtual bool	Start()				= 0;
      virtual bool	Stop()				= 0;
      virtual void ReadCamParam(std::string filename = "");

      virtual Halcon::HWindow* GetWindow();
      virtual void DeleteWindow();
      Halcon::HWindow*   m_win;
      RelPose*		       m_relPose;
      RobotMeshModel  *m_self_filter;
      char*  m_image_data;
      std::string m_stCalibName;
      bool m_bselffilterActive;                               

      void CreateSelfFilter();
      void GetSelfFilterMask(Halcon::Hobject* region);

      virtual void Show(const long frame = -1);
      virtual void WriteToFile(std::string fileName, const long& Frame = -1);
      virtual XMLTag* Save() = 0;
      bool IsCamera() const {return true;}

  };
}
#endif /*CAMERA_H*/
