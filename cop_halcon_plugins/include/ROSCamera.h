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


#ifndef ROSCOPCamera_H
#define ROSCOPCamera_H

#include "Camera.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


#define XML_NODE_ROSCOPCAMERA "ROSCOPCamera"
namespace cop
{
  /**
  *   Class ROSCOPCamera
  *   @brief Provides a connection to ros-Image topics
  */
  class ROSCOPCamera : public Camera
  {
  public:
      /***
      *   @brief Constructor, initializes camera from xml file
      */
      ROSCOPCamera();

      /**
      *   The destructor
      */
      virtual ~ROSCOPCamera();

  public:
     virtual std::string GetName() const {return XML_NODE_ROSCOPCAMERA;};

      /**
      * GetImage
      * @param Frame frame number, to specify an offset or a specific file
      * @throw char* with an error message in case of failure
      */
      virtual Reading*	GetReading(const long &Frame);

      virtual XMLTag* Save();

      virtual void SetData(XMLTag* tag);

      virtual bool CanSee(RelPose &pose) const {if(!m_grabbing)return false;
                                                else return Camera::CanSee(pose);}


  private:
      void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
      virtual bool	Start();
      virtual bool	Stop();
      std::string m_stImageTopic;
      bool m_grabbing;
      ros::Subscriber m_subsciber;

  };
}
#endif /*ROSCOPCamera_H*/
