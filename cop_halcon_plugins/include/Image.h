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
                        Image.h - Copyright klank

**************************************************************************/


#ifndef IMAGE_H
#define IMAGE_H

#ifdef OPENCV_USED
//#include "cv.h"
//#include "highgui.h"
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif /*OPENCV_USED*/

#include <string>
#include "Reading.h"

/**
  * class Image
  */

#define GRAY_IMAGE 0
#define RGB_IMAGE 1
#define YUV_IMAGE 2
#define HSV_IMAGE 4
#define GRAY_DISPARITY_IMAGE 8
#define YUV_DISPARITY_IMAGE 16
#define NO_IMAGE 32

#define XML_NODE_IMAGEFILE "ImageFile"
#define XML_ATTRIBUTE_FILENAME "FileName"
#define XML_ATTRIBUTE_IMGTYPE "ImgType"

namespace Halcon
{
    class Hobject;
}

namespace cop
{
  class XMLTag;
  class RelPose;
  /******************************************************************
  *           class Image                                           */
  /** ****************************************************************
  *   @brief Abstraction of an images, can be derived for different
              images or acquisition devices
  *   Provides Halcon Image and IplImage
  *******************************************************************/
  class Image : public Reading
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    Image (int type);
    Image ( const Image& img);


      /**
      * Constructor Image
     */
      Image ();

      Image ( Halcon::Hobject* img,int type, RelPose* pose = NULL);
    /**
     * Empty Destructor
     *   @throws char* with an error message in case of failure
     */
    virtual ~Image ( );

    /**
     *   Save the image in a file and put a string in the xml
     */
    XMLTag* Save();
    /**
      *   @param tag contains a file name or similar information
      *   @throws char* with an error message in case of failure
    */
    virtual void SetData(XMLTag* tag);
    /**
     *   remove a file? TODO implement
     */
    void Delete(XMLTag* tag) ;

    /**
     *   Make a copy in meory of the image
     */
    virtual Reading* Clone();
    /**
     *   Get image information
     */
    int GetColorSpace() const {return m_type;}


    /**
     *   Get the Halcon image representation
     */
    Halcon::Hobject* GetHImage() const;
    /**
     *   Get the Halcon image representation of a zoom of the image
     */
    Halcon::Hobject* ZoomImage(int width, int height);
  #ifdef OPENCV_USED
    /**
     *   Get an opencv image
     */
    IplImage* GetIplImage();
  #endif /*OPENCV_USED*/

  protected:


  public:
      Halcon::Hobject* m_image;
  public:
      int m_type;

      static void RegisterImageConverter();


  };
}

#endif // IMAGE_H
