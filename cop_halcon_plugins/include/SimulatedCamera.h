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
                        SimulatedCamera.h - Copyright klank


**************************************************************************/


#ifndef SIMULATEDCAMERA_H
#define SIMULATEDCAMERA_H

#include "Camera.h"
#define XML_NODE_SIMULATEDCAMERA "SimulatedCamera"
#define XML_NODE_IMAGE_FILE "ImageFile"
#define XML_ATTRIBUTE_SOURCEISACTIVE "SourceIsActive"
#define XML_ATTRIBUTE_SOURCETYPE "SourceType"
namespace cop
{
  /**
    * class SimulatedCamera
    * @brief a class that allows to load images and videos in order to simulate a Camera
    */
  class SimulatedCamera : public Camera
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Constructor from a file
     */
      SimulatedCamera ();
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const{return XML_NODE_SIMULATEDCAMERA;}
      /**
      * Empty Destructor
      */
      virtual ~SimulatedCamera ( );

      virtual Reading*    GetReading(const long &Frame);
      virtual bool CanSee  (RelPose &pose) const;

      virtual long GetCurFrame(){return m_FrameCount;}
      virtual void SetCurFrame(long framecount){m_FrameCount = framecount;}



      virtual bool    Start();
      virtual bool    Stop();


      virtual XMLTag* Save();
      virtual void SetData(XMLTag* tag);


      void SetFileName(std::string fileName);
    virtual void NextFrame(){m_FrameCount++;if(!m_isVideo)m_FrameCount=m_FrameCount%(long)m_sourceFileName.size();}

  protected:
        Image* ReadFromFile(std::string filename);

  private:

      bool            m_sourceIsActiv;
      std::vector<std::string> m_sourceFileName;
      int                m_sourceType;


      bool            m_isVideo;
  #ifdef  HALCONIMG
      Halcon::HTuple*    m_acqHandle;
  #endif
  };
}
#endif // SIMULATEDCAMERA_H
