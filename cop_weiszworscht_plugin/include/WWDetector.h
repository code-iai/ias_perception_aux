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


#ifndef WWDETECTOR_H
#define WWDETECTOR_H


#include "LocateAlgorithm.h"
#include "Camera.h"

#include "SurfaceDetection.h"

#include "XMLTag.h"


#define XML_NODE_WWDETECTOR "WWDetector"


#define XML_NODE_WEISSWURSTCONTAINER "WeisswurstContainer"



namespace cop
{
  /********************************************************************
  *   class PanCakeDetector                                          */
  /********************************************************************
  *   @brief a planar locating algorithm
  *
  *********************************************************************/
  class WWDetector :
    public LocateAlgorithm
  {
  public:
    /**
      Constructor
    */
    WWDetector(void){};
    /**
      Destructor
    */
    ~WWDetector(void){};

    /**
    *
    */
    void SetData(XMLTag* tag);
    /**
      Action function, prepares images
    */
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
    /**
      Test if the already there are models learned
    */
    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual std::string GetName(){return XML_NODE_WWDETECTOR;}

    /**
      Save the parameters or any
    */
    XMLTag* Save(){XMLTag* tag = new XMLTag(GetName()); tag->AddChild(m_surfaceDetect->Save()); return tag;}

    SurfaceDetection*  m_surfaceDetect;

  };


  class WeisswurstContainer : public SurfaceModel
  {
  public:
      WeisswurstContainer() : m_minArea(3500), m_maxArea(30000), m_minCompactness(0.0), m_maxCompactness(3.0)
      {}
      virtual ElemType_t GetType()const{return DESCRIPTOR_SURFACE;}
      virtual std::string GetNodeName() const {return "WeisswurstContainer";}

      virtual void SetData(XMLTag* tag);
      virtual void SaveTo(XMLTag* tag);

      virtual void Show(RelPose* pose, Sensor* cam)
      {
        cam->Publish3DData(m_lastX, m_lastY, m_lastZ);
      }

      bool GetShape(GeometricShape &objectShape) const;
      std::vector<double> m_lastX;
      std::vector<double> m_lastY;
      std::vector<double> m_lastZ;

      int m_minArea;
      int m_maxArea;
      double m_minCompactness;
      double m_maxCompactness;

  };

}


#endif /*WWDETECTOR_H*/

