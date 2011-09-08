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


#ifndef PANCAKEDETECTOR_H
#define PANCAKEDETECTOR_H


#include "LocateAlgorithm.h"
#include "Camera.h"

#include "DeformShapeModel.h"

#include "XMLTag.h"


#define XML_NODE_PANCAKEDETECTOR "PanCakeDetector"


#define XML_NODE_PANCAKEMAKERMODEL "PanCakeMaker"



namespace cop
{
  /********************************************************************
  *   class PanCakeDetector                                          */
  /********************************************************************
  *   @brief a planar locating algorithm
  *
  *********************************************************************/
  class PanCakeDetector :
    public LocateAlgorithm
  {
  public:
    /**
      Constructor
    */
    PanCakeDetector(void){};
    PanCakeDetector(XMLTag* tag){};
    /**
      Destructor
    */
    ~PanCakeDetector(void){};

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

    virtual std::string GetName(){return XML_NODE_PANCAKEDETECTOR;}

    /**
      Save the parameters or any
    */
    XMLTag* Save(){return new XMLTag(GetName());};

  };


  class PanCakeMaker : public DeformShapeModel
  {
  public:
      PanCakeMaker() : m_minArea(3500), m_maxArea(30000), m_minCompactness(0.0), m_maxCompactness(3.0)
      {}
      virtual ElemType_t GetType()const{return DESCRIPTOR_DEFORMSHAPE;}
      virtual std::string GetNodeName() const {return XML_NODE_PANCAKEMAKERMODEL ;}
      virtual void Show(RelPose* pose, Sensor* cam)
      {
        DeformShapeModel::Show(pose, cam);
        cam->Publish3DData(m_lastX, m_lastY, m_lastZ);
      }
     std::vector<double> m_lastX;
     std::vector<double> m_lastY;
     std::vector<double> m_lastZ;
     
     int m_minArea;
     int m_maxArea;
     double m_minCompactness;
     double m_maxCompactness;
     
  };

}


#endif /*PANCAKEDETECTOR_H*/

