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


#ifndef DEFORMSHAPEMODEL_H
#define DEFORMSHAPEMODEL_H
/*#ifdef DEFORMSHAPE_AVAILABLE*/

#include "Descriptor.h"
#include "Sensor.h"
#include "RelPose.h"


#include <map>
#include <stdio.h>
#define XML_NODE_DEFORMSHAPEMODEL "DeformShapeModel"

namespace Halcon
{
	class HTuple;
	class Hobject;
}

namespace cop
{
  class Signature;
  class Camera;
  class Image;
  /********************************************************************
  *   class DeformShapeModel                                          */
  /********************************************************************
  *   @brief The model that enables using DeformShapeBased
  *
  *********************************************************************/
  class DeformShapeModel :
    public Descriptor
  {

  public:
    DeformShapeModel(Class* classref);

    DeformShapeModel();

    ~DeformShapeModel(void);

    virtual ElemType_t GetType()const{return DESCRIPTOR_DEFORMSHAPE;}
    virtual std::string GetNodeName() const {return XML_NODE_DEFORMSHAPEMODEL;}


    long GetDeformShapeHandle(std::string sensor_id);

    /*First match, needed for tracking*/
    double DefineDeformShapeModel(Image* img, Halcon::Hobject* region, Camera* cam, RelPose* pose, Matrix planarAssumtionCompensation);
    /** Implement a local copy*/
    virtual Elem* Duplicate(bool bStaticCopy);

    /***********************************************************************
    * ShapeModel::Show                                                     */
    /************************************************************************
    * @brief if this Descriptor can be showed, show it.
    * @param pose A position where this descriptor should be displayed,
    * @param camera that took the picture where the descriptor was displayed
    *************************************************************************/
    virtual void Show(RelPose* pose, Sensor* cam);


    class SensorSpecificDeformModel
    {
      public:
        long m_handle;
        MinimalCalibration m_calib;

        bool IsCompatible(Sensor* sens)
        {
          MinimalCalibration *temp = new MinimalCalibration(sens->GetUnformatedCalibrationValues());
          if(temp->width != m_calib.width || temp->height !=m_calib.height)
          {
            delete temp;
            return false;
          }
          else
          {
            delete temp;
            return true;
          }
        }
    };

    SensorSpecificDeformModel& GetDeformShapeModel()
    {
      if(m_models.size() > 0)
        return (*m_models.begin()).second;
      else
        throw "No DeformShapeModel Available";
    }


  protected:
    bool LoadFromFile(std::string fileName, std::string stSensorID);

    virtual void SetData(XMLTag* tag);
    virtual void SaveTo(XMLTag* tag);


  private:
    std::map<std::string, SensorSpecificDeformModel> m_models;
    std::string m_filename;
    bool m_bWritten;
    Halcon::Hobject* m_regionTemp;
    bool m_autoLearned;
    
  public:
    Matrix m_planarAssumtionCompensation;
    Matrix m_covInherit;
  };
}
/*#endif *//*DEFORMSHAPE_AVAILABLE*/
#endif
