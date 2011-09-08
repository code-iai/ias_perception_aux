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


#ifndef POINTDESCRMODEL_H
#define POINTDESCRMODEL_H

#include "Descriptor.h"
#include "Signature.h"

#define XML_NODE_POINTDESCRMODEL "PointDescrModel"

class DXFReader;

namespace cop
{
  class Calibration;
  class Camera;
  /**
  *   class PointDescrModel
  *   @brief the model that is needed for DescritporBased ( LocateAlgorthm )
  */
  class PointDescrModel :
    public Descriptor
  {
  public:
    PointDescrModel(Class* classref) : Descriptor(classref){}
    PointDescrModel(XMLTag* tag);
      /**
      * Constructor PointDescrModel
      *   @param dxf An initialized dxf reader
      *   @param sig Signature that contains shape information
      *   @throws char* with an error message in case of failure
      */
    PointDescrModel(DXFReader* dxf, Signature* sig);

    ~PointDescrModel(void);

    virtual ElemType_t GetType() const{return DESCRIPTOR_FEATURE;}
    virtual std::string GetNodeName() const{return XML_NODE_POINTDESCRMODEL;}

    /**
    *	Returns the current handle of the descriptor
    */
    int GetDescriptorHandle(){return m_DescriptoHandel;}
    Calibration* GetCurCalibration();

    void SaveTo(XMLTag* tag);
    RelPose* GetRelPose() const {return m_objectPose;}
  private:
    int m_DescriptoHandel;
    XMLTag* m_Config;
    RelPose* m_objectPose;
  };
}
#endif /*POINTDESCRMODEL_H*/
