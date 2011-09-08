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
TransparentObject.h - Copyright klank
**************************************************************************/




#ifndef TRANSPARENTOBJECT_H
#define TRANSPARENTOBJECT_H
#include "Descriptor.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RegionOI.h"
#include "RelPoseHTuple.h"

#define XML_NODE_TRANSPARENTOBJECT "TransparentObject"

namespace cop
{
  class TransparentObject : public Descriptor
  {
  public:
    TransparentObject(){};
    TransparentObject(Class* classref):Descriptor(classref){};

    void SaveTo(XMLTag* tag);

    virtual void Show(RelPose* pose, Sensor* camin);

    virtual std::string GetNodeName() const{return XML_NODE_TRANSPARENTOBJECT;}

    virtual ElemType_t GetType() const{return DESCRIPTOR_TRANSPARENTOBJECT;}

  private:
    void SetData(XMLTag* tag);
  };
}
#endif /*TRANSPARENTOBJECT_H*/
