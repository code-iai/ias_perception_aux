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


#ifndef SUPPORTINGPLANEDESCRIPTOR_H
#define SUPPORTINGPLANEDESCRIPTOR_H
#include "Descriptor.h"

#define XML_NODE_PLANE_DESCR "SupportingPlaneDescriptor"
namespace cop
{
  class SupportingPlaneDescriptor :
      public Descriptor
  {
  public:
      SupportingPlaneDescriptor(Class* cl);
      SupportingPlaneDescriptor();
      virtual ~SupportingPlaneDescriptor(void);

      virtual std::string GetNodeName() const {return XML_NODE_PLANE_DESCR;}

      virtual ElemType_t GetType() const {return DESCRIPTOR_PLANE;}

      virtual void Show(RelPose*, Sensor* );
      Elem*& GetMarker(){return m_marker;}
      double& GetExtX(){return m_extX;}
      double& GetExtY(){return m_extY;}

  protected:
      virtual void SaveTo(XMLTag* );
      virtual void SetData(XMLTag* );

  private:
      double m_extX;
      double m_extY;
      Elem* m_marker;
  };
}
#endif /*SUPPORTINGPLANEDESCRIPTOR_H*/
