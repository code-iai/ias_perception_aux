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
CircleDescriptor.h - Copyright klank
**************************************************************************/




#ifndef CIRCLEDESCRIPTOR_H
#define CIRCLEDESCRIPTOR_H
#include "Descriptor.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RegionOI.h"
#include "RelPoseHTuple.h"

#define XML_NODE_CIRCLEDESRIPTOR "CircleDescriptor"
#define XML_ATTRIBUTE_RADIUS "Radius"

namespace cop
{
  class CircleDescriptor : public Descriptor
  {
  public:
    CircleDescriptor(){};


    void SaveTo(XMLTag* tag)
    {
      Descriptor::SaveTo(tag);
      tag->AddProperty(XML_ATTRIBUTE_RADIUS, m_radius);
    }

    virtual std::string GetNodeName() const{return XML_NODE_CIRCLEDESRIPTOR;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_CIRCLE;}
    double GetRadius(){return m_radius;};

    virtual void Show(RelPose* pose, Sensor* cam)
    {
      printf("CircleDescriptor::Show\n");
      if(cam != NULL)
      {
        printf("Not empty cam\n");
        if(m_x.size() > 0 && m_x.size() == m_y.size() &&  m_x.size() == m_z.size())
        {
          printf("Not empty point cloud\n");
          std::vector<double> x,y,z;
          Matrix m = cam->GetRelPose()->GetMatrix(ID_WORLD);
          for(size_t len = 0; len < m_x.size(); len++)
          {
            ColumnVector d(4);
            d << m_x[len] << m_y[len] << m_z[len] << 1;
            ColumnVector f = m * d;
            x.push_back(f.element(0));
            y.push_back(f.element(1));
            z.push_back(f.element(2));
            
          }
          cam->Publish3DData(x,y,z);
        }
      }
    }
    std::vector<double> m_x;
    std::vector<double> m_y;
    std::vector<double> m_z;

    Elem* Duplicate(bool bStaticCopy)
    {
      CircleDescriptor* new_obj = (CircleDescriptor*)Descriptor::Duplicate(bStaticCopy);

      new_obj->m_x = m_x;
      new_obj->m_y = m_y;
      new_obj->m_z = m_z;
      new_obj->m_radius = m_radius;
      /** Assign Descriptor Memebers*/
      new_obj->m_class = m_class;
      new_obj->m_imgLastMatchReading = m_imgLastMatchReading;
      new_obj->m_poseLastMatchReading = m_poseLastMatchReading;
      new_obj->m_qualityMeasure = m_qualityMeasure;
      return new_obj;
    }



  private:
    void SetData(XMLTag* tag)
    {
      Descriptor::SetData(tag);
      m_radius = tag->GetPropertyDouble(XML_ATTRIBUTE_RADIUS);
    }
    double m_radius;

  };
}
#endif /*CIRCLEDESCRIPTOR_H*/

