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
ColorClass.h - Copyright klank
**************************************************************************/




#ifndef COLORCLASS_H
#define COLORCLASS_H
#include "Descriptor.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RegionOI.h"
#include "RelPoseHTuple.h"

#define XML_NODE_COLORCLASS "ColorClass"
#define XML_ATTRIBUTE_COLORNAME "ColorName"
#define XML_ATTRIBUTE_CLUSTERHISTO "ClusterHisto"
namespace cop
{
  class ColorClass : public Descriptor
  {
  public:
    ColorClass(Class* cl, std::string colorName, std::map<std::string, double> histo) : Descriptor(cl), m_stColorName(colorName), m_histo(histo)
    {};


    ColorClass(){};

    double IsClass(const std::map<std::string, double> &histo) const
    {
      double result = 0.0;
      int count = 0;
      std::map<std::string, double>::const_iterator iter_in = histo.begin();
      for(; iter_in != histo.end(); iter_in++)
      {
        std::map<std::string, double>::const_iterator iter_self = m_histo.find((*iter_in).first);
        if(iter_self != m_histo.end())
        {
          double val1 = (*iter_self).second;
          double val2 = (*iter_in).second;
          result += (val1 - val2)*(val1 - val2);
          count++;
        }

      }
      if(count != 0)
        result = 1.0 - sqrt(result) / count;
      else
        result = 0.0;

      return result;

    }
    void SaveTo(XMLTag* tag)
    {
      Descriptor::SaveTo(tag);
      tag->AddProperty(XML_ATTRIBUTE_COLORNAME, m_stColorName);
      tag->AddChild(XMLTag::Tag(m_histo, XML_ATTRIBUTE_CLUSTERHISTO));
    }

     virtual void Show(RelPose* pose, Sensor* camin)
     {
      if(pose != NULL && camin != NULL && camin->IsCamera())
      {
        Camera* cam = (Camera*)camin;
        Halcon::HWindow* hwin = cam->GetWindow();
        hwin->SetColor("green");
        Halcon::HTuple pose_ht;
        RelPoseHTuple::GetPose(pose, &pose_ht, cam->m_relPose->m_uniqueID);
        RelPoseHTuple::Print(pose);
        Halcon::HTuple camparam = cam->m_calibration.CamParam();
        Halcon::HTuple r,c;
        Halcon::project_3d_point(pose_ht[0],pose_ht[1],pose_ht[2], camparam, &r,&c);
        Halcon::disp_cross(hwin->WindowHandle(), r, c, 15, 0);
      }
    }

    virtual std::string GetNodeName() const{return XML_NODE_COLORCLASS;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_COLORCLASS;}
    std::string GetMainColorName(){return m_stColorName;}
  private:
    void SetData(XMLTag* tag)
    {
      Descriptor::SetData(tag);
      m_stColorName = tag->GetProperty(XML_ATTRIBUTE_COLORNAME);
      XMLTag* histo = tag->GetChild(XML_ATTRIBUTE_CLUSTERHISTO);
      if(histo != NULL)
      {
         m_histo = XMLTag::Load(histo, &m_histo);
      }
      else /** backwards compatibility*/
      {
        m_histo["red"] = 0.0;
        m_histo["green"] = 0.0;
        m_histo["blue"] = 0.0;
        m_histo["white"] = 0.0;
        m_histo["black"] = 0.0;
        if(m_stColorName.compare("red") == 0)
        {
            m_histo["red"] = 1.0;
        }
        else if (m_stColorName.compare("green") == 0)
        {
          m_histo["green"] = 1.0;
        }
        else if (m_stColorName.compare("blue") == 0)
        {
          m_histo["blue"] = 1.0;
        }
        else if (m_stColorName.compare("white") == 0)
        {
          m_histo["white"] = 1.0;
        }
        else if (m_stColorName.compare("black") == 0)
        {
          m_histo["black"] = 1.0;
        }
      }

    };
    std::string m_stColorName;
    std::map<std::string, double> m_histo;
  };
}
#endif /*COLORCLASS_H*/
