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


#ifndef COLORMODEL_H
#define COLORMODEL_H

#include "Descriptor.h"
#include "XMLTag.h"

#define XML_NODE_COLORMODEL "ColorModel"
#define XML_NODE_PATCHSIZE "PatchSize"
#define XML_NODE_COLORSPEC "ColorSpec"

namespace cop
{
  /**
  *   class ColorModel
  *   @brief Signature that contains an color model for a simple ColorBased detection
  */
  class ColorModel :
    public Descriptor
  {
  public:
    ColorModel(){}
    ColorModel(Class* classref) : Descriptor(classref){}
    ~ColorModel(void){}

    virtual ElemType_t GetType()const{return DESCRIPTOR_COLOR;}
    virtual std::string GetNodeName() const {return XML_NODE_COLORMODEL;}

    virtual double Compare(std::vector<double> vec)
    {
      size_t length = m_colorSpec.size() > vec.size() ? vec.size() : m_colorSpec.size();
      double d = 0.0;
      for(unsigned int i = 0; i < length; i++)
      {
        d+= (vec[i] - m_colorSpec[i]) * (vec[i] - m_colorSpec[i]);
      }
      return d;
    }



    void SetSize(int size){m_patchSize = size;}
    int GetSize(){return m_patchSize;}
    inline void AddColorSpec(double histo){m_colorSpec.push_back(histo);}
    void SaveTo(XMLTag* tag)
    {
      tag->AddChild(XMLTag::Tag(m_patchSize, XML_NODE_PATCHSIZE));
      tag->AddChild(XMLTag::Tag(m_colorSpec, XML_NODE_COLORSPEC));
    }

    std::vector<double> m_colorSpec;
    int m_patchSize;
  protected:
    virtual void SetData(XMLTag* tag)
    {
      Descriptor::SetData(tag);
      if(tag != NULL)
      {
        XMLTag* patchSize = tag->GetChild(XML_NODE_PATCHSIZE);
        if(patchSize != NULL)
          m_patchSize = tag->Load(patchSize,&m_patchSize);
        XMLTag* colorspec = tag->GetChild(XML_NODE_COLORSPEC);
        if(colorspec != NULL)
          m_colorSpec = tag->Load(colorspec,&m_colorSpec) ;
      }
    }
  };
}
#endif /*COLORMODEL_H*/
