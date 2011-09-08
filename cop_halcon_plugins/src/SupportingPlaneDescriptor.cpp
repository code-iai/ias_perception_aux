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


#include "SupportingPlaneDescriptor.h"
#include "XMLTag.h"

#define XML_TAG_EXT_X "ExtensionsX"
#define XML_TAG_EXT_Y "ExtensionsY"
using namespace cop;


SupportingPlaneDescriptor::SupportingPlaneDescriptor(Class* cl) :
    Descriptor(cl),
    m_extX(0.1),
    m_extY(0.1),
    m_marker(NULL)
{
}

SupportingPlaneDescriptor::SupportingPlaneDescriptor()
{
}

void SupportingPlaneDescriptor::SetData(XMLTag* node)
{
  Descriptor::SetData(node);
  if(node != NULL)
  {
    if(node->GetChild(XML_TAG_EXT_X) != NULL)
      m_extX = XMLTag::Load(node->GetChild(XML_TAG_EXT_X), &m_extX);
    if(node->GetChild(XML_TAG_EXT_Y) != NULL)
      m_extY = XMLTag::Load(node->GetChild(XML_TAG_EXT_Y), &m_extY);
    if(node->GetChild(3) != NULL)
      m_marker = XMLTag::Load(node->GetChild(3), &m_marker);
  }
}


SupportingPlaneDescriptor::~SupportingPlaneDescriptor(void)
{
}

void SupportingPlaneDescriptor::Show(RelPose* pose, Sensor* cam)
{
    return;
}


void SupportingPlaneDescriptor::SaveTo(XMLTag* tag)
{
    tag->AddChild(XMLTag::Tag(m_extX, XML_TAG_EXT_X));
    tag->AddChild(XMLTag::Tag(m_extY, XML_TAG_EXT_Y));
    if(m_marker != NULL)
      tag->AddChild(XMLTag::Tag(m_marker));
}
