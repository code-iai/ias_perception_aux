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

#include "RFAColorByShape.h"
#include "ColorClass.h"
/*#include "ShapeModel.h"*/
#include "CheckColorClass.h"
#include "ShapeModel.h"
#include "Camera.h"

using namespace cop;


RFAColorByShape::RFAColorByShape()
{
}

void RFAColorByShape::SetData(XMLTag* tag)
{
  printf("Loading Algorithm RFAColorByShape\n");
  XMLTag* child = tag->GetChild(XML_NODE_CHECKCOLORCLASS);
  if(child == NULL)
    throw "Error loading RFAColorByShape";
  m_checkColor = (CheckColorClass*)LocateAlgorithm::LocAlgFactory(child);

}
RFAColorByShape::RFAColorByShape(CheckColorClass* checkColor) :
  m_checkColor(checkColor)
{
}


RFAColorByShape::~RFAColorByShape(void)
{
}

Descriptor* RFAColorByShape::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  ColorClass* cm = NULL;
  for(size_t i = 0; i < sensors.size(); i++)
  {
    if(sensors[i]->IsCamera())
    {
      Camera* cam = ((Camera*)sensors[i]);
      Image* img = cam->GetImage(-1);
      Halcon::HTuple count;
      Halcon::Hobject* himg = img->GetHImage();
      Halcon::count_channels(*himg, &count);
      if(count < 3)
        continue;
      std::map<std::string, double> hist;
      RegionOI* region = new RegionOI(pose,cam->m_relPose->m_uniqueID, &(cam->m_calibration));
      std::string stColor;
	  /*(Hobject *img, Hobject *region, std::string &color, double& qualityMeasure)*/
      m_checkColor->Inner(himg , &region->GetRegion(), stColor, hist, qualityMeasure);
      if(qualityMeasure > 0.2)
        cm = new ColorClass(new Class(stColor, Elem::m_LastID), stColor, hist);
      else
        cm = new ColorClass(new Class("MultiColor", Elem::m_LastID), stColor, hist);
      cm->Evaluate(qualityMeasure, 100.0);
      break;
    }
  }
  return cm;

}

double RFAColorByShape::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sens)
{
  if(sig.GetElement(0,DESCRIPTOR_COLORCLASS) == NULL)
  {
    if(sig.GetElement(0, DESCRIPTOR_TRANSPARENTOBJECTCAND) == NULL)
      return 1.0;
    else
      return -0.0;
  }
  else
    return -0.0;
}

XMLTag* RFAColorByShape::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_COLORBYSHAPE);

	//TODO: parameter?
	return tag;
}




