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


#include "Blob.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RelPoseHTuple.h"

#include <cpp/HalconCpp.h>

#define XML_ATTRIBUTE_REDTHRESMIN   "RedThresMin"
#define XML_ATTRIBUTE_REDTHRESMAX   "RedThresMax"
#define XML_ATTRIBUTE_GREENTHRESMIN "GreenThresMin"
#define XML_ATTRIBUTE_GREENTHRESMAX "GreenThresMax"
#define XML_ATTRIBUTE_BLUETHRESMIN  "BlueThresMin"
#define XML_ATTRIBUTE_BLUETHRESMAX  "BlueThresMax"
#define XML_ATTRIBUTE_MINAREA       "MinArea"
#define XML_ATTRIBUTE_MAXAREA       "MaxArea"
#define XML_ATTRIBUTE_SIZEXY        "SizeXY"
#define XML_ATTRIBUTE_SIZEZ         "SizeZ"
#define XML_ATTRIBUTE_MINROUNDNESS  "MinRoundness"

using namespace cop;


Blob::Blob(int thresRedMin, int thresRedMax,
        int thresGreenMin, int thresGreenMax,
        int thresBlueMin, int thresBlueMax,
        int areaMin, int areaMax,
        double minRoundness, double sizeXY, double sizeZ) :
  Descriptor(new Class("Blob", Elem::m_LastID)),
  RedThresMin(thresRedMin),
  RedThresMax(thresRedMax),
  GreenThresMin(thresGreenMin),
  GreenThresMax(thresGreenMax),
  BlueThresMin(thresBlueMin),
  BlueThresMax(thresBlueMax),
  MinArea(areaMin),
  MaxArea(areaMax),
  SizeXY(sizeXY),
  SizeZ(sizeZ),
  MinRoundness(minRoundness)
{
}

Blob::Blob()
{
}

void Blob::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  RedThresMin =      tag->GetPropertyInt(XML_ATTRIBUTE_REDTHRESMIN   , RedThresMin);
  RedThresMax =      tag->GetPropertyInt(XML_ATTRIBUTE_REDTHRESMAX   , RedThresMax);
  GreenThresMin =    tag->GetPropertyInt(XML_ATTRIBUTE_GREENTHRESMIN , GreenThresMin);
  GreenThresMax =    tag->GetPropertyInt(XML_ATTRIBUTE_GREENTHRESMAX , GreenThresMax);
  BlueThresMin =     tag->GetPropertyInt(XML_ATTRIBUTE_BLUETHRESMIN  , BlueThresMin);
  BlueThresMax =     tag->GetPropertyInt(XML_ATTRIBUTE_BLUETHRESMAX  , BlueThresMax);
  MinArea =          tag->GetPropertyInt(XML_ATTRIBUTE_MINAREA       , MinArea);
  MaxArea =          tag->GetPropertyInt(XML_ATTRIBUTE_MAXAREA       , MaxArea);
  SizeXY =        tag->GetPropertyDouble(XML_ATTRIBUTE_SIZEXY        , SizeXY);
  SizeZ =         tag->GetPropertyDouble(XML_ATTRIBUTE_SIZEZ         , SizeZ);
  MinRoundness =  tag->GetPropertyDouble(XML_ATTRIBUTE_MINROUNDNESS  , MinRoundness);
}

Blob::~Blob(void)
{
}


void Blob::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_REDTHRESMIN     ,    RedThresMin);
  tag->AddProperty(XML_ATTRIBUTE_REDTHRESMAX     ,    RedThresMax);
  tag->AddProperty(XML_ATTRIBUTE_GREENTHRESMIN   ,    GreenThresMin);
  tag->AddProperty(XML_ATTRIBUTE_GREENTHRESMAX   ,    GreenThresMax);
  tag->AddProperty(XML_ATTRIBUTE_BLUETHRESMIN    ,    BlueThresMin);
  tag->AddProperty(XML_ATTRIBUTE_BLUETHRESMAX    ,    BlueThresMax);
  tag->AddProperty(XML_ATTRIBUTE_MINAREA         ,    MinArea);
  tag->AddProperty(XML_ATTRIBUTE_MAXAREA         ,    MaxArea);
  tag->AddProperty(XML_ATTRIBUTE_SIZEXY       , SizeXY);
  tag->AddProperty(XML_ATTRIBUTE_SIZEZ        , SizeZ);
  tag->AddProperty(XML_ATTRIBUTE_MINROUNDNESS , MinRoundness);
}


void Blob::Show(RelPose* pose, Sensor* camin)
{

    if(pose != NULL && camin != NULL&& camin->IsCamera())
    {
      Camera* cam = (Camera*)camin;
      Halcon::HWindow* hwin = cam->GetWindow();
      hwin->SetColor("green");
      Halcon::HTuple pose_ht;
      RelPoseHTuple::GetPose(pose, &pose_ht);
      Halcon::HTuple camparam = cam->m_calibration.CamParam();
      Halcon::HTuple r,c;
      Halcon::project_3d_point(pose_ht[0],pose_ht[1],pose_ht[2], camparam, &r,&c);
      Halcon::disp_cross(hwin->WindowHandle(), r, c, 15, 0);
    }
}

