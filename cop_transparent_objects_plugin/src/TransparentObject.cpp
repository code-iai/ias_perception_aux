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


#include "TransparentObject.h"
#include "XMLTag.h"
#include "RelPoseHTuple.h"

#include <cpp/HalconCpp.h>

#define XML_ATTRIBUTE_REDTHRESMIN   "RedThresMin"


using namespace cop;

void TransparentObject::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);

}


void TransparentObject::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
}


void TransparentObject::Show(RelPose* pose, Sensor* camin)
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

