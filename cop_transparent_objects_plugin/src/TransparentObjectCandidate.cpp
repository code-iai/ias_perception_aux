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


#include "TransparentObjectCandidate.h"
#include "XMLTag.h"
#include "RelPoseHTuple.h"

#include <cpp/HalconCpp.h>

#define XML_ATTRIBUTE_REDTHRESMIN   "RedThresMin"


using namespace cop;

void TransparentObjectCandidate::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);

}

void TransparentObjectCandidate::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
}


void TransparentObjectCandidate::Show(RelPose* pose, Sensor* camin)
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

Elem* TransparentObjectCandidate::Duplicate(bool bStaticCopy)
{
  TransparentObjectCandidate* new_obj = (TransparentObjectCandidate*)Descriptor::Duplicate(bStaticCopy);
  /** Assign TransparentObjectCandidate Memebers*/
  new_obj->m_detectedRegions = m_detectedRegions;
  new_obj->m_usedDistance = m_usedDistance;
  new_obj->m_usedIntensities = m_usedIntensities;

  /** Assign Descriptor Memebers*/
  new_obj->m_class = m_class;
  new_obj->m_imgLastMatchReading = m_imgLastMatchReading;
  new_obj->m_poseLastMatchReading = m_poseLastMatchReading;
  new_obj->m_qualityMeasure = m_qualityMeasure;

  new_obj->m_usedImageX1 = m_usedImageX1;
  new_obj->m_usedImageY1 = m_usedImageY1;
  new_obj->m_usedImageZ1 = m_usedImageZ1;

  return new_obj;
}

