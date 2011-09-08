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


#include "CalTab.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RelPoseHTuple.h"

#define XML_ATTRIBUTE_CALIBNAME "CalibFileName"
#define XML_ATTRIBUTE_INIT_CALTHRES "CalThres"
#define XML_ATTRIBUTE_INIT_HOLES "Holes"
#define XML_ATTRIBUTE_INIT_STARTTHRES "StartThres"
#define XML_ATTRIBUTE_INIT_STEPS "Steps"
#define XML_ATTRIBUTE_INIT_MT "MinThres"
#define XML_ATTRIBUTE_INIT_CONT "ContLength"
#define XML_ATTRIBUTE_INIT_ALPHA "Alpha"
#define XML_ATTRIBUTE_INIT_DIAM "Diam"


#include <cpp/HalconCpp.h>

using namespace cop;


CalTab::CalTab(std::string stFileName) :
  Descriptor(new Class("CalTab", Elem::m_LastID)),
  m_stCalTabDescriptionFile(stFileName),
  m_alpha(0.3),
  m_st(169),
  m_steps(1),
  m_mt(13),
  m_cont(18),
  m_diam(100)
{
}
CalTab::CalTab()
{
}

void CalTab::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  m_stCalTabDescriptionFile = (tag->GetProperty(XML_ATTRIBUTE_CALIBNAME));
  m_alpha = (tag->GetPropertyDouble( XML_ATTRIBUTE_INIT_ALPHA,   0.3));
  m_st = (tag->GetPropertyInt(    XML_ATTRIBUTE_INIT_STARTTHRES, 169));
  m_steps = (tag->GetPropertyDouble( XML_ATTRIBUTE_INIT_STEPS, 1));
  m_mt = (tag->GetPropertyInt(    XML_ATTRIBUTE_INIT_MT, 13));
  m_cont = (tag->GetPropertyInt(  XML_ATTRIBUTE_INIT_CONT, 18));
  m_diam = (tag->GetPropertyInt(  XML_ATTRIBUTE_INIT_DIAM, 100 ));
  m_calThres = (tag->GetPropertyInt(  XML_ATTRIBUTE_INIT_CALTHRES, 97 ));
  m_holes = (tag->GetPropertyInt(  XML_ATTRIBUTE_INIT_HOLES, 10 ));
}

void CalTab::SaveTo(XMLTag* tag)
{
  Descriptor::SaveTo(tag);
  tag->AddProperty(XML_ATTRIBUTE_CALIBNAME, m_stCalTabDescriptionFile);
  tag->AddProperty(XML_ATTRIBUTE_INIT_ALPHA, m_alpha);
  tag->AddProperty(XML_ATTRIBUTE_INIT_STARTTHRES, m_st);
  tag->AddProperty(XML_ATTRIBUTE_INIT_STEPS, m_steps);
  tag->AddProperty(XML_ATTRIBUTE_INIT_MT, m_mt);
  tag->AddProperty(XML_ATTRIBUTE_INIT_CONT, m_cont);
  tag->AddProperty(XML_ATTRIBUTE_INIT_DIAM, m_diam);
  tag->AddProperty(XML_ATTRIBUTE_INIT_CALTHRES, m_calThres);
  tag->AddProperty(XML_ATTRIBUTE_INIT_HOLES, m_holes);
}


void CalTab::Show(RelPose* pose, Sensor* cam_in)
{
    if(pose != NULL && cam_in != NULL && cam_in->IsCamera())
    {
      Camera* cam = (Camera*)cam_in;
      Halcon::HWindow* hwin = (cam)->GetWindow();
      hwin->SetColor("red");
      Halcon::HTuple pose_ht;
      RelPoseHTuple::GetPose(pose, &pose_ht);
      Halcon::disp_caltab(hwin->WindowHandle(), m_stCalTabDescriptionFile.c_str(), cam->m_calibration.CamParam(), pose_ht, 1);
    }
}

CalTab::~CalTab(void)
{
}
