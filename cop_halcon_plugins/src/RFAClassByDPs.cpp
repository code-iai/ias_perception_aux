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

#include "RFAClassByDPs.h"

#include "Camera.h"
#include "XMLTag.h"
#include "RegionOI.h"
#include "cpp/HalconCpp.h"
#include "HCPPdescr_class.h"


#define XML_ATTRIBUTE_SVMPATH     "svmpath"
#define XML_ATTRIBUTE_DESCRPATH    "descrpath"
#define XML_ATTRIBUTE_NAMEFILEPATH  "namefilepath"

#include <std_msgs/String.h>

#include "NamedDescriptor.h"
#include <pluginlib/class_list_macros.h>

namespace cop
{
  CREATE_NAMED_DESCRIPTOR(NamedClass, "NamedClass", "UnknownClass", DESCRIPTOR_NAMEDCLASS, std_msgs::String)
}

// Procedure declarations
void classify_object (Halcon::Hobject Region, Halcon::Hobject Image,
    Halcon::HTuple DescriptorHandle, Halcon::HTuple SVMHandle, Halcon::HTuple *Class, double &score);

using namespace Halcon;

using namespace cop;
RFAClassByDPs::RFAClassByDPs()
{
}

void RFAClassByDPs::SetData(XMLTag* tag)
{
   m_svm_filename = tag->GetProperty(XML_ATTRIBUTE_SVMPATH, "");
   try
   {
     read_class_svm(m_svm_filename.c_str(),  &m_svmHandle);
   }
   catch(...)
   {
     printf("ClassByDPS:  problems reading the svm\n");
   }
   m_descr_filename = tag->GetProperty(XML_ATTRIBUTE_DESCRPATH, "");
   try
   {
     Halcon::HTuple tup;
     read_descriptor_model_3d(m_descr_filename.c_str(), &tup);
     m_descrHandle = tup[0].L();
   }
   catch(Halcon::HException ex)
   {
     printf("ClassByDPS:  %s\n", ex.message);
   }
   catch(...)
   {
        printf("ClassByDPS:  problems reading the descr\n");
   }
   m_tuple_filename = tag->GetProperty(XML_ATTRIBUTE_NAMEFILEPATH, "");
   try
   {
     HTuple tup_names;
     read_tuple(m_tuple_filename.c_str(),  &tup_names);
     for(int i = 0; i < tup_names.Num(); i++)
     {
       m_classNames.push_back(tup_names[i].S());
     }
   }
   catch(...)
   {
     printf("ClassByDPS:  problems reading the tuple with names\n");
   }

}


RFAClassByDPs::~RFAClassByDPs(void)
{
  clear_descriptor_model_3d(m_descrHandle);
  clear_class_svm(m_svmHandle);
}

Descriptor* RFAClassByDPs::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  NamedClass *descr = NULL;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    if(cam->CanSee(*pose))
    {
      Halcon::HTuple Class;
      Image* img = cam->GetImage(-1);
      std::map<std::string, double> hist;
      RegionOI* region = new RegionOI(pose, cam->m_relPose->m_uniqueID, &(cam->m_calibration));
      try
      {
        double score;
        classify_object (region->GetRegion(), *(img->GetHImage()),
           m_descrHandle, m_svmHandle, &Class, score);
        std_msgs::String st;
        if(Class[0].L() != -1)
        {
          st.data = m_classNames[Class[0].L()];
          descr = new NamedClass();
          descr->SetContent(st);
          descr->SetClass(st.data);
          descr->Evaluate(score, 100.0);
          qualityMeasure = score;
        }
        else
        {
          printf("RFAClassByDPs: No class assigned\n");
        }
      }
      catch(Halcon::HException &ex)
      {
        printf("Error in RFAClassByDPs: %s\n", ex.message);
      }
      catch(...)
      {
       printf("Error in  RFAClassByDPs::Perform\n");
      }
    }

  }
  return descr;

}

double RFAClassByDPs::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sens)
{
  if(sig.GetElement(0, DESCRIPTOR_NAMEDCLASS) == NULL)
    return 1.0;
  else
    return -0.0;
}

XMLTag* RFAClassByDPs::Save()
{
	XMLTag* tag = new XMLTag(GetName());
        tag->AddProperty(XML_ATTRIBUTE_SVMPATH, m_svm_filename);
        tag->AddProperty(XML_ATTRIBUTE_DESCRPATH, m_descr_filename);
        tag->AddProperty(XML_ATTRIBUTE_NAMEFILEPATH, m_tuple_filename);
	//TODO: parameter?
	return tag;
}

#define PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(class_name, class_type, base_class_type) \
  POCO_BEGIN_NAMED_MANIFEST(class_name, base_class_type) \
    POCO_EXPORT_CLASS(class_type) \
      POCO_END_MANIFEST
      

PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(NamedClass, NamedClass, Descriptor);
