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

#include "PVWObjectClassification.h"

#include "Camera.h"
#include "XMLTag.h"
#include "RegionOI.h"
#include "cpp/HalconCpp.h"


#define XML_ATTRIBUTE_SVMPATH     "svmpath"
#define XML_ATTRIBUTE_DESCRPATH    "descrpath"
#define XML_ATTRIBUTE_NAMEFILEPATH  "namefilepath"

#include <std_msgs/String.h>

#include <sstream>

#include "NamedDescriptor.h"
#include <pluginlib/class_list_macros.h>
#include <python2.6/Python.h>
namespace cop
{
   CREATE_NAMED_DESCRIPTOR(PVWObject, "PVWObject", "UnknownClass", DESCRIPTOR_NAMEDCLASS, std_msgs::String)
}


using namespace Halcon;
using namespace cop;

PVWObjectClassification::PVWObjectClassification()
{
}

void PVWObjectClassification::SetData(XMLTag* tag)
{
   m_svm_filename = tag->GetProperty(XML_ATTRIBUTE_SVMPATH, "");
   try
   {
     //read_class_svm(m_svm_filename.c_str(),  &m_svmHandle);
   }
   catch(...)
   {
     printf("ClassByDPS:  problems reading the svm\n");
   }
   m_descr_filename = tag->GetProperty(XML_ATTRIBUTE_DESCRPATH, "");
   try
   {
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
   }
   catch(...)
   {
     printf("ClassByDPS:  problems reading the tuple with names\n");
   }
   Py_Initialize();
   printf("Python initialized\n");
   std::ostringstream str;
   str << "import roslib ; roslib.load_manifest('cop_bvw_class')\n" <<
          "import sys; print sys.path\n" <<
          "import recognosco.classification\n" <<
          "l = recognosco.classification.Learner(learnerFile=\""<< m_svm_filename.c_str() << "\")\n" <<
          "c = recognosco.classification.Classifier(l)\n";

   printf("Calling %s\n", str.str().c_str());
   PyRun_SimpleString(str.str().c_str());
   printf("Called\n");

}


PVWObjectClassification::~PVWObjectClassification(void)
{
  Py_Finalize();
}

Descriptor* PVWObjectClassification::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  PVWObject *descr = NULL;
  Camera* cam = Camera::GetFirstCamera(sensors);
  printf("in here?\n");
  if(cam != NULL)
  {
    printf("Camera exists\n");
    if(cam->CanSee(*pose))
    {
      printf("Camera can see\n");

      Halcon::HTuple Class;
      Image* img = cam->GetImage(-1);


      numOfObjects = 0;
      if(img != NULL)
      {
        printf("Camera has an image\n");

        std::map<std::string, double> hist;
        Halcon::Hobject regionTemp, image_cropped;
        Halcon::HTuple r1,c1, r2, c2;
        RegionOI* region = new RegionOI(pose, cam->m_relPose->m_uniqueID, &(cam->m_calibration));
        Halcon::dilation_circle(region->m_reg, &regionTemp, 5);
        Halcon::smallest_rectangle1(regionTemp, &r1, &c1, &r2, &c2);
        try
        {

          Halcon::HTuple ptr, width, height, type;

          if(r2 - r1 > 20 && c2 -c1 > 20)
          {
            Halcon::crop_rectangle1(*img->GetHImage(), &image_cropped, r1, c1, r2,c2);
            Halcon::rgb1_to_gray(image_cropped, &image_cropped);
            Halcon::get_image_pointer1(image_cropped, &ptr, &type, &width, &height);
          }
          else
          {
            Halcon::rgb1_to_gray(*img->GetHImage(), &image_cropped);
            Halcon::get_image_pointer1(image_cropped, &ptr, &type, &width, &height);
          }


          std::ostringstream str;


          str << "from PIL import Image\n" <<
                 "import ctypes\n" <<
                 "string1 = ctypes.string_at("<<ptr[0].L() << "," <<(width*height)[0].L() << ")\n" <<
                 "image_cur = Image.frombuffer('L', ("<< width[0].L() << "," << height[0].L() << "), string1, 'raw', 'L', 0, 1)\n" <<
                 "image_cur.save('test.jpg', 'JPEG')\n"  <<
                 "labels = c.classify(image_cur, confidenceThreshold=0.03)\n" <<
                 "if len(labels) > 0:\n" <<
                 "  label = labels[0]\n" <<
                 "else:\n" <<
                 "  label = \"None\"\n";
                 /* 0.03 - 0.10 als relevanz**/
                 /*   Test if image is good: */
          printf("calling:\n%s\n\n", str.str().c_str());

          PyRun_SimpleString(str.str().c_str());
          PyObject* evalModule;
          PyObject* evalDict;
          PyObject* evalVal;
          char* retString;

          evalModule = PyImport_AddModule( (char*)"__main__" );
          evalDict = PyModule_GetDict( evalModule );
          evalVal = PyDict_GetItemString( evalDict, "label" );

          if( evalVal == NULL )
          {
            PyErr_Print();
          }
          else
          {
            /*
            * PyString_AsString returns char* repr of PyObject, which should
            * not be modified in any way...this should probably be copied for
            * safety
            */
            retString = PyString_AsString( evalVal );
          }
          printf("Result in c: %s\n",retString);
          std::string tmp(retString);
          std_msgs::String st;
          if(tmp.length() > 0 && tmp.compare("None") != 0 && tmp.compare("<not enough features>") != 0)
          {
            st.data = tmp;
            descr = new PVWObject();
            descr->SetContent(st);
            descr->SetClass(st.data);
            descr->Evaluate(1.0, 100.0);
            qualityMeasure = 1.0;
            printf("Descriptor was set\n");
            numOfObjects = 1;
          }
          else
          {
            printf("PVWObjectClassification: No class assigned\n");
            numOfObjects = 0;
            descr = NULL;
          }
        }
        catch(Halcon::HException &ex)
        {
          printf("Error in PVWObjectClassification: %s\n", ex.message);
        }
        catch(...)
        {
         printf("Error in  PVWObjectClassification::Perform\n");
        }
     }
     else
     {
       printf("Camera has no images\n");
     }
   }

  }
  return descr;

}

double PVWObjectClassification::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sensors)
{
  if(sig.GetElement(0, DESCRIPTOR_NAMEDCLASS) == NULL )
  {
    if(Camera::GetFirstCamera(sensors) != NULL)
      return 1.0;
    else
      return 0.0;
  }
  else
    return 0.0;
}

XMLTag* PVWObjectClassification::Save()
{
	XMLTag* tag = new XMLTag(GetName());
        tag->AddProperty(XML_ATTRIBUTE_SVMPATH, m_svm_filename);
        tag->AddProperty(XML_ATTRIBUTE_DESCRPATH, m_descr_filename);
        tag->AddProperty(XML_ATTRIBUTE_NAMEFILEPATH, m_tuple_filename);
	//TODO: parameter?
	return tag;
}

PLUGINLIB_REGISTER_CLASS(PVWObject, cop::PVWObject, Descriptor);
PLUGINLIB_REGISTER_CLASS(PVWObjectClassification, cop::PVWObjectClassification, RefineAlgorithm);
