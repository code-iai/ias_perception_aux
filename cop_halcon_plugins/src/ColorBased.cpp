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
                        ColorBased.cpp - Copyright klank

**************************************************************************/

#include "ColorBased.h"
#include "ColorModel.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RelPoseHTuple.h"

#include "cpp/HalconCpp.h"



using namespace cop;

// Constructors/Destructors
//
ColorBased::ColorBased ( ) :
  LocateAlgorithm()
{
}

ColorBased::ColorBased ( XMLTag* tag) :
  LocateAlgorithm()
{
}


ColorBased::~ColorBased ( ) { }

//
// Methods
//
XMLTag* ColorBased::Save()
{
  XMLTag* tag = new XMLTag(XLM_NODE_COLORBASED);
  return tag;
}

std::vector<RelPose*> ColorBased::Perform(std::vector<Sensor*> sensors, RelPose* , Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Image* img = cam->GetImage(-1);
    RelPose* campose = cam->m_relPose;

    if(campose != NULL)
    {
      const Halcon::Hobject* image = img->GetHImage();
      if(img != NULL)
      {
        char* r, *g , *b;
        Hlong width, height;
        char type[30];
        ColorModel* elem = (ColorModel*)object.GetElement(0, DESCRIPTOR_COLOR);
        int numChannels;	Halcon::count_channels(*image, (Hlong*)&numChannels);
        if(numChannels == 1)
          Halcon::get_image_pointer1(*image, (Hlong*)&r, type, &width, &height);
        else
          Halcon::get_image_pointer3(*image, (Hlong*)&r,(Hlong*)&g,(Hlong*)&b, type, &width, &height);
        int size = elem->GetSize();
        int patchheight = (int)sqrt((double)size);
        int patchwidth = (int)sqrt((double)size); //TODO randbehandlung
        double val, min = 1.0;
        int rowMax = -1, colMax = -1;
        for(int row = 0; row < height; row += patchheight)
        {
          for(int col = 0; col < width; col+= patchwidth)
          {
            Halcon::Hobject region, reducedimg;
            Halcon::gen_rectangle1(&region, row, col, row + patchheight, col + patchwidth);
            Halcon::reduce_domain(*image, region, &reducedimg);
            std::vector<double> temp;
            for(int i = 0; i < numChannels; i++)
            {
              Halcon::HTuple abshisto;
              Halcon::HTuple relhisto;
              Halcon::Hobject obj;
              Halcon::access_channel(reducedimg, &obj, i + 1);
              Halcon::gray_histo(region, obj, &abshisto, &relhisto);
              for(int j = 0; j < relhisto.Num(); j++)
              {
                temp.push_back(relhisto[j].D());
              }
            }
            val = elem->Compare(temp);
            if(val < min)
            {
              min = val;
              rowMax = row + (patchheight / 2);
              colMax = col + (patchwidth / 2);
            }
          }
        }
        img->Free();
        if(min < 0.1)
        {
          numOfObjects = 1;
          qualityMeasure = min;
          qualityMeasure = 0.5;
          Halcon::HTuple tup, cov;
          Halcon::tuple_gen_const(7,0,&tup);
          Halcon::tuple_gen_const(6,0,&cov);
          tup[0] = (double)(colMax - (width  / 2)) / sqrt((double)width * height );
          tup[1] = (double)(rowMax - (height / 2)) / sqrt((double)width *  height );
          printf("Color Based: %d, %d\n", rowMax, colMax);
          tup[2] = 1.0;
          if(object.GetObjectPose() != NULL)
            result.push_back(RelPoseHTuple::FRelPose(tup, cov, campose, object.GetObjectPose()->m_uniqueID));
          else
            result.push_back(RelPoseHTuple::FRelPose(tup, cov, campose));
#ifdef _DEBUG
          RelPoseHTuple::Print(result[0]);
#endif
          return result;
        }
      }
    }

  }
  numOfObjects = 0;
  qualityMeasure = 0.0;
  return result;
}


double ColorBased::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  if(object.GetElement(0, DESCRIPTOR_COLOR) != NULL)
    return 1.0;
  else return 0.0;
}

