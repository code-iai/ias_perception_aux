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


#include "ColorModelFromCAD.h"
#include "XMLTag.h"
#include "ShapeModel.h"


#include <cpp/HalconCpp.h>


using namespace cop;


ColorModelFromCAD::ColorModelFromCAD(Class* classref, Signature* sig)
: ColorModel(classref)
{
  ShapeModel* sm = (ShapeModel*)sig->GetElement(0, DESCRIPTOR_SHAPE);
  Image* img = (Image*)sm->GetLastMatchedImage();

  try
  {
   if(sig->GetObjectPose() != NULL)
   {
     Halcon::Hobject xld = sm->GetContour(*sig->GetObjectPose());
    int num = 0;
    Halcon::count_obj(xld, (Hlong*)&num);
    printf("Point Num Xld: %d\n", num);
    Halcon::Hobject region;
    Halcon::gen_empty_region(&region);
    for(int i = 0; i < num; i++)
    {
         Halcon::Hobject obj;
         Halcon::gen_region_contour_xld(xld, &obj ,"filled");
         Halcon::union2(obj, region, &region);
    }
    Halcon::connection(region, &region);
    Halcon::fill_up(region, &region);
    Halcon::Hobject reducedimg;
    Halcon::union1(region, &region);
    Halcon::HTuple area, r,c;
    Halcon::area_center(region, &area, &r, &c);
    SetSize(area[0].L());
    Halcon::reduce_domain(*img->GetHImage(), region, &reducedimg);
    Halcon::count_channels(reducedimg, (Hlong*)&num);
    for(int i = 0; i < num; i++)
    {
         Halcon::HTuple abshisto;
         Halcon::HTuple relhisto;
         Halcon::Hobject obj;
         Halcon::access_channel(reducedimg, &obj, i + 1);
         Halcon::gray_histo(region, obj, &abshisto, &relhisto);
         for(int j = 0; j < relhisto.Num(); j++)
         {
                 AddColorSpec(relhisto[j].D());
         }
     }
    }
   }
   catch(Halcon::HException ex)
   {
      printf("Error while Learning: %s\n", ex.message);
   }
}
