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


#include "ColorClass.h"
#include "CheckColorClass.h"




#define XML_NODE_CHECKCOLORCLASS "CheckColorClass"
#define XML_ATTRIBUTE_MLPPATH "mlpPath"



#include <cpp/HalconCpp.h>



using namespace cop;

// Procedure declarations
// Local procedures
void classify_colors_score (Halcon::Hobject Image, Halcon::Hobject Region, Halcon::Hobject *ClassRegions,
    Halcon::HTuple MLPHandle, Halcon::HTuple Colors, Halcon::HTuple *Color, Halcon::HTuple *Score);
void read_color_mlp (Halcon::HTuple Path, Halcon::HTuple *MLPHandle, Halcon::HTuple *Colors);

CheckColorClass::CheckColorClass(std::string path) :
  m_stPath(path),
  m_MLPHandle(-1)
{

  Halcon::HTuple mlp;
  try
  {
  read_color_mlp(m_stPath.c_str(), &mlp, &m_Colors);
  }
  catch(Halcon::HException ex)
  {
    printf("Error in loading mlp: %s\n", ex.message);
    throw "Error loading mlp";
  }
  m_MLPHandle = mlp[0].I();
}


void classify_colors_score (Halcon::Hobject Image, Halcon::Hobject Region, Halcon::Hobject *ClassRegions,
    Halcon::HTuple MLPHandle, Halcon::HTuple Colors, Halcon::HTuple *Color, Halcon::HTuple *Score)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  ImageReduced, RegionTemp, img_equ;

  // Local control variables
  HTuple  Number, Area, Row, Column, Indices, Area1;
  HTuple  Row1, Column1, Sum, WindowHandle;
  area_center(Region, &Area1, &Row, &Column);
  erosion_circle(Region, &RegionTemp, sqrt((double)Area1[0].I()) / 3);
  equ_histo_image(Image, &img_equ);
  reduce_domain(img_equ, RegionTemp, &ImageReduced);

/*    open_window(200, 200, 400, 400, 0, "visible", "", &WindowHandle);
      set_part(WindowHandle, 0, 0, 1600, 1200);
      disp_obj(ImageReduced, WindowHandle);*/
  classify_image_class_mlp(ImageReduced, &(*ClassRegions), MLPHandle, 0.2);
  count_obj((*ClassRegions), &Number);
  area_center((*ClassRegions), &Area, &Row, &Column);
  for(int i = 0; i < Area.Num(); i++)
    printf("%d/%d(%s) ",  Area[i].I(), Area1[0].I(), Colors[i].S());
  printf("\n");
//  cout <<"AreaTupleCheckColor: "<< Area.ToString("04d") << std::endl;
  tuple_sort_index(Area, &Indices);
  tuple_sum(Area, &Sum);
  if(Sum[0].I() == 0)
    (*Score) = 0.0;
  else
    (*Score) = (HTuple(Area).Real())/(Sum.Real());
  (*Color) = Colors;
  return;
}

void read_color_mlp (Halcon::HTuple Path, Halcon::HTuple *MLPHandle, Halcon::HTuple *Colors)
{
  using namespace Halcon;
  try
  {
    read_class_mlp(Path+"color_classification_mlp.gmc", &(*MLPHandle));
  }
  catch(Halcon::HException ex)
  {
    printf("Error reading class mlp: %s\n", ex.message);
  }
  try
  {
    read_tuple(Path+"color_classification_mlp_colors.dat", &(*Colors));
  }
  catch(Halcon::HException ex)
  {
    printf("Error reading color classes: %s\n", ex.message);
  }
  return;
}

CheckColorClass::CheckColorClass()
{
}

void CheckColorClass::SetData(XMLTag* tag)
{
  printf("Loading Algorithm CheckColorClass\n");
  m_stPath = tag->GetProperty(XML_ATTRIBUTE_MLPPATH);
  Halcon::HTuple mlp;
  try
  {
    read_color_mlp(m_stPath.c_str(), &mlp,  &m_Colors);
  }
  catch(Halcon::HException ex)
  {
    printf("Error in loading mlp: %s\n", ex.message);
    throw "Error loading mlp";
  }
  m_MLPHandle = mlp[0].I();
}


CheckColorClass::~CheckColorClass(void)
{


  using namespace Halcon;
  clear_all_class_mlp();

}


XMLTag* CheckColorClass::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_CHECKCOLORCLASS);
  tag->AddProperty(XML_ATTRIBUTE_MLPPATH, m_stPath);
  return tag;
}

void CheckColorClass::Inner(Halcon::Hobject *img, Halcon::Hobject *region, std::string &color, std::map<std::string, double> &histo_cmp, double &max_score)
{
  Halcon::Hobject ClassRegions;
  Halcon::HTuple Color, Score;
  try
  {
    classify_colors_score(*img, *region, &ClassRegions, m_MLPHandle, m_Colors, &Color, &Score);
    max_score = -1.0;
    if(Score.Num() > 1)
    {
      for(int i = 0; i < Score.Num(); i++)
      {
        histo_cmp[Color[i].S()] = Score[i].D();
        if(max_score < Score[i].D())
        {
          max_score = Score[i].D();
          color = Color[i].S();
        }
      }
    }


  }
  catch(Halcon::HException ex)
  {
    printf("Error in CheckColorClass::Inner: %s\n", ex.message);
    color = "";
  }
}

// Public attribute accessor methods
//
std::vector<RelPose*> CheckColorClass::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{

  int i = 0;
  std::vector<ColorClass*> possibleColors;
  std::vector<RelPose*> results;
  numOfObjects = 0;
  qualityMeasure = 0;
  Camera* cam = Camera::GetFirstCamera(sensors);
  while (true)
  {
    Elem *cl = object.GetElement(i++,DESCRIPTOR_COLORCLASS);
    if(cl != NULL)
    {
      possibleColors.push_back((ColorClass*)cl);
    }
    else
      break;
  }
  if(possibleColors.size() > 0 && cam != NULL)
  {
    Halcon::HTuple Color;
    Halcon::HTuple Score;
    bool added_pose = false;
    Image* img = cam->GetImage(-1);
    RegionOI* region = new RegionOI(pose, cam->m_relPose->m_uniqueID, &(cam->m_calibration));
    if(img != NULL && region != NULL && img->GetType() == ReadingType_HalconImage)
    {
      Halcon::Hobject* obj = img->GetHImage();
      Halcon::Hobject& reg = region->GetRegion();
      std::string st;
      double max_score;
      std::map<std::string, double> hist;
      Inner(obj, &reg, st, hist, max_score);
      for(size_t i = 0; i < possibleColors.size(); i++)
      {
        ColorClass* cl = (ColorClass*)possibleColors[i];
        double d  = cl->IsClass(hist);
        printf("The proposed pose %ld has a Color Compatibility of %f with %s(threshold is %f)\n", pose->m_uniqueID, d, cl->GetMainColorName().c_str(), qualityMeasure);
        if(d > qualityMeasure)
        {
          numOfObjects = 1;
          pose->m_qualityMeasure = d;
          qualityMeasure = d;
          if(!added_pose)
          {
            results.push_back(pose);
            added_pose = true;
          }
        }
      }
    }
    else
    {
      printf("Error: %p %p\n",img, region);
    }
    img->Free();
    delete region;
  }
  return results;

}

double CheckColorClass::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  if(object.GetElement(0, DESCRIPTOR_COLORCLASS) != NULL)
  {
   if(Camera::GetFirstCamera(sensors) != NULL)
    return 1.0;
  else
    return 0.0;

  }
  else
    return 0.0;
}
