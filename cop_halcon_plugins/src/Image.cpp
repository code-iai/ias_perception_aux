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


/*****************************************************************
                        Image.cpp - Copyright klank

**************************************************************************/

#include "IplImageReading.h"

#include "Image.h"

#include "cpp/HalconCpp.h"

#include "XMLTag.h"
#include <sstream>

//#include "IplImageReading.h"

using namespace cop;



// Constructors/Destructors
//

Image::Image (int type)  :
  Reading(ReadingType_HalconImage),
  m_image(NULL),
  m_type(type)
{
}

Image::Image ()  :
  Reading(ReadingType_HalconImage),
 m_image(NULL),
  m_type(GRAY_IMAGE)
{
}

Image::Image ( const Image& img) :
   Reading(ReadingType_HalconImage),
   m_image(img.m_image),
   m_type(img.m_type)
{
};

#ifdef HALCONIMG
Image::Image ( Halcon::Hobject* img, int type, RelPose* pose) :
    Reading(ReadingType_HalconImage),
    m_image(img),
    m_type(type)
{
  SetPose(pose);
}
#endif

Image::~Image ( )
{
    if(m_usageCount > 0)
        throw "deleting error";
    if(m_image!= NULL)
        delete m_image;
}

void Image::SetData(XMLTag* tag)
{
  Reading::m_readingType = ReadingType_HalconImage;
  m_type = (GRAY_IMAGE);

    std::string stFileName = tag->GetProperty(XML_ATTRIBUTE_FILENAME);

    if(stFileName.length() == 0)
        throw "No Image-Filename Specified in xml tag";
    m_image = new Halcon::Hobject();
    try
    {
        Halcon::read_image(m_image, stFileName.c_str());
    }
    catch(...)
    {
        delete m_image;
        m_image = NULL;
        throw "File not found: read image from file";
    }
    Halcon::HTuple num;
    Halcon::count_obj(*m_image, &num);
    m_type = tag->GetPropertyInt(XML_ATTRIBUTE_IMGTYPE, num == 1 ? GRAY_IMAGE : RGB_IMAGE);

}

//
// Methods
//
Halcon::Hobject* Image::GetHImage() const
{
    //Halcon::HTuple a,b,c,d;
    //Halcon::get_image_pointer1(*m_image, &a,&b,&c,&d);
    return m_image;
}


XMLTag* Image::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_IMAGEFILE);
    std::ostringstream os;
    os << "img" << tag->date() << ".png";
    try
    {
        Halcon::write_image(*m_image, "png", 0, os.str().c_str());
        tag->AddProperty(XML_ATTRIBUTE_FILENAME, os.str());
        tag->AddProperty(XML_ATTRIBUTE_IMGTYPE, m_type);
    }
    catch(...)
    {
        printf("Image File not found!!\n");
    }
    return tag;
}

void Image::Delete(XMLTag* tag)
{

}
Reading* Image::Clone()
{

    Halcon::Hobject* obj = new Halcon::Hobject();
    Halcon::copy_image(*m_image, obj);
    Image* img = new Image(obj, m_type);
    return img;
}


namespace cop
{
  class Image2IplImage : public ReadingConverter
  {
      Reading* Convert(Reading* in)
      {
        printf("Conversion Requested from Hobject to cv::Mat\n");
        Hlong img[3], width, height;
        Hlong channels = 0;
        char type[20];
        Halcon::count_channels(*((Image*)in)->GetHImage(), &channels);
        Herror err;
        if(channels == 3)
        {
          printf("Its a 3 Channel image\n");
          err = Halcon::get_image_pointer3(*((Image*)in)->GetHImage(), &img[0], &img[1], &img[2],type ,&width, &height);
          printf("Request a matrix of type CV_8UC3\n");
          cv::Mat mat((int)height, (int)width, CV_8UC3);
          unsigned char* pin1 = ((unsigned char*)img[0]);
          unsigned char* pin2 = ((unsigned char*)img[1]);
          unsigned char* pin3 = ((unsigned char*)img[2]);
          unsigned char* pout = ((unsigned char*)mat.data);
          for(long i = 0; i < width*height; i++)
          {
            pout[i *  3]     = pin1[i];
            pout[i *  3 + 1] = pin2[i];
            pout[i *  3 + 2] = pin3[i];
          }
          return new IplImageReading(mat);

        }
        else
        {
          cv::Mat mat((int)height, (int)width, CV_8UC1);
          err = Halcon::get_image_pointer1(*((Image*)in)->GetHImage(), &img[0],type ,&width, &height);
          unsigned char* pout = ((unsigned char*)mat.data);
          unsigned char* pin1 = ((unsigned char*)img[0]);
          for(long i = 0; i < width*height; i++)
          {
            pout[i] = pin1[i];
          }
          return new IplImageReading(mat);
        }
      }
      ReadingType_t TypeIn(){return ReadingType_HalconImage;}
      ReadingType_t TypeOut(){return ReadingType_IplImage;}
  };
  class IplImage2Image : public ReadingConverter
  {
      Reading* Convert(Reading* in)
      {
        int type = RGB_IMAGE;
        Image* img = new Image(type);
        img->m_image = new Halcon::Hobject();
        IplImageReading* reading_conv = (IplImageReading*)in;
        Halcon::gen_image_interleaved(img->m_image, (Hlong)reading_conv->m_image.data, "rgb",
                       reading_conv->m_image.cols, reading_conv->m_image.rows, 0, "byte", 0, 0,  0, 0, -1, 0);
        return img;
      }
      ReadingType_t TypeOut(){return ReadingType_HalconImage;}
      ReadingType_t TypeIn(){return ReadingType_IplImage;}
  };


}
void Image::RegisterImageConverter()
{
  Reading::s_conv[std::pair<ReadingType_t, ReadingType_t>(ReadingType_HalconImage, ReadingType_IplImage)] = new Image2IplImage();
  Reading::s_conv[std::pair<ReadingType_t, ReadingType_t>(ReadingType_IplImage, ReadingType_HalconImage)] = new IplImage2Image();
}
