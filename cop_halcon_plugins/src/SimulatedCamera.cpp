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
                        SimulatedCamera.cpp - Copyright klank

**************************************************************************/

#include "SimulatedCamera.h"

#include "cpp/HalconCpp.h"

#include "XMLTag.h"

#define MAX_SIM_IMAGES 10
using namespace cop;


SimulatedCamera::SimulatedCamera()
{
}

void SimulatedCamera::SetData(XMLTag* tag)
{
    Camera::SetData(tag);
    m_sourceIsActiv = (false);
    m_sourceType = (0);
    m_isVideo = (false);

    m_acqHandle = (NULL);


    if(tag != NULL)
    {
        m_sourceIsActiv = tag->GetPropertyInt(XML_ATTRIBUTE_SOURCEISACTIVE) != 0;
        m_sourceType = tag->GetPropertyInt(XML_ATTRIBUTE_SOURCETYPE);
        XMLTag* tagName = tag->GetChild(XML_NODE_IMAGE_FILE);
        int i = 0;
        while(tagName != NULL)
        {
            std::string st = XMLTag::Load(tagName, &st);
            m_sourceFileName.push_back(st);
            if(!m_sourceIsActiv && m_sourceType == 0 && m_sourceFileName[i].length() > 0)
            {
                try
                {
                    PushBack(ReadFromFile(m_sourceFileName[i]));
                }
                catch(char const* text)
                {
                    printf("Error Reading images: %s\n", text);
                }
            }
            i++;
            tagName = tag->GetChild(XML_NODE_IMAGE_FILE, i);
        }
    }
}

SimulatedCamera::~SimulatedCamera ( )
{
    Stop();

    delete m_win;
    delete m_acqHandle;

}

Image* SimulatedCamera::ReadFromFile(std::string filename)
{
    std::string stExtension = filename.substr(filename.length() - 3, 3);
    if(stExtension.compare("jpg") == 0 || stExtension.compare("png") == 0 || stExtension.compare("tiff") == 0 || stExtension.compare("tif") == 0)
    {


        try
        {
            Halcon::Hobject* obj = new Halcon::Hobject();
            if(obj == NULL)
                throw("Bad Error");
            Halcon::read_image(obj, filename.c_str());
            Halcon::HTuple chan;
            Halcon::count_channels(*obj, &chan);
            int type = chan[0].I() > 1 ? RGB_IMAGE : GRAY_IMAGE;
            return new Image(obj, type);
        }
        catch(Halcon::HException& except)
        {
            printf("%s  (Path: %s)\n", except.message, filename.c_str());
            throw ("Image not loaded\n");
        }

    }
    else if(stExtension.compare("avi") == 0)
    {
        m_isVideo = true;

        Halcon::Hobject* obj = new Halcon::Hobject();
        if(m_acqHandle != NULL)
            delete m_acqHandle;
        m_acqHandle = new Halcon::HTuple();
        try
        {
        Halcon::open_framegrabber("File", 1,1,0,0,0,0,"default",-1,"default", -1,"false",
                                  filename.c_str(), "default", 1, -1,
                                  m_acqHandle);
        }
        catch(Halcon::HException ex)
        {
            printf("Error loading simulated camera: %s\n",ex.message);
            throw("Error in simulated camera\n");
        }
        try
        {
            Halcon::grab_image_start(*m_acqHandle, -1);
            Halcon::grab_image_async(obj, *m_acqHandle, -1);
            Halcon::HTuple chan;
            Halcon::count_channels(*obj, &chan);
            int type = chan[0].I() > 1 ? RGB_IMAGE : GRAY_IMAGE;
            return new Image(obj, type);
        }
        catch(Halcon::HException& except)
        {
            printf("%s  (Path: %s)\n", except.message, filename.c_str());
            throw ("Video not loaded\n");
        }

    }
    else /* Assume directory*/
    {

        Halcon::HTuple ImageFiles, param, regex;
        param = "files";
        param.Append("follow_links");
        regex = "\\.(tif|tiff|gif|bmp|jpg|jpeg|jp2|png|pcx|pgm|ppm|pbm|xwd|ima)$";
        regex.Append("ignore_case");
        Halcon::list_files (filename.c_str(), param, &ImageFiles);
        Halcon::tuple_regexp_select (ImageFiles, regex, &ImageFiles);
        for(int i= 1; i < ImageFiles.Num(); i++)
        {
            Halcon::Hobject* obj = new Halcon::Hobject();
            try
            {
                Halcon::read_image (obj, ImageFiles[i].S());
                Halcon::HTuple chan;
                Halcon::count_channels(*obj, &chan);
                int type = chan[0].I() > 1 ? RGB_IMAGE : GRAY_IMAGE;
                if(i < ImageFiles.Num() - 1)
                    PushBack(new Image(obj, type));
            }
            catch(Halcon::HException& except)
            {
                printf("%s  (Path: %s)\n", except.message, filename.c_str());
                throw ("Directory not loaded\n");
            }
        }


    }
    return new Image(NO_IMAGE);
}

//
// Methods
//
Reading* SimulatedCamera::GetReading(const long &Frame)
{
    if(Frame != -1 && m_FrameCount > Frame)
    {
        return GetReading_Lock(Frame - m_deletedOffset);
    }
    else
    {
        int nFrame = 0;
        if(m_isVideo) // TODO  Frame < deleteObjects!!!
        {
            nFrame = Frame == -1 ? m_FrameCount: Frame;

            while(m_FrameCount <= nFrame)
            {
                Halcon::Hobject* obj = new Halcon::Hobject();
                Halcon::grab_image_async(obj, *m_acqHandle, -1);
                Halcon::HTuple chan;
                Halcon::count_channels(*obj, &chan);
                int type = chan[0].I() > 1 ? RGB_IMAGE : GRAY_IMAGE;
                if(m_calibration.m_radialDistortionHandling)
                    Halcon::map_image(*obj, *m_calibration.m_radialDistMap, obj);
                while(m_images.size() > MAX_SIM_IMAGES)
                {
                    if(!DeleteReading())
                        break;
                }
                PushBack(new Image(obj, type ));
            }

        }
        else
        {
      if(m_images.size() == 0)
          return NULL;
      nFrame = m_FrameCount < (signed long)m_images.size() ? m_FrameCount : 0;
      return GetReading_Lock(nFrame - m_deletedOffset);
    }
        return GetReading_Lock(nFrame - m_deletedOffset);
    }
}

bool SimulatedCamera::CanSee  (RelPose &) const
{
    return true;
}



bool	SimulatedCamera::Start()
{
    return false;
}
bool	SimulatedCamera::Stop()
{
  Sensor::Stop();
  if(m_isVideo)
  {
     delete m_acqHandle;
  }
  return false;
}

void SimulatedCamera::SetFileName(std::string fileName)
{
    m_sourceFileName.push_back(fileName);
    size_t i = m_sourceFileName.size() - 1;
    if(!m_sourceIsActiv && m_sourceType == 0 && m_sourceFileName[i].length() > 0)
    {
        try
        {
            PushBack(ReadFromFile(m_sourceFileName[i]));
        }
        catch(char const* text)
        {
            printf("Error Reading images: %s\n", text);
        }
    }
}

XMLTag* SimulatedCamera::Save()
{
    XMLTag* tag = new XMLTag(GetName());
    if(m_isVideo && m_sourceFileName.size() > 1)
        tag->AddChild(XMLTag::Tag(m_sourceFileName[0], XML_NODE_IMAGE_FILE));
    else
    {
        for(unsigned int i = 0; i < m_sourceFileName.size(); i++)
        {
            tag->AddChild(XMLTag::Tag(m_sourceFileName[i], XML_NODE_IMAGE_FILE));
        }
    }
    Camera::SaveTo(tag);
    tag->AddProperty(XML_ATTRIBUTE_SOURCETYPE, m_sourceType);
    tag->AddProperty(XML_ATTRIBUTE_SOURCEISACTIVE, m_sourceIsActiv);
    return tag;
}
