/******
 * Copyright (c) 2010, Ulrich Klank
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/************************************************************************
TableObjectDescriptors.cpp
**************************************************************************/

#include "ElemTypes.h"
#include "NamedDescriptor.h"
#include "AttentionAlgorithm.h"
#include "XMLTag.h"

#include <Camera.h>
#include <cpp/HalconCpp.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>




#define XML_NODE_FINDBARCODE "FindBarCode"


namespace cop
{

RelPose* distance_to_3d(RelPose *sensor_pose, MinimalCalibration &calib, double length_min,
                       double length_max, double x, double y, double cov_width, double cov_height)
{
  printf("distance_to_3d with lmin %f  lmax %f x %f y %f co_w %f co_h%f\n ",
         length_min, length_max, x, y, cov_width, cov_height);
  Matrix m(4,4), cov(6,6);
  double X, Y, Z;
  X = (x - calib.proj_center_x)*calib.pix_size_x;
  Y = (y - calib.proj_center_y)*calib.pix_size_y;
  Z = calib.focal_length;
  double norm = sqrt( X*X + Y*Y + Z*Z);
  double xnorm = X/norm;
  double ynorm = Y/norm;
  double znorm = Z/norm;
  X *= (length_max + length_min)/(2*norm);
  Y *= (length_max + length_min)/(2*norm);
  Z *= (length_max + length_min)/(2*norm);
  /* TODO rotate the matrix*/   
  m << 1.0 << 0.0 << 0.0 << X   
    << 0.0 << 1.0 << 0.0 << Y   
    << 0.0 << 0.0 << 1.0 << Z   
    << 0.0 << 0.0 << 0.0 << 1.0;
                                                       
  cov << cov_width / 2 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << cov_height / 2<< 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << (length_max - length_min)/2 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
  return RelPoseFactory::FRelPose(sensor_pose, m, cov);
}
                                                                                         

   CREATE_NAMED_DESCRIPTOR(BarCode, "BarCode", "NotRead", DESCRIPTOR_BARCODE, std_msgs::String)

   class  FindBarCode : public AttentionAlgorithm
   {
     public:
       FindBarCode(){
         lastCall = (unsigned long)time(NULL);
       }

       ~FindBarCode(){}
      virtual std::string GetName(){return XML_NODE_FINDBARCODE;}

      double CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
      {
          if(sig.GetElement(0,DESCRIPTOR_BARCODE) != NULL)
          {
           printf("FindBarCode::CheckSignature\n");
           return 1.0;
          }
          else
            return 0.0;

      }

      virtual XMLTag* Save()
      {
        return new XMLTag(GetName());
      }
      virtual void SetData(XMLTag* tag)
      {}
      std::vector<Signature*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& prototype, int &numOfObjects, double& qualityMeasure)
      {
        std::vector<Signature*> results;
        unsigned long now = (unsigned long)time(NULL);
        if(now - lastCall  > 10000)
        {
          printf("\n\n\nClear list\n\n\n");
          m_lastDecoding.clear();
        }
        else
        {
          if(sensors.size() > 0)
          {
            sensors[0]->WaitForNewData();
          }
        }
        lastCall = now;
        try
        {
          using namespace Halcon;
          ScopedImage<Image, Hobject*> img(sensors, ReadingType_HalconImage);
          
          Hobject outregions;
          HTuple  DecryptedString, genparam, genvalue, handle;
          try
          {
            create_bar_code_model(genparam, genvalue, &handle);
            find_bar_code(*(*img), &outregions, handle, "EAN-13", &DecryptedString);
            if(DecryptedString.Num() == 0)
            {
              find_bar_code(*(*img), &outregions, handle, "EAN-8", &DecryptedString);
            }

          }
          catch(HException ex)
          {
             printf("Error in FindBarCode::Perform: %s\n", ex.message);
          }
          Halcon::clear_bar_code_model(handle);
          for(int i = 0 ; i < DecryptedString.Num(); i++)
          {
            double lastQuality = 0.0;
            if(m_lastDecoding.find(DecryptedString[i].S()) != m_lastDecoding.end())
            {
              lastQuality = m_lastDecoding[std::string(DecryptedString[i].S())];
              if(lastQuality >= 1.0)
              {
                
                break;
              }

               m_lastDecoding[std::string(DecryptedString[i].S())] = lastQuality += 0.1;
            }
            else
            {
              printf("\n\nFound new Barcode\n\n");
              lastQuality = m_lastDecoding[std::string(DecryptedString[i].S())] = 0.1;
            }


            Signature* obj = new Signature();
            BarCode* barcode = new BarCode();
            barcode->SetClass(DecryptedString[i].S());
            Hobject temp;
            HTuple row, col, area;
            select_obj(outregions, &temp, i + 1);
            area_center(temp, &area, &row, &col);
            obj->SetElem(barcode);
            RelPose* barcodepose = distance_to_3d(pose, img.calib, 0.05, 
                       0.10, col[0].D(), row[0].D(), 0.02, 0.02);
                       
            barcodepose->m_qualityMeasure = lastQuality;
            obj->SetPose(barcodepose);
            results.push_back(obj);
            printf("Returning object with id %ld\n", obj->m_ID);
          }
        }
        catch(const char* exception)
        {
           printf("Error in FindBarCode::Perform: Problems: %s\n", exception);
        }
        numOfObjects = results.size();
        return results;
      }
    private:
      std::map<std::string, double> m_lastDecoding;
      unsigned long lastCall;
   };
}

using namespace cop;

PLUGINLIB_REGISTER_CLASS(BarCode, cop::BarCode, Descriptor);
PLUGINLIB_REGISTER_CLASS(FindBarCode, cop::FindBarCode, AttentionAlgorithm);

