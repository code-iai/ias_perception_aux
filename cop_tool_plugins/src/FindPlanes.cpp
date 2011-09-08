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
FindPlanes.cpp
**************************************************************************/

#include "ElemTypes.h"
#include "NamedDescriptor.h"
#include "AttentionAlgorithm.h"
#include "XMLTag.h"

#include <Camera.h>
#include <cpp/HalconCpp.h>
#include <SwissRangerReading.h>
#include <RangeSensor.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#define XML_NODE_FINDPLANES "FindPlanes"

void find_max_concav_diff (Halcon::Hobject Image3, Halcon::HTuple *maxdiff, Halcon::HTuple *concavity);

namespace cop
{


    inline void normalize(double &a,double &b, double &c)
    {
      double norm = sqrt(a*a + b*b + c*c);
      a /= norm;
      b /= norm;
      c /= norm;
    }

    inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
    {
      return a * d + b* e + c*f;
    }

    inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
    {
        a_x = b_y*c_z - b_z*c_y;
        a_y = b_z*c_x - b_x*c_z;
        a_z = b_x*c_y - b_y*c_x;
    }


    inline double ConcavityTest(Matrix *cov2, Matrix plane_inv, Matrix m, Matrix cov, const sensor_msgs::PointCloud &pcd_in)
    {
      using namespace Halcon;
      double result = 0.0, maxx = -10000.0, minx = 10000.0, maxy = -10000.0, miny = 10000.0, maxz = -10000.0, minz = 10000.0;
      ColumnVector d(4),e(4);
      Hobject img, img_height, img_div, tmp, reg;
      HTuple image_ptr, type, w, h;
      float *ptr, *ptr_height;
      double scalex, scaley;
      int row , col, size = 20, count = 0;
      printf("debug1\n");
      gen_image_const(&img, "float", size,size);
      gen_image_const(&img_height, "float", size, size);

      get_image_pointer1(img, &image_ptr, &type, &w, &h);
      ptr = (float*)image_ptr[0].L();
      get_image_pointer1(img_height, &image_ptr, &type, &w, &h);
      ptr_height = (float*)image_ptr[0].L();

      printf("debug2\n");
      for(size_t j = 0; j < pcd_in.points.size(); j++)
      {
         if( m.element(0, 3) - cov.element(0,0) < pcd_in.points[j].x && pcd_in.points[j].x < m.element(0, 3) + cov.element(0,0) &&
          m.element(1, 3) - cov.element(1,1) < pcd_in.points[j].y && pcd_in.points[j].y < m.element(1, 3) + cov.element(1,1) &&
          m.element(2, 3) - cov.element(2,2) < pcd_in.points[j].z && pcd_in.points[j].z < m.element(2, 3) + cov.element(2,2))
          {
            d << (double)pcd_in.points[j].x << (double)pcd_in.points[j].y << (double)pcd_in.points[j].z << 1.0;
            e = plane_inv * d;

            if(e.element(0) > maxx)
              maxx = e.element(0);
            if(e.element(0) < minx)
              minx = e.element(0);
            if(e.element(1) > maxy)
              maxy = e.element(1);
            if(e.element(1) < miny)
              miny = e.element(1);
            if(e.element(2) > maxz)
              maxz = e.element(2);
            if(e.element(2) < minz)
              minz = e.element(2);

            for(int k = 0; k < 3; k++)
              for(int l = 0; l < 3; l++)
                cov2->element(k,l) += e.element(k)*e.element(l);
            count++;
          }
      }

      scalex = maxx - minx;
      scaley = maxy - miny;
      double scalez = maxz - minz;
      for(size_t j = 0; j < pcd_in.points.size(); j++)
      {
         if( m.element(0, 3) - cov.element(0,0) < pcd_in.points[j].x && pcd_in.points[j].x < m.element(0, 3) + cov.element(0,0) &&
          m.element(1, 3) - cov.element(1,1) < pcd_in.points[j].y && pcd_in.points[j].y < m.element(1, 3) + cov.element(1,1) &&
          m.element(2, 3) - cov.element(2,2) < pcd_in.points[j].z && pcd_in.points[j].z < m.element(2, 3) + cov.element(2,2))
          {
            d << (double)pcd_in.points[j].x << (double)pcd_in.points[j].y << (double)pcd_in.points[j].z << 1.0;
            e = plane_inv * d;
            col =  size * ((e.element(0) - minx) / scalex);
            row =  size * ((e.element(1) - miny) / scaley);
            if(row >= size || col >= size || row < 0 || col < 0)
            {
               printf("Wrong index ... Concavity\n");
            }
            else
            {
              ptr[row * size + col] += 1.0f;
              ptr_height[row * size + col] += e.element(2);
            }
          }
      }

      threshold(img, &reg, 0.99, 100000.0);
      reduce_domain(img, reg, &tmp);

      div_image(img_height, tmp,  &img_div, 1.0, 0.0);

      write_image(img_height, "tiff", 0, "Concavity_image1.tiff");
      write_image(tmp, "tiff", 0, "Concavity_image2.tiff");
      write_image(img_div, "tiff", 0, "Concavity_image3.tiff");
      HTuple max_diff,concavity;

      find_max_concav_diff(img_div, &max_diff, &concavity);

      printf("Max diff between middle and outer: %f, concavity is considered: %d \n", max_diff[0].D(), concavity[0].I());
      if(max_diff[0].D() > 0.005 && concavity[0].I() == 1)
        result = 1.0;
      else
        result = 0.0;
      for(int k = 0; k < 3; k++)
        for(int l = 0; l < 3; l++)
          cov2->element(k,l) = sqrt(cov2->element(k,l) / count);

      return result;
    }

   CREATE_NAMED_DESCRIPTOR_SHOW(Plane3D, "Plane3D", "Plane", DESCRIPTOR_PLANE, std_msgs::String)
   CREATE_NAMED_DESCRIPTOR_SHOW(Concavity, "Concavity", "Concavity", DESCRIPTOR_PLANE, std_msgs::String)

   void Plane3D::Show(RelPose* pose, Sensor* sens)
   {
      using namespace Halcon;
      std::vector<double> x,y,z;
      if(sens != NULL && sens->IsCamera())
      {
        Camera* cam = (Camera*)sens;
        HWindow* hwin = cam->GetWindow();
        HTuple CamParam = cam->m_calibration.CamParam(), row, col;
        Matrix m = pose->GetMatrix(sens->GetRelPose()->m_uniqueID);
        Matrix w = pose->GetMatrix(ID_WORLD);

        ColumnVector d(4);
        ColumnVector f(4);
        for(int i=0; i < 8; i++)
        {
          switch(i)
          {
            case 0:
             d << 0.05 << 0.0 << 0.0 << 1;
             break;
            case 1:
             d << -0.05 << 0.0 << 0.0 << 1;
             break;
            case 2:
             d << 0.0 << 0.05  << 0.0 << 1;
             break;
            case 3:
             d << 0.0 << -0.05  << 0.0 << 1;
             break;
            case 4:
             d << 0.05 << 0.05 << 0.0 << 1;
             break;
            case 5:
             d << 0.05 << -0.05 << 0.0 << 1;
             break;
            case 6:
             d << -0.05 << 0.05 << 0.0 << 1;
             break;
            case 7:
             d << -0.05 << -0.05 << 0.0 << 1;
             break;
          }
          f = m * d;
          project_3d_point(f.element(0), f.element(1), f.element(2), CamParam, &row, &col);
          f = w * d;
          x.push_back(f.element(0));
          y.push_back(f.element(1));
          z.push_back(f.element(2));
          disp_cross(hwin->WindowHandle(), row, col, 6, 0.79);
        }

        sens->Publish3DData(x,y,z);
      }
   }
   void Concavity::Show(RelPose* pose, Sensor* sens)
   {
      using namespace Halcon;
      std::vector<double> x,y,z;
      if(sens != NULL && sens->IsCamera())
      {
        Camera* cam = (Camera*)sens;
        HWindow* hwin = cam->GetWindow();
        HTuple CamParam = cam->m_calibration.CamParam(), row, col;
        Matrix m = pose->GetMatrix(sens->GetRelPose()->m_uniqueID);
        Matrix w = pose->GetMatrix(ID_WORLD);

        ColumnVector d(4);
        ColumnVector f(4);
        for(int i=0; i < 8; i++)
        {
          switch(i)
          {
            case 0:
             d << 0.05 << 0.0 << 0.0 << 1;
             break;
            case 1:
             d << -0.05 << 0.0 << 0.0 << 1;
             break;
            case 2:
             d << 0.0 << 0.05  << 0.0 << 1;
             break;
            case 3:
             d << 0.0 << -0.05  << 0.0 << 1;
             break;
            case 4:
             d << 0.05 << 0.05 << 0.0 << 1;
             break;
            case 5:
             d << 0.05 << -0.05 << 0.0 << 1;
             break;
            case 6:
             d << -0.05 << 0.05 << 0.0 << 1;
             break;
            case 7:
             d << -0.05 << -0.05 << 0.0 << 1;
             break;
          }
          f = m * d;
          project_3d_point(f.element(0), f.element(1), f.element(2), CamParam, &row, &col);
          f = w * d;
          x.push_back(f.element(0));
          y.push_back(f.element(1));
          z.push_back(f.element(2));
          disp_cross(hwin->WindowHandle(), row, col, 6, 0.79);
        }

        sens->Publish3DData(x,y,z);
      }
   }

   class  FindPlanes : public LocateAlgorithm
   {
     public:
       FindPlanes(){}
       ~FindPlanes(){}
      virtual std::string GetName(){return XML_NODE_FINDPLANES;}

      double CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
      {
        printf("FindPlanes::CheckSignature: ");
          if(sig.GetElement(0,DESCRIPTOR_PLANE) != NULL && sensors.size() > 0)
          {
            printf("Accepted \n");
            return 1.0;
          }
          else
          {
            printf("Declined\n");
            return 0.0;
          }
      }

      virtual XMLTag* Save()
      {
        return new XMLTag(GetName());
      }

      virtual void SetData(XMLTag* tag)
      {}

      std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& prototype, int &numOfObjects, double& qualityMeasure)
      {
        std::vector<RelPose*> results;
        std::vector<Sensor*> sensors2;
        try
        {
          using namespace Halcon;
          ScopedImage<Image, Hobject*> img(sensors, ReadingType_HalconImage);
          SwissRangerReading* reading = NULL;
          for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
          {
            printf("%s\n", (*it)->GetName().c_str());
            if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
            {
              try
              {
                reading = (SwissRangerReading*)((*it)->GetReading());
                break;
              }
              catch(const char* text)
              {
                printf("Error getting pcd (Exception %s)\n", text);
              }
              catch(...)
              {
                 printf("Error getting pcd (Exception)\n");
                return results;
              }
            }
          }
          if(reading == NULL)
          {
             printf("Error getting pcd (no reading -> NULL)\n");
             return results;
          }
          sensor_msgs::PointCloud& pcd_in = reading->m_image;
          Matrix mean = IdentityMatrix(4);

          Matrix m = pose->GetMatrix(reading->GetPose()->m_uniqueID);
          Matrix cov = pose->GetCovarianceMatrix(reading->GetPose()->m_uniqueID);

          int count = 0;
          for(size_t j = 0; j < pcd_in.points.size(); j++)
          {
            if( m.element(0, 3) - cov.element(0,0) < pcd_in.points[j].x && pcd_in.points[j].x < m.element(0, 3) + cov.element(0,0) &&
                m.element(1, 3) - cov.element(1,1) < pcd_in.points[j].y && pcd_in.points[j].y < m.element(1, 3) + cov.element(1,1) &&
                m.element(2, 3) - cov.element(2,2) < pcd_in.points[j].z && pcd_in.points[j].z < m.element(2, 3) + cov.element(2,2))
                {
                  mean.element(0,3) += pcd_in.points[j].x;
                  mean.element(1,3) += pcd_in.points[j].y;
                  mean.element(2,3) += pcd_in.points[j].z;

                  count++;
                }
          }
          printf("count %d \n", count);
          mean.element(0,3) /= count;
          mean.element(1,3) /= count;
          mean.element(2,3) /= count;
          Matrix msv(count, 3);
          count = 0;
          for(size_t j = 0; j < pcd_in.points.size(); j++)
          {
            if( m.element(0, 3) - cov.element(0,0) < pcd_in.points[j].x && pcd_in.points[j].x < m.element(0, 3) + cov.element(0,0) &&
                m.element(1, 3) - cov.element(1,1) < pcd_in.points[j].y && pcd_in.points[j].y < m.element(1, 3) + cov.element(1,1) &&
                m.element(2, 3) - cov.element(2,2) < pcd_in.points[j].z && pcd_in.points[j].z < m.element(2, 3) + cov.element(2,2))
                {
                  msv.element(count, 0) = pcd_in.points[j].x - mean.element(0, 3);
                  msv.element(count, 1) = pcd_in.points[j].y - mean.element(1, 3);
                  msv.element(count, 2) = pcd_in.points[j].z - mean.element(2, 3);

                  count++;
                }
          }
          DiagonalMatrix D;
          Matrix U, V;
          try
          {
            printf("Call SVD\n");
            SVD(msv, D, U, V);
          }
          catch(BaseException ex)
          {
            printf("Error in newmat: %s\n", ex.what());
          }
          Matrix mnew (4,4);
          Matrix cov2(6,6);
          for(int r = 0; r < 6; r++) for(int c = 0; c < 6; c++) cov2.element(r,c) = 0.0;

          cout << "V" << endl << V << endl;

          CrossProduct_l(V.element(0, 0), V.element(1, 0), V.element(2, 0),
                         V.element(0, 1),  V.element(1, 1),  V.element(2, 1),
                         mnew.element(0, 2), mnew.element(1, 2),mnew.element(2, 2));
          normalize(mnew.element(0, 2), mnew.element(1, 2),mnew.element(2, 2));
          if(mnew.element(0,2) < 0.0)
          {
            mnew.element(0, 2) *= -1.0;
            mnew.element(1, 2) *= -1.0;
            mnew.element(2, 2) *= -1.0;
          }
          mnew.element(3, 0) = 0.0;
          mnew.element(3, 1) = 0.0;
          mnew.element(3, 2) = 0.0;
          mnew.element(3, 3) = 1.0;


          CrossProduct_l(mnew.element(0, 2), mnew.element(1, 2), mnew.element(2, 2),
                       mean.element(0, 2),  mean.element(1, 2),  mean.element(2, 2),
                       mnew.element(0, 1), mnew.element(1, 1),mnew.element(2, 1));
          normalize(mnew.element(0, 1), mnew.element(1, 1),mnew.element(2, 1));
          CrossProduct_l(mnew.element(0, 2), mnew.element(1, 2), mnew.element(2, 2),
                        mnew.element(0, 1), mnew.element(1, 1),mnew.element(2, 1),
                        mnew.element(0, 0), mnew.element(1, 0),mnew.element(2, 0));
          if(reading->GetPose()->m_uniqueID != pose->m_uniqueID)
          {
            Matrix position = pose->GetMatrix(reading->GetPose()->m_uniqueID);
            mnew.element(0,3) = mean.element(0,3);
            mnew.element(1,3) = mean.element(1,3);
            mnew.element(2,3) = mean.element(2,3);
          }
          else
          {
            mnew.element(0,3) = 0.0;
            mnew.element(1,3) = 0.0;
            mnew.element(2,3) = 0.0;
            mean.element(0,3) = 0.0;
            mean.element(1,3) = 0.0;
            mean.element(2,3) = 0.0;
          }
          mnew.element(0, 0) *= -1;
          mnew.element(1, 0) *= -1;
          mnew.element(2, 0) *= -1;

          normalize(mnew.element(0, 0), mnew.element(1, 0),mnew.element(2, 0));
          cout << "New Matrix:" << endl << mnew << endl;

          Matrix mnew_inv = mnew.i();
          double concavity = ConcavityTest(&cov2, mnew_inv, m, cov, pcd_in);
          printf("concavity: %f,  (%s)\n", concavity, prototype.GetElement(0,DESCRIPTOR_PLANE)->GetNodeName().c_str());
          if((concavity < 0.5 && prototype.GetElement(0,DESCRIPTOR_PLANE)->GetNodeName().compare("Plane3D") == 0) ||
             (concavity > 0.5 && prototype.GetElement(0,DESCRIPTOR_PLANE)->GetNodeName().compare("Concavity") == 0))
          {
            RelPose* poseFinal = RelPoseFactory::FRelPose(reading->GetPose()->m_uniqueID, mnew, cov2);
            Matrix test = poseFinal->GetMatrix(img.sensor_pose_at_capture_time->m_uniqueID);
            qualityMeasure = poseFinal->m_qualityMeasure = pcd_in.points.size() / count;
            results.push_back(poseFinal);
          }

        }
        catch(const char* exception)
        {
           printf("Error in FindPlane::Perform: Problems: %s\n", exception);
        }
        numOfObjects = results.size();
        return results;
      }
   };
}

using namespace cop;

PLUGINLIB_REGISTER_CLASS(Plane3D, cop::Plane3D, Descriptor);
PLUGINLIB_REGISTER_CLASS(Concavity, cop::Concavity, Descriptor);
PLUGINLIB_REGISTER_CLASS(FindPlanes, cop::FindPlanes, LocateAlgorithm);

