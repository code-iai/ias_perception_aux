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
FindLines.cpp
**************************************************************************/

#include "ElemTypes.h"
#include "NamedDescriptor.h"
#include "AttentionAlgorithm.h"
#include "XMLTag.h"

#include <Camera.h>
#include <RelPoseHTuple.h>
#include <RegionOI.h>
#include <cpp/HalconCpp.h>
#include <SwissRangerReading.h>
#include <RangeSensor.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#define XML_NODE_FINDLINES "FindLines"

void find_lines (Halcon::Hobject ImageReducedL, Halcon::Hobject ImageReducedR, Halcon::HTuple CamParamL,
    Halcon::HTuple CamParamR, Halcon::HTuple Pose, Halcon::HTuple *XFinal1, Halcon::HTuple *YFinal1,
    Halcon::HTuple *ZFinal1, Halcon::HTuple *XFinal2, Halcon::HTuple *YFinal2, Halcon::HTuple *ZFinal2);

void find_lines_1image (Halcon::Hobject ImageReducedL, Halcon::HTuple CamParamL, Halcon::HTuple hommat,
    Halcon::HTuple &XL1, Halcon::HTuple &YL1, Halcon::HTuple &ZL1, Halcon::HTuple &XL2, Halcon::HTuple &YL2, Halcon::HTuple &ZL2);

namespace cop
{

   CREATE_NAMED_DESCRIPTOR_SHOW(Line3D, "Line3D", "Line", DESCRIPTOR_LINE3D, std_msgs::String)

   void Line3D::Show(RelPose* pose, Sensor* sens)
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
        Matrix cov = pose->GetCovarianceMatrix(ID_WORLD);

        ColumnVector d(4);
        ColumnVector f(4);
        for(int i=0; i < 8; i++)
        {
          d << 0.0 <<  0.0 << (0.25 *(4-i) * cov.element(2,2)) << 1.0;
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

   using namespace Halcon;
   HTuple CamPar(Sensor* sens)
   {
     HTuple CamPar(8,0.0);
     std::vector<double> vec = sens->GetUnformatedCalibrationValues().second;
     CamPar[0] = vec[0];
     CamPar[1] = 0.0;
     for(int i = 1; i < vec.size(); i++)
     {
       CamPar[i+1] = vec[i];
     }
     printf("CamParam: %f %f   %f %f   %f %f   %f %f\n",  CamPar[0].D(), CamPar[1].D(), CamPar[2].D(), CamPar[3].D(), CamPar[4].D(), CamPar[5].D(), CamPar[6].D(), CamPar[7].D());
     return CamPar;
   }

  inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
  {
      a_x = b_y*c_z - b_z*c_y;
      a_y = b_z*c_x - b_x*c_z;
      a_z = b_x*c_y - b_y*c_x;
  }

  inline void normalize(double &a,double &b, double &c)
  {
    double norm = sqrt(a*a + b*b + c*c);
    a /= norm;
    b /= norm;
    c /= norm;
  }

  RelPose* LineToPose(double x1, double y1, double z1, double x2, double y2, double z2, LocatedObjectID_t ref)
  {
    Matrix m(4,4);
    Matrix cov(6,6);
    RelPose* pos = NULL;
    double  dx = (x2 - x1), dy = (y2 - y1), dz = (z2 - z1);
    double normf = sqrt(dx*dx+dy*dy+dz*dz);
    double  ndx = dx / normf, ndy = dy / normf, ndz = dz / normf;

    m.element(0,3) = (x1 + x2) / 2.0;
    m.element(1,3) = (y1 + y2) / 2.0;
    m.element(2,3) = (z1 + z2) / 2.0;
    m.element(3,3) = 1.0;

    m.element(0,2) = ndx;
    m.element(1,2) = ndy;
    m.element(2,2) = ndz;
    double ya_x, ya_y, ya_z, xa_x, xa_y, xa_z;
    if(ndz != 1.0)
    {
      CrossProduct_l(ndx, ndy, ndz, 0,0,1, ya_x, ya_y, ya_z);
      normalize(ya_x, ya_y, ya_z);
      CrossProduct_l(ya_x, ya_y, ya_z, ndx, ndy, ndz, xa_x, xa_y, xa_z);
      normalize(xa_x, xa_y, xa_z);
    }
    else
    {
      CrossProduct_l(ndx, ndy, ndz, 1,0,0, ya_x, ya_y, ya_z);
      normalize(ya_x, ya_y, ya_z);
      CrossProduct_l(ya_x, ya_y, ya_z, ndx, ndy, ndz, xa_x, xa_y, xa_z);
      normalize(xa_x, xa_y, xa_z);
    }
    m.element(0,0) = xa_x;
    m.element(1,0) = xa_y;
    m.element(2,0) = xa_z;
    m.element(0,1) = ya_x;
    m.element(1,1) = ya_y;
    m.element(2,1) = ya_z;
    m.element(3,0) = 0.0;
    m.element(3,1) = 0.0;
    m.element(3,2) = 0.0;

    for(int i = 0; i < 6; i++)
      for(int j = 0; j < 6; j++)
      {
        cov.element(i,j) = 0.0;
      }
    cov.element(2,2) = normf / 2;
    pos = RelPoseFactory::FRelPose(ref, m, cov);
    pos->m_qualityMeasure = normf;
    return pos;
  }

   class  FindLines : public LocateAlgorithm
   {
     public:
       FindLines(){}
       ~FindLines(){}
      virtual std::string GetName(){return XML_NODE_FINDLINES;}

      double CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
      {
        printf("FindLines::CheckSignature: ");
          if(sig.GetElement(0,DESCRIPTOR_LINE3D) != NULL && sensors.size() > 1)
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
            if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
            {
              try
              {
                reading = (SwissRangerReading*)((*it)->GetReading());
                break;
              }
              catch(...)
              {
                 printf("Error getting pcd\n");
                return results;
              }
            }
          }
          if(reading == NULL)
          {
             printf("Error getting pcd\n");
             return results;
          }
          sensor_msgs::PointCloud& pcd_in = reading->m_image;
          Matrix mean = IdentityMatrix(4);
          double maxx = -1000.0, minx = 1000.0, maxy = -1000.0, miny = 1000.0, maxz = -1000.0, minz = 1000.0;

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
                  if(pcd_in.points[j].x > maxx)
                    maxx = pcd_in.points[j].x;
                  if(pcd_in.points[j].x < minx)
                    minx = pcd_in.points[j].x;
                  if(pcd_in.points[j].y > maxy)
                    maxy = pcd_in.points[j].y;
                  if(pcd_in.points[j].y < miny)
                    miny = pcd_in.points[j].y;
                  if(pcd_in.points[j].z > maxz)
                    maxz = pcd_in.points[j].z;
                  if(pcd_in.points[j].z < minz)
                    minz = pcd_in.points[j].z;
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

          RelPose* poseFinal = RelPoseFactory::FRelPose(reading->GetPose()->m_uniqueID, mnew, cov2);
          Matrix test = poseFinal->GetMatrix(img.sensor_pose_at_capture_time->m_uniqueID);

          ColumnVector extremVec(4);


          HTuple CamParam1, area_roi1;
          HTuple X1, X2, Y1, Y2, Z1, Z2, hommat;

          hommat[0] = test.element(0,0);
          hommat[1] = test.element(0,1);
          hommat[2] = test.element(0,2);
          hommat[3] = test.element(0,3);
          hommat[4] = test.element(1,0);
          hommat[5] = test.element(1,1);
          hommat[6] = test.element(1,2);
          hommat[7] = test.element(1,3);
          hommat[8] = test.element(2,0);
          hommat[9] = test.element(2,1);
          hommat[10] = test.element(2,2);
          hommat[11] = test.element(2,3);


          CamParam1 = CamPar(img.selected_sensor);

          Hobject roi_obj1,  ImageReduced1;
          if(pose != NULL)
          {
            RegionOI roi1(pose, img.sensor_pose_at_capture_time->m_uniqueID, CamParam1);
            try
            {
              HTuple rowtmp, coltmp;
              dilation_circle(roi1.GetRegion(), &roi_obj1, 20);
              area_center(roi_obj1, &area_roi1, &rowtmp, &coltmp );
            }
            catch(HException ex)
            {
              printf("FindLines::Perform: Error creating region: %s\n", ex.message);
              area_roi1 = 0;
            }
          }
          if(area_roi1[0].I() > 0)
            reduce_domain(**img, roi_obj1, &ImageReduced1);

          write_image(ImageReduced1, "tiff", 0, "FindLineDebugReduced.tif");
          find_lines_1image (ImageReduced1, CamParam1, hommat, X1, Y1, Z1, X2, Y2, Z2);
          printf("Got %ld Line Candidates in 3D\n", X1.Num());
          for(int i = 0; i < X1.Num(); i++)
          {

            ColumnVector d(4),e(4), f(4), g(4),h(4);

            d << X1[i].D()<<Y1[i].D()<< Z1[i].D() << 1.0;
            f = test * d;
            e << X2[i].D()<< Y2[i].D()<< Z2[i].D() << 1.0;
            g = test * e;

            printf("Line with %f length\n", sqrt((f.element(0) - g.element(0))*(f.element(0) - g.element(0)) +
                     (f.element(1) - g.element(1))*(f.element(1) - g.element(1)) +
                     (f.element(2) - g.element(2))*(f.element(2) - g.element(2))));

            if( sqrt((f.element(0) - g.element(0))*(f.element(0) - g.element(0)) +
                     (f.element(1) - g.element(1))*(f.element(1) - g.element(1)) +
                     (f.element(2) - g.element(2))*(f.element(2) - g.element(2))) > 0.042)
            {

              RelPose* line = LineToPose(f.element(0),f.element(1), f.element(2) , g.element(0), g.element(1), g.element(2), img.sensor_pose_at_capture_time->m_uniqueID);


              Matrix m_rgb = pose->GetMatrix( img.sensor_pose_at_capture_time->m_uniqueID);
              Matrix cov_rgb = pose->GetCovarianceMatrix( img.sensor_pose_at_capture_time->m_uniqueID);
              if(line != NULL)
              {

                Matrix m_line  = line->GetMatrix(img.sensor_pose_at_capture_time->m_uniqueID);
                if( m_rgb.element(0, 3) - 2*cov_rgb.element(0,0) < m_line.element(0,3) && m_line.element(0,3) < m_rgb.element(0, 3) + 2*cov_rgb.element(0,0) &&
                    m_rgb.element(1, 3) - 2*cov_rgb.element(1,1) < m_line.element(1,3) && m_line.element(1,3) < m_rgb.element(1, 3) + 2*cov_rgb.element(1,1) &&
                    m_rgb.element(2, 3) - 2*cov_rgb.element(2,2) < m_line.element(2,3) && m_line.element(2,3) < m_rgb.element(2, 3) + 2*cov_rgb.element(2,2))
                {
                  results.push_back(line);
                }
                else
                  RelPoseFactory::FreeRelPose(&line);
              }
            }
          }
          RelPoseFactory::FreeRelPose(&poseFinal);

        }
        catch(HException ex)
        {
           printf("Error in FindLine::Perform: HProblems: %s\n", ex.message);

        }
        catch(const char* exception)
        {
           printf("Error in FindLine::Perform: Problems: %s\n", exception);
        }
        numOfObjects = results.size();
        return results;
      }
   };
}

using namespace cop;

PLUGINLIB_REGISTER_CLASS(Line3D, cop::Line3D, Descriptor);
PLUGINLIB_REGISTER_CLASS(FindLines, cop::FindLines, LocateAlgorithm);


