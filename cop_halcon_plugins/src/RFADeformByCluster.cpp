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

#include "RFADeformByCluster.h"
#include "Camera.h"
#include "RegionOI.h"
#include <algorithm>
#include "DeformShapeModel.h"
#include "SegmentPrototype.h"

using namespace cop;

RFADeformByCluster::RFADeformByCluster(void)
{
}

RFADeformByCluster::RFADeformByCluster(XMLTag* tag)
{
}


RFADeformByCluster::~RFADeformByCluster(void)
{
}

inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}

Descriptor* RFADeformByCluster::Perform(std::vector<Sensor*> cam_vec, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  DeformShapeModel* deformshape  = NULL;
  Camera *cam = NULL;
  qualityMeasure = 0.0;

  for(size_t i = 0; i < cam_vec.size(); i++)
  {
    if(cam_vec[i] != NULL && cam_vec[i]->IsCamera())
    {
      Matrix planarCompensate = IdentityMatrix(4); planarCompensate.element(0,0) = -1.0; planarCompensate.element(2,2) = -1.0;
      printf("Creating Model for Sensor %s\n", cam_vec[i]->GetSensorID().c_str());
      cam = (Camera*)cam_vec[i];
      try
      {
        if(deformshape == NULL)
        {
          Class *cl = new Class();
          std::stringstream st;
          st << "Texture_" << cl->m_ID;
          cl->SetName(st.str());
          deformshape = new DeformShapeModel(cl);
        }
        Image* img = (Image*)cam->GetReading(-1);
        RelPose* readingpose = img->GetPose();
        Halcon::Hobject* region_h, saver;
        RegionOI region(pose, readingpose->m_uniqueID, &(cam->m_calibration));
        region_h =  &(region.m_reg);

        SegmentPrototype* proto = (SegmentPrototype*)sig.GetElement(0, DESCRIPTOR_SEGMPROTO);
        RelPose* poseFinal = pose;
        /** This means we can calculate a good approximated
            normal which will make the model better*/
        if(proto != NULL && proto->GetLastMatchedPose() != NULL)
        {
          printf("We have a point cloud to derive the best plane hypothesis (%ld != %ld ?)\n", proto->GetLastMatchedPose()->m_uniqueID, pose->m_uniqueID);
          RegionOI regionProto(proto, readingpose->m_uniqueID, &(cam->m_calibration));
          Halcon::copy_obj(regionProto.m_reg, &saver, 1, 1);
          region_h =  &saver;


          sensor_msgs::PointCloud pcd_in = proto->GetPointCloud(proto->GetLastMatchedPose()->m_uniqueID);
          Matrix mean = IdentityMatrix(4);
          Matrix m(pcd_in.points.size(), 3);
          for(size_t j = 0; j < pcd_in.points.size(); j++)
          {
            m.element(j, 0) = pcd_in.points[j].x - mean.element(0, 3);
            m.element(j, 1) = pcd_in.points[j].y - mean.element(1, 3);
            m.element(j, 2) = pcd_in.points[j].z - mean.element(2, 3);
          }
          DiagonalMatrix D;
          Matrix U, V;
          try
          {
            printf("Call SVD\n");
            SVD(m, D, U, V);
          }
          catch(BaseException ex)
          {
            printf("Error in newmat: %s\n", ex.what());
          }
          Matrix mnew (4,4);
          Matrix cov(6,6);
          for(int r = 0; r < 6; r++) for(int c = 0; c < 6; c++) cov.element(r,c) = 0.0;

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
          if(mnew.element(0,2) < 0.5)
          {
            /** we do not like orthogonal view */
            printf("Correct a strange of assumed plane by flipping it...\n");
            CrossProduct_l(V.element(0, 0), V.element(1, 0), V.element(2, 0),
                                mnew.element(0, 2), mnew.element(1, 2),mnew.element(2, 2),
                                mnew.element(0, 2), mnew.element(1, 2),mnew.element(2, 2));
            normalize(mnew.element(0, 2), mnew.element(1, 2),mnew.element(2, 2));
            if(mnew.element(0,2) < 0.0)
            {
              mnew.element(0, 2) *= -1.0;
              mnew.element(1, 2) *= -1.0;
              mnew.element(2, 2) *= -1.0;
            }
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
          if(proto->GetLastMatchedPose()->m_uniqueID != pose->m_uniqueID)
          {
            Matrix position = pose->GetMatrix(proto->GetLastMatchedPose()->m_uniqueID);
            mnew.element(0,3) = position.element(0,3);
            mnew.element(1,3) = position.element(1,3);
            mnew.element(2,3) = position.element(2,3);
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
          cout << "Old Matrix:" << endl << mean << endl;
          cout << "Combined: " << endl << mean.i() * mnew << endl;
          Matrix t = mean.i() * mnew;
          planarCompensate = t.i();
          poseFinal = RelPoseFactory::FRelPose(proto->GetLastMatchedPose()->m_uniqueID, t, cov);
          Matrix test = poseFinal->GetMatrix(readingpose->m_uniqueID);

          double testd = test.element(2,2);
          cout << "orientation in image:" << endl<< test << endl;
          if(testd < 0.4) /** stuff looks away or is not frontal*/
          {
            printf("Ignore plane assumption and take the camera orientation\n");
            RelPoseFactory::FreeRelPose(&poseFinal);
            t = readingpose->GetMatrix(proto->GetLastMatchedPose()->m_uniqueID);
            
            poseFinal = RelPoseFactory::FRelPose(proto->GetLastMatchedPose()->m_uniqueID, t, cov);
          }
        }


        try
        {
          double temp = deformshape->DefineDeformShapeModel(img, region_h, cam, poseFinal, planarCompensate);
          if(proto != NULL && proto->GetLastMatchedPose() != NULL)
            deformshape->m_covInherit = proto->GetLastMatchedPose()->GetCovarianceMatrix(0);
          else
            deformshape->m_covInherit = pose->GetCovarianceMatrix(0);

          if(poseFinal != pose)
            RelPoseFactory::FreeRelPose(&poseFinal);
          if(temp > qualityMeasure)
          {
            qualityMeasure = temp;
            deformshape->Evaluate(qualityMeasure, 100.0);
          }

          if(temp == 0.0)
          {
            delete deformshape;
            deformshape = NULL;
          }
        }
        catch(Halcon::HException ex)
        {
          printf("Learning of Descriptorbased model Failed: %s\n", ex.message);
          delete deformshape;
          deformshape = NULL;
        }
        catch(char const* ex)
        {
          printf("Learning of Descriptorbased model Failed: %s\n", ex);
          delete deformshape;
          deformshape = NULL;
        }
        img->Free();
       }
       catch(char const* ex)
       {
         printf("Learning of Descriptorbased model Failed: %s\n", ex);
         if(deformshape != NULL)
         {
           delete deformshape;
         }
         deformshape = NULL;
       }
       printf("Finished creating for Sensor %s\n", cam_vec[i]->GetSensorID().c_str());
    }
  }
  return deformshape;
}

double RFADeformByCluster::CheckSignature(const Signature& sig, const std::vector<Sensor*> &sensors)
{

	if(sig.GetElement(0, DESCRIPTOR_DEFORMSHAPE) == NULL)
	{
			return 1.0;
	}
	return 0.0;
}

XMLTag* RFADeformByCluster::Save()
{
	XMLTag* tag = new XMLTag(GetName());

	//TODO: parameter?
	return tag;
}
