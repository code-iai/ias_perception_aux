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
#ifdef DESCRIPTOR_AVAILABLE


#include "PointDescrModel.h"


#include <cpp/HalconCpp.h>
#include <HCPPdescriptor3d.h>

//#include "HCPPplanar_pose_estimation.h" /*< Removed cause code is very closed source*/


#ifndef DXFLIB
#include "DxfReader.h"
#endif

#include "ShapeModel.h"
#define XML_NODE_POINTDESCRCONFIG "DescrConfig"
#define XML_ATTRIBUTE_POINTDESCRFILE "DescrFileName"

PointDescrModel::PointDescrModel(XMLTag* tag) :
    Descriptor(tag),
    m_DescriptoHandel(-1),
    m_Config(tag->GetChild(XML_NODE_POINTDESCRCONFIG) != NULL ? tag->GetChild(XML_NODE_POINTDESCRCONFIG)->Clone() : new XMLTag(XML_NODE_POINTDESCRCONFIG))
{
  std::string filename = m_Config->GetProperty(XML_ATTRIBUTE_POINTDESCRFILE);
  if(filename.length() > 0)
  {

    try
    {
      Halcon::read_descriptor_model_3d(filename.c_str(),(Hlong*)&m_DescriptoHandel);
    }
    catch(...)
    {
      printf("DecriptorModel: File not found\n");
    }

  }
}

PointDescrModel::PointDescrModel(DXFReader* dxf, Signature* sig) :
  Descriptor(((ShapeModel*)sig->GetElement(0, DESCRIPTOR_SHAPE))->GetClass()),
  m_DescriptoHandel(-1),
  m_Config(new XMLTag(XML_NODE_POINTDESCRCONFIG))
{
  /* Load necessary information from the signature*/
  ShapeModel* sm = (ShapeModel*)sig->GetElement(0, DESCRIPTOR_SHAPE);
  if(sm == NULL)
    throw "No shape model available";
  Image* img = sm->GetLastMatchedImage();
  if(img == NULL)
    throw "No matched image available";
  ShapeModelParamSet* pm = sm->GetParamSet();
  Calibration* calib = pm->m_calib;
  calib->SaveTo(m_Config);
  if(calib == NULL)
    throw "No Calibration found";
  if(dxf == NULL)
    throw "No DXF-information available";
  /* Create the overlapping region, where the 2d points have to be searched*/
  try
  {
#ifdef HALCONIMG
    if(sig->GetObjectPose() == NULL)
      throw "No Pose Available";
    Halcon::Hobject xld = sm->GetContour(*sig->GetObjectPose());
  try
  {
     m_objectPose = RelPoseFactory::CloneRelPose(sig->GetObjectPose());
  }
  catch(...)
  {
    printf("Tying to copy a singular position\n");
    m_objectPose = RelPoseFactory::FRelPoseWorld();
  }

    int num = 0;
    Halcon::count_obj(xld, (Hlong*)&num);

    Halcon::Hobject region;
    /* Generate region for descriptor extraction*/
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

    Halcon::reduce_domain(*img->GetHImage(), region, &reducedimg);
    /* Get Camparam */
    Halcon::HTuple camparam = calib->CamParam();
    /* Prepare 3d and 2d point in htuples */
    std::vector<face3dTransformed> vec = dxf->m_3dFaceData;

    Halcon::HTuple ox, oy, oz;
    Halcon::HTuple r, c;
    Halcon::Hobject reg;
    Halcon::gen_empty_region(&reg);
    /* Visibility Test for all points...*/
    for(std::vector<face3dTransformed>::const_iterator iter = vec.begin(); iter != vec.end(); iter++)
    {
      Halcon::Hobject obj;
      Halcon::HTuple x, y, z;
      Halcon::HTuple rpoly, cpoly, test;

      x.Append((*iter).m_x1);
      y.Append((*iter).m_y1);
      z.Append((*iter).m_z1);
      x.Append((*iter).m_x2);
      y.Append((*iter).m_y2);
      z.Append((*iter).m_z2);
      x.Append((*iter).m_x3);
      y.Append((*iter).m_y3);
      z.Append((*iter).m_z3);
      x.Append((*iter).m_x4);
      y.Append((*iter).m_y4);
      z.Append((*iter).m_z4);

      Halcon::project_3d_point(x,y,z, camparam, &rpoly,&cpoly);

      Halcon::HTuple rb, cb;
      rb = rpoly[0].D();
      cb = cpoly[0].D();
      rpoly.Append(rb);
      cpoly.Append(cb);
      Halcon::test_region_point(reg, rb,cb, &test);
      bool b = test[0].I() == 0;
      rb = rpoly[1].D();
      cb = cpoly[1].D();
      Halcon::test_region_point(reg, rb,cb, &test);
      b = b || test[0].I() == 0;
      rb = rpoly[2].D();
      cb = cpoly[2].D();
      Halcon::test_region_point(reg, rb,cb, &test);
      b = b || test[0].I() == 0;
      rb = rpoly[3].D();
      cb = cpoly[3].D();
      Halcon::test_region_point(reg, rb,cb, &test);
      b = b || test[0].I() == 0;


      if(b)
      {
        ox.Append((*iter).m_ox1);
        oy.Append((*iter).m_oy1);
        oz.Append((*iter).m_oz1);
        r.Append(rpoly[0].D());
        c.Append(cpoly[0].D());
        ox.Append((*iter).m_ox2);
        oy.Append((*iter).m_oy2);
        oz.Append((*iter).m_oz2);
        r.Append(rpoly[1].D());
        c.Append(cpoly[1].D());
        ox.Append((*iter).m_ox3);
        oy.Append((*iter).m_oy3);
        oz.Append((*iter).m_oz3);
        r.Append(rpoly[2].D());
        c.Append(cpoly[2].D());
        ox.Append((*iter).m_ox3);
        oy.Append((*iter).m_oy3);
        oz.Append((*iter).m_oz3);
        r.Append(rpoly[2].D());
        c.Append(cpoly[2].D());
        ox.Append((*iter).m_ox4);
        oy.Append((*iter).m_oy4);
        oz.Append((*iter).m_oz4);
        r.Append(rpoly[3].D());
        c.Append(cpoly[3].D());
        ox.Append((*iter).m_ox1);
        oy.Append((*iter).m_oy1);
        oz.Append((*iter).m_oz1);
        r.Append(rpoly[0].D());
        c.Append(cpoly[0].D());
      }
      Halcon::gen_region_polygon_filled(&obj, rpoly, cpoly);
      Halcon::union2(reg, obj, &reg);
    }
    /* Get the 2d points*/
    //Halcon::project_3d_point(x,y,z, camparam, &r,&c);
    Halcon::HTuple modelID, empty, hommat;

    m_objectPose->GetHommat(&hommat, NULL);
    /* Create the descriptor model */
    Halcon::create_descriptor_model_3d_calib(reducedimg, "harris", empty,empty,
      "randomized_fern", empty, empty, 42,ox,oy,oz,r,c, camparam, hommat ,&modelID);
    m_DescriptoHandel = modelID[0].L();
    printf("Write Descriptor Model to File\n");
    std::string stFileName("descr_obj.dcm");
    Halcon::write_descriptor_model_3d(modelID, stFileName.c_str());
    m_Config->AddProperty(XML_ATTRIBUTE_POINTDESCRFILE, stFileName);
  }
  catch(Halcon::HException ex)
  {
    printf("Error while Learning: %s\n", ex.message);
    throw "Error in Point DescrModel";
#endif
  }
  catch(...)
  {
    printf("Error while Learning: ---- unknown error ---\n");
    throw "Error in Point DescrModel";
  }
  delete calib;
}


PointDescrModel::~PointDescrModel(void)
{

  if(m_DescriptoHandel != -1)
    Halcon::clear_descriptor_model_3d(m_DescriptoHandel);

}

Calibration* PointDescrModel::GetCurCalibration()
{
  Calibration* calib  = NULL;
  if(m_Config != NULL && m_Config->GetChild(XML_NODE_CALIBRATION) != NULL)
    calib = new Calibration(m_Config);
  return calib;
}


Halcon::Hobject PointDescrModel::GetContour(RelPose& pose, Camera* cam)
{
  return Halcon::Hobject();
}


void PointDescrModel::SaveTo(XMLTag* tag)
{
  tag->AddChild(m_Config->Clone());
  std::string stFileName = m_Config->GetProperty(XML_ATTRIBUTE_POINTDESCRFILE);
  Halcon::write_descriptor_model_3d(m_DescriptoHandel, stFileName.c_str());
}

#endif
