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

#include "RFASurfaceByShape.h"
#include "ShapeModel.h"
#include "SurfaceDetection.h"
#include "Camera.h"

using namespace cop;



RFASurfaceByShape::RFASurfaceByShape()
{
}

void RFASurfaceByShape::SetData(XMLTag* tag)
{
  printf("Loading Algorithm RFASurfaceByShape\n");
}

RFASurfaceByShape::~RFASurfaceByShape(void)
{
}



Descriptor* RFASurfaceByShape::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
  SurfaceModel *surface_model = NULL;
  ShapeModel *model = (ShapeModel*)sig.GetElement(0, DESCRIPTOR_SHAPE);
  if(model != NULL)
  {
    std::string stFileName = model->GetDXFFileName();
    ShapeModelParamSet* sm = model->GetParamSet();

    if(stFileName.length() > 0)
    {
      surface_model = new SurfaceModel();
      surface_model->SetClass((Class*)(model->GetClass()->Duplicate(true)));
      surface_model->filename = stFileName + ".sfm";
      try
      {
        Halcon::HTuple OM3DID, Status, SMID, empty, pn, pv;
        pn = "convert_to_triangles";
        pv = "true";
        tuple_concat(pn, "invert_normals" , &pn);
        tuple_concat(pv, sm->m_invertNormals ? "true" : "false", &pv);
        
        Halcon::read_object_model_3d (stFileName.c_str(), sm->m_measure, "convert_to_triangles", "true", &OM3DID, &Status);
        Halcon::create_surface_model (OM3DID, 0.03, empty, empty, &SMID);
        Halcon::write_surface_model(SMID, (stFileName + ".sfm").c_str());
        surface_model->m_surface_handle = SMID[0].L();
        surface_model->m_hommat = IdentityMatrix(4);
        surface_model->m_approxCov = IdentityMatrix(6);
        surface_model->m_approxCov.element(0,0) = 0.01; surface_model->m_approxCov.element(1,1) = 0.01; surface_model->m_approxCov.element(2,2) = 0.01;
        surface_model->m_approxCov.element(3,3) = 0.01; surface_model->m_approxCov.element(4,4) = 0.01; surface_model->m_approxCov.element(4,4) = 0.01;

      }
      catch(Halcon::HException ex)
      {
        ROS_ERROR("Error reading object model: %s\n", ex.message);
        throw "Error in RFASurfaceByShape::Perform";
      }
    }
  }
  surface_model->Evaluate(1.0, 100.0);
  return surface_model;

}

double RFASurfaceByShape::CheckSignature(const Signature& sig, const  std::vector<Sensor*> &sens)
{
  if(sig.GetElement(0,DESCRIPTOR_SURFACE) == NULL)
  {
    if(sig.GetElement(0, DESCRIPTOR_SHAPE) != NULL)
      return 1.0;
    else
      return -0.0;
  }
  else
    return -0.0;
}

XMLTag* RFASurfaceByShape::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  return tag;
}


