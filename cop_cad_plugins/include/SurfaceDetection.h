
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


#ifndef SURFACEDETECTION_H
#define SURFACEDETECTION_H


#include "Descriptor.h"
#include "LocateAlgorithm.h"
#include "RefineAlgorithm.h"



#define XML_NODE_SURFACEDETECTION "SurfaceDetection"
#define XML_NODE_SURFACEMODEL "SurfaceModel"
#define XML_NODE_SURFACEMODELEXTRACTION "SurfaceModelExtraction"


namespace cop
{
  class SurfaceModel : public Descriptor
  {
    public:

    virtual std::string GetNodeName() const{return XML_NODE_SURFACEMODEL;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_SURFACE;}

    void SetData(XMLTag* tag);
    void SaveTo(XMLTag* tag);

    Matrix m_hommat;
    Matrix m_approxCov;

    std::string filename;
    long m_surface_handle;
    double  m_minscore;
  };

  class SurfaceModelExtraction : public RefineAlgorithm
  {
  public:
    SurfaceModelExtraction();

    ~SurfaceModelExtraction();

    virtual Descriptor* Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    XMLTag* Save();
    void SetData(XMLTag* tag);

    virtual std::string GetName(){return XML_NODE_SURFACEMODELEXTRACTION;}

  private:
    std::vector<double> m_camparams;
    std::string m_camfilename;

  };


  class SurfaceDetection : public LocateAlgorithm
  {
  public:
    SurfaceDetection();

    ~SurfaceDetection();

    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    XMLTag* Save();
    void SetData(XMLTag* tag);

    virtual std::string GetName(){return XML_NODE_SURFACEDETECTION;}
    double m_lastTableHeight;
  private:
    std::vector<double> m_camparams;
    std::string m_camfilename;

  };
}
#endif /*SURFACEDETECTION_H*/
