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
                        ShapeModel.h - Copyright klank
**************************************************************************/


#ifndef SHAPEMODEL_H
#define SHAPEMODEL_H
#include "Descriptor.h"
#include "XMLTag.h"
#include "Camera.h"
#include "RegionOI.h"
#include <sstream>

#include "MeshProcessing.h"

#define NAME_EXTENT_FROM_DXF_TO_SHAPE "gen.sm3"


namespace Halcon
{
  class HShapeModel3D;
}

#define XML_NODE_SHAPEMODEL "ShapeModel"
#define XML_NODE_SHAPEPARAM_LIST "ShapeParamList"
#define XML_NODE_SHAPEPARAM_ENTRY "ShapeParamEntry"
namespace cop
{
  /**
    * class ShapeModelParamSet
    * @brief A parameter set for a ShapeModel
    */
  class ShapeModelParamSet
  {
  public:
    ShapeModelParamSet(Calibration* calib, double lgmi = -0.8, double lgma=0.8,double ltmi = -0.7, double ltma=0.8,double crmi= -3.141, double crma= 3.141,double dimi=0.7, double dima =1.5, double refRoX=3.3, double refRoY=5.7, double refRoZ=6.1, double measure=0.01, double scale = 1.0, int minContrast = 10, double minFaceAngle = 0.3);
    ShapeModelParamSet(XMLTag*, Calibration* calib = NULL, bool deleteCalib = false);
    ~ShapeModelParamSet(){
      delete m_region;
      if(m_deleteCalib)delete m_calib;}

    XMLTag* Save();
    void SetRegion(RegionOI* reg){if(m_region != NULL)m_region = reg;}
    double* GetGravPoint(std::string st);
    double m_longitudeMin;
    double m_longitudeMax;
    double m_latitudeMin;
    double m_latitudeMax;
    double m_camRollMin;
    double m_camRollMax;
    double m_initRefX;
    double m_initRefY;
    double m_initRefZ;
    double m_distMin;
    double m_distMax;
    double m_measure;
    double m_scale;
    int m_minContrast;
    double m_minFaceAngle;
    RegionOI* m_region;
    Calibration* m_calib;
    bool m_deleteCalib;
    double m_gravFinal[3];
    bool m_gravPointInited;
    bool m_invertNormals;
  };


  /**
    * class ShapeModel
    * @brief Loads, Saves and Creates 3D shape models for ShapeBased3D
    */
  class ShapeModel : public Descriptor
  {
  public:

    // Constructors/Destructors
    //
    /**
     * Empty Constructor
     */
    ShapeModel (Class* classref );
    ShapeModel ();
    virtual ~ShapeModel();

  /** public static*/
    static double RotationError(double rot_x_1, double rot_y_1, double rot_z_1, double rot_x_2, double rot_y_2, double rot_z_2);
    /****************************************************************************/
    /**  CheckOverlapping:
     ****************************************************************************
     *
     *  @brief compares two shape models
     *
     *  \pi     pose
     *  \pi     pm
     *  \pi     stFilename
     *
     *  @returns 0 on incompatibility 1 on complete compatibility
     *****************************************************************************
     *
     *****************************************************************************/
    static double CheckOverlapping(ShapeModelParamSet* sm_new, ShapeModelParamSet* sm_old);

    /****************************************************************************/
    /**  PoseToRange: takes an lo and generates a shapemodelparam set
     ****************************************************************************
     *
     *  @brief takes an lo and generates a shapemodelparam set
     *
     *  \pi     pose
     *  \pi     pm
     *  \pi     stFilename
     *
     *  @throws char* with an error message in case of failure
     *****************************************************************************
     *
     *****************************************************************************/
    bool PoseToRange(RelPose* pose, ShapeModelParamSet &pm, double* gravPoint, Calibration* calib);

  /****************************************************************************/
  /**  SetShapeModelParamSet: sets a new parameter set, return the index
   * it was mapped to
   ****************************************************************************
   *
   *  loads the dxf model and creates the shape model
   *
   *  \pi     paramset	path to the file
   *  \pi     st	    calibration matrix is needed for creation of an shape model
   *	\pi 		shapeModelCreated we are sure, that the file found under
   *	        GenShapeModelFileName(stFielname,..) exists and it is actual
   *
   *  @throws char* with an error message in case of failure
  *****************************************************************************
  *
  *****************************************************************************/
  int SetShapeModelParamSet(ShapeModelParamSet* paramset, std::string st, bool shapeModelCreated);
  /****************************************************************************/
  /**  SetShapeModelParamSet: sets a new parameter set, return the index
   * it was mapped to
   ****************************************************************************
   *
   *  loads the dxf model and creates the shape model
   *
   *  \pi     stFileName	path to the file
   *  \pi     calib	    calibration matrix is needed for creation of an shape model
   *	\pi 		shapeModelCreated we are sutrre, that the file found under
   *	        GenShapeModelFileName(stFielname,..) exists and it is actual
   *
   *  @throws char* with an error message in case of failure
  *****************************************************************************
   *
   *****************************************************************************/
        bool SetShapeModelParamSet(RelPose* pose, Calibration* calib, double prob = 0.8);

    virtual std::string GetNodeName() const{return XML_NODE_SHAPEMODEL;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_SHAPE;}

    /**
    *     Get the last loaded shape model id
    * @param scale returns the scale factor that has to be applied on the image to apply this scaling model correctly
    * @remarks This means the image resolution has to be reduced/adapted by the returned factor before search
    *
    */
    long GetShapeModel(double &scale);

    std::string GetDXFFileName(){return m_shapeParams_file[m_curIndex].second.first;}
    ShapeModelParamSet* GetParamSet(){return m_shapeParams_file[m_curIndex].first;}

    /***********************************************************************
    * ShapeModel::GetMesh                                                  */
    /************************************************************************
    * @brief Loads the object model and returns the content as a mesh.
    * @param pose Refernce pose for the mesh
    *************************************************************************/
    Mesh_t GetMesh(RelPose* pose, double measure_adaption = 1.0);
    
    /*********************************************************************** 
    * ShapeModel::GetShape                                                */
    /************************************************************************
    * @brief put the mesh information to objectShape.
    *************************************************************************/
    bool GetShape(GeometricShape &objectShape) const;

    /***********************************************************************
    * ShapeModel::Save                                                      */
    /************************************************************************
    * @brief Saves all properties of the shapemodel to a xmltag generated by
    *   the original Elem::Save.
    * @param tag xml tag
    *************************************************************************/
    void SaveTo(XMLTag* tag);
    /***********************************************************************
    * ShapeModel::Show                                                     */
    /************************************************************************
    * @brief if this Descriptor can be showed, show it.
    * @param pose A position where this descriptor should be displayed,
    * @param camera that took the picture where the descriptor was displayed
    *************************************************************************/
    virtual void Show(RelPose* pose, Sensor* cam);
    void ShowRegion(Camera* cam);

    double IntersectWithSupportingPlane(RelPose* planePose, RelPose* matchPose, RelPose* cameraPose, Calibration* calib);
    static void CalcMeshCenterBox(const Mesh_t &mesh, double &x, double &y, double &z, double &height,double &width, double &depth);
    void EvalScaling( ShapeModelParamSet* sm,  Calibration* calib, std::string stFileName, RelPose* pose);

    double m_initializationLevel;
  protected:
    virtual void SetData(XMLTag* tag);
  public:
    Halcon::Hobject GetContour(RelPose& rpose, Camera* cam =NULL);
  private:
    void WriteShapeModelThreaded(Hlong shapeModelId, const char* name);

    std::string GenShapeModelFileName(std::string stFileNameDxf, int index)
    {
      std::ostringstream os;
      os  << stFileNameDxf << index << NAME_EXTENT_FROM_DXF_TO_SHAPE;
      std::string ret(os.str());
      return ret;
    }

    long m_ShapeModelID;
    int m_curIndex;
    std::vector< std::pair<ShapeModelParamSet*, std::pair<std::string, bool > > > m_shapeParams_file;
    bool m_initialized;
  public:
    bool m_showNow;

  };
}
#endif // SHAPEMODEL_H
