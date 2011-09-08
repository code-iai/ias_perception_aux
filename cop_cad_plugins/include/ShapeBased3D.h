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
                        ShapeBased3D.h - Copyright klank


**************************************************************************/


#ifndef SHAPEBASED3D_H
#define SHAPEBASED3D_H
#include "LocateAlgorithm.h"
#include "Camera.h"

#include <string>

#define XML_NODE_SHAPEBASED3DALG "ShapeBased3DAlg"

namespace Halcon
{
  class HTuple;
}

namespace cop
{
  /**
    * class ShapeBased3D
    * @brief Halcons ShapeBased3d algorithms
    */
  class ShapeBased3D : public LocateAlgorithm
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    ShapeBased3D ();

    /**
     * Empty Destructor
     */
    virtual ~ShapeBased3D ( );

    // Methods
    //
    XMLTag* Save();
    void SetData(XMLTag* tag);
    // Public attributes
    //

    /*******************************************************************************
    *  Perform                                                                    */
    /*******************************************************************************
    *  @brief Calls the corresponding Inner Function
    *  @param cam should contain one camera, that provides a actual image
    *  @param pose The estimated pose or pose range that the algorithms should search
    *  @param object The signature containing the model information that is necessary
    *                 for ShapeBased3D Matching (shoudl contin atleast one ShapeModel)
    *  @param numofObjects in: the maximal number of Number of objects to be
    *                       searched, out: the number of objects found
    *  @param qualityMeasure The quality of the first match
    *  @returns a vector of rel poses
    *******************************************************************************/
    virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure);

    /*******************************************************************************
    *  CheckSignature                                                             */
    /*******************************************************************************
    *  @brief Looks up if the Signature contains a shape Model
    *  @param object The signature containing the model information that is necessary
    *                 for ShapeBased3D Matching (shoudl contin atleast one ShapeModel)
    *  @param sensors the list of available sensors that should contain at least 1
                      camera. otherwise the return value will be 0.0
    *  @return 0.0 if there is no Shape Model, otherwise 1.0 TODO: return max
    *               descriptor quality
    *******************************************************************************/
    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    /*******************************************************************************
    *  Inner                                                                      */
    /*******************************************************************************
    *  @brief Calls the corresponding Inner Function
    *  @param Image An image
    *  @param camPose The pose of the camera
    *  @param Calibration a corresponding calibration
    *  @param camPose The estimated pose or pose range that the algorithms should search
    *  @param object The signature containing the model information that is necessary
    *                 for ShapeBased3D Matching (shoudl contin atleast one ShapeModel)
    *  @param numofObjects in: the maximal number of Number of objects to be
    *                       searched, out: the number of objects found
    *  @param qualityMeasure The quality of the first match
    *  @returns a vector of rel poses
    *******************************************************************************/
    virtual std::vector<RelPose*> Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure, int i = 0);

    virtual std::string GetName(){return XML_NODE_SHAPEBASED3DALG;}

  private:
    std::vector<RelPose*>  PerformForAll(Image* img, RelPose* camPose,Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);

    int FindBestSupportedMatch(int &numOfObjects, std::vector<std::vector< RelPose*> > poses, std::vector<double> quality);
    // Private attributes
    //
    double m_minScore;
    double m_greediness;
    int    m_levels;
    Halcon::HTuple* m_paramNameList;
    Halcon::HTuple* m_paramList;
  };
}
#endif // SHAPEBASED3D_H
