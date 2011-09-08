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


// Functions, used to determine longitude, latitude and radius from pose uncertainties (covariance).
#ifndef SEARCHPARAMS_H
#define SEARCHPARAMS_H
#include "cpp/HalconCpp.h"
#include "RelPose.h"
using namespace Halcon;
//using namespace std;

namespace cop
{
  class Calibration;
  class RegionOI;


  /*******************************************************************************
  *   GetExtremePoses                                                           */
  /*******************************************************************************
  *
  *   @param cov Takes the mean and the covariance of the pose in question.
  *
  *   @returns the 12 extreme poses (min and max for each pose parameter),
  *           according to the pose uncertainty. The poses are offsets from MEAN!
  *            Add mean pose to get absolute poses.
  *******************************************************************************/
  ReturnMatrix GetExtremePoses(const Matrix& cov/*, const ColumnVector& mean*/);

  /*******************************************************************************
  *   GetVPFromPose                                                           */
  /*******************************************************************************
  *  @param pose
  *  @param ViewPoint
  *  @param ObjGravPoint
  *  Takes a pose of the object in camera coordinates
  *   tranforms it into the viewpoint (longitude, latitude, radius),
  *   from which the camera is looking, realtive to the object's original
  *   reference pose (rotX=rotY=rotZ=0). The Result is stored in ViewPoint.
  *******************************************************************************/
  void GetVPFromPose(const HTuple& Pose, HTuple *ViewPoint, double *ObjGravPoint);


  /*******************************************************************************
  *   GetExtents                                                                */
  /*******************************************************************************
  * @param ExtremePosesTransp
  * @param MeanPose
  * @param gravPoint
  * Takes a matrix of extreme poses and creates a tuple with minimum and
  * maximum values for longitude, latitude and radius. These values correspond
  *  to the extents of the viewpoint search space around the MEAN(!) pose.
  ********************************************************************************/
  HTuple GetExtents(const Matrix& ExtremePosesTransp, const HTuple& MeanPose, double *gravPoint, Calibration* calib, RegionOI* r);
  HTuple GetExtents(const Matrix& ExtremePosesTransp, const HTuple& MeanPose, double *gravPoint, const HTuple& CamPar, RegionOI* r);
}

#endif /*SEARCHPARAMS_H*/
