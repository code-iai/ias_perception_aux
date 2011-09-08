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


#ifndef REGIONOI_H
#define REGIONOI_H
#include "Descriptor.h"
#include "XMLTag.h"
#include <cpp/HalconCpp.h>

#define XML_NODE_ROI "RegionOI"
typedef std::vector<std::pair< int, std::pair<int, int > > >  RegionRuns;
namespace cop
{
  class Calibration;
  class SegmentPrototype;

  class RegionOI
  {
  public:
    RegionOI(RegionRuns row_colStart_colEnd);
    RegionOI(Halcon::Hobject region) : m_reg(region){};
    RegionOI(std::string stFilename);
    RegionOI(RelPose* pose, LocatedObjectID_t cam_pose_id, Calibration* calib);
    RegionOI(RelPose* pose, LocatedObjectID_t cam_pose_id, Halcon::HTuple CamPar);

    RegionOI(SegmentPrototype* proto, LocatedObjectID_t cam_pose_id, Calibration* calib);
    RegionOI(RelPose* pose);

    RegionOI();
    ~RegionOI(void){}
    XMLTag* Save(std::string tagName);
    /*********************************************************************
    *   AddPoint                                                         */
    /** *******************************************************************
    *   @brief  Adds a hull point to the region
    *   @param Row y coordinate
    *   @param Column x coordinate
    *
    *********************************************************************/
    void AddPoint(double Row, double Column);
    /*********************************************************************
    *   GetSize                                                          */
    /** *******************************************************************
    *   @brief return the number of pixels covered by the given region
    *
    *********************************************************************/
    int GetSize();
    /*********************************************************************
    *   TransitiveHull                                                   */
    /** *******************************************************************
    *   @brief  Calculates and dilates the transitive hull
    *
    *********************************************************************/
    void TransitiveHull(int camwidth = 1600);
    Halcon::Hobject& GetRegion(double scale = 1.0);
    Halcon::Hobject m_reg;
    Halcon::Hobject m_regZoomTmp;

  };
}
#endif /*REGIONOI_H*/
