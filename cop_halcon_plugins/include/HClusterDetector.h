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


#ifndef CLUSTERDETECTOR_H
#define CLUSTERDETECTOR_H

#define XML_NODE_HCLUSTERDETECTOR "HClusterDetector"

#include "LocateAlgorithm.h"
class PlaneClusterResult;

namespace cop
{

  class SegmentPrototype;

  class HClusterDetector : public LocateAlgorithm
  {
  public:
    HClusterDetector(int swiss, int ptu_base);
    HClusterDetector();

    ~HClusterDetector();

    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure);

    std::vector<RelPose*> Inner(Sensor* sens,RelPose* innial_pose_estimate,  SegmentPrototype* proto, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    bool TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose);

    XMLTag* Save();
    void SetData(XMLTag* tag);

    virtual std::string GetName(){return XML_NODE_HCLUSTERDETECTOR;}

    static bool CallStaticPlaneClusterExtractor(Sensor* sens, PlaneClusterResult* response, int m_ptu_jlo_id, bool parallel);

    std::vector<double> m_camparams;
  private:
    int m_swissranger_jlo_id;
    int m_ptu_jlo_id;

    double m_table_noise;
    double m_min_object_size_for_split;
    double m_min_table_percentage;
  };
}
#endif /*CLUSTERDETECTOR_H*/
