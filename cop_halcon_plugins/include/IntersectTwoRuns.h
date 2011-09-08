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


#ifndef INTERSECTTWORUNS_H
#define INTERSECTTWORUNS_H

#include "LocateAlgorithm.h"

#define XML_NODE_INTERSECTTWORUNS "IntersectTwoRuns"
namespace cop
{

  class IntersectTwoRuns :  public LocateAlgorithm
  {
  public:
    IntersectTwoRuns(LocateAlgorithm* first, LocateAlgorithm* second);
    IntersectTwoRuns();
    ~IntersectTwoRuns(void);

    XMLTag* Save();
    // Public attribute accessor methods
    //
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual std::string GetName(){return XML_NODE_INTERSECTTWORUNS;}
    virtual void SetData(XMLTag* tag);
  private:
    LocateAlgorithm* m_firstAlg;
    LocateAlgorithm* m_secondAlg;
    size_t m_index_sens1;
    size_t m_index_sens2;
  };
}
#endif // IntersectTwoRuns_H
