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


#ifndef PRASHAPEVSPOINTDESCR_H
#define PRASHAPEVSPOINTDESCR_H

#include "ProveAlgorithm.h"

/**
*   class PRAShapeVsPointDescr
*   @brief Validation of models
*/
namespace cop
{
  class PRAShapeVsPointDescr :
    public ProveAlgorithm
  {
  public:
    PRAShapeVsPointDescr(void);
    PRAShapeVsPointDescr(XMLTag*);
    ~PRAShapeVsPointDescr(void);

      /**
      *   Perform
      *   @param cam
      *   @param pose
      *   @param Object
      *   @param numOfObjects
      *   @param qualityMeasure
      *   @throws char* with an error message in case of failure
      */
    virtual ImprovedPose Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

      virtual std::string GetName(){return XML_NODE_PRASP;}


    virtual XMLTag* Save();

  };
}
#endif /*PRASHAPEVSPOINTDESCR_H*/
