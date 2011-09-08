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



#ifndef SIMULATEDLOCATE_H
#define SIMULATEDLOCATE_H
#include "LocateAlgorithm.h"
#define XML_NODE_SIMLOCATE "SimulatedLocate"
namespace cop
{
  /**
    * class LocateAlgorithm
    * @brief Specialisation of algoithms that return a 3d position as an result
    */
  class SimulatedLocate : public LocateAlgorithm
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    SimulatedLocate (){};

    /**
     * Empty Destructor
     */
    virtual ~SimulatedLocate ( ){};

    virtual std::string GetName(){return XML_NODE_SIMLOCATE;}

    // Static Public attributes
    //

    // Public attribute accessor methods
    //
    virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors){return 1.0;}

    virtual bool TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose){return false;}
    virtual XMLTag* Save();
  };
}
#endif // SIMULATEDLOCATE_H
