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
                        ColorBased.h - Copyright klank


**************************************************************************/


#ifndef COLORBASED_H
#define COLORBASED_H
#include "LocateAlgorithm.h"

#define XLM_NODE_COLORBASED "ColorBased"
namespace cop
{
  /**
    * class ColorBased
    * @brief A simple color based 2d-location algorithm
    */
  class ColorBased : public LocateAlgorithm
  {
  public:

    // Constructors/Destructors
    //

    ColorBased ();
    ColorBased (XMLTag* tag);

    /**
     * Empty Destructor
     */
    virtual ~ColorBased ( );
    //
    // methods
    //
      /**
      *   Perform the color based algorithm
      */
    virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);


    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);


    XMLTag* Save();

      std::string GetName(){return XLM_NODE_COLORBASED;}

  protected:

    // Static Protected attributes
    //

    // Protected attributes
    //


    // Protected attribute accessor methods
    //


    // Protected attribute accessor methods
    //


  private:

    // Static Private attributes
    //

    // Private attributes
    //


    // Private attribute accessor methods
    //


    // Private attribute accessor methods
    //



  };
}
#endif /* COLORBASED_H */

