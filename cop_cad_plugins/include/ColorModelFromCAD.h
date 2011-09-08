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


#ifndef COLORMODELFROMCAD_H
#define COLORMODELFROMCAD_H

#include "Descriptor.h"
#include "ColorModel.h"

#define XML_NODE_COLORMODELFROMCAD "ColorModelFromCAD"
namespace cop
{
  class Signature;
  /**
  *   class ColorModel
  *   @brief Signature that contains an color model for a simple ColorBased detection
  */
  class ColorModelFromCAD : public ColorModel
  {
   public:
    ColorModelFromCAD(Class* classref, Signature* sig);

    virtual std::string GetNodeName() const {return XML_NODE_COLORMODELFROMCAD;}
  };
}
#endif /*COLORMODELFROMCAD_H*/
