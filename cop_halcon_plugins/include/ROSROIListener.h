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


#ifndef ROSROILISTENER_H
#define ROSROILISTENER_H

#include "ROIGenerator.h"

#define XML_ATTRIBUTE_TIMEOUT "Timeout"
namespace cop
{
  class ROSROIListener : public ROIGenerator
  {
  public:
      ROSROIListener(XMLTag* tag) : ROIGenerator(tag != NULL ? (tag->GetPropertyInt(XML_ATTRIBUTE_TIMEOUT)) : 0)
      {
        tag->GetName();
      }
  };
}

#endif // ROSROILISTENER_H
