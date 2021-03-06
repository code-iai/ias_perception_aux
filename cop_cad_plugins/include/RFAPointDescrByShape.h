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


#ifndef RFAPOINTDESCRBYSHAPE_H
#define RFAPOINTDESCRBYSHAPE_H
#ifdef DESCRIPTOR_AVAILABLE

#include "RefineAlgorithm.h"

class RFAPointDescrByShape :
	public RefineAlgorithm
{
public:
	RFAPointDescrByShape(void);
	RFAPointDescrByShape(XMLTag* tag);

	~RFAPointDescrByShape(void);

	virtual Descriptor* Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

  double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

	virtual XMLTag* Save();

   virtual std::string GetName(){return XML_NODE_POINTSDESCRBYSHAPE;}

};
#endif
#endif /*RFAPOINTDESCRBYSHAPE_H*/
