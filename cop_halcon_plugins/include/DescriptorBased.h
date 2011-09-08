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
                        DescriptorBased.h - Copyright klank

riptorBased.h
**************************************************************************/


#ifndef DESCRIPTORBASED_H
#define DESCRIPTORBASED_H
#ifdef DESCRIPTOR_AVAILABLE
#include <string>

#include "LocateAlgorithm.h"
#include "Image.h"

#define XML_NODE_DESCRIPTORBASEDALG "DescriptorBasedAlg"

/**
  * class DescriptorBased
  * @brief  A 3d descriptor based loaction algorithm
  */
class DescriptorBased : public LocateAlgorithm
{
public:

	// Constructors/Destructors
	//


	/**
	* Empty Constructor
	*/
	DescriptorBased ();
	DescriptorBased (XMLTag* tag);

	/**
	* Empty Destructor
	*/
	virtual ~DescriptorBased ( );

    /**
    *   @throws char* with an error message in case of failure
    */
	std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
    /**
    *   @throws char* with an error message in case of failure
    */
	std::vector<RelPose*> Inner(Image* img,RelPose* camPose, Calibration* calib, RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);

  double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

	void Show(Camera* cam);

    std::string GetName(){return XML_NODE_DESCRIPTORBASEDALG;}

	//
	// Methods
	//
	XMLTag* Save();




	private:
		Halcon::HTuple* m_row;
		Halcon::HTuple* m_col;
	};

#endif /*DESCRIPTOR_AVAILABLE*/
#endif /* DESCRIPTORBASED_H */
