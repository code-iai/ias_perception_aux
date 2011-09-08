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


#ifdef DESCRIPTOR_AVAILABLE
#include "RFAPointDescrByShape.h"
#ifndef DXFLIB
#include "DxfReader.h"
#include <algorithm>
#endif
#include "PointDescrModel.h"
#include "ShapeModel.h"

#define XML_NODE_POINTSDESCRBYSHAPE "PointdescrByShape"
#define XML_NODE_COLORBYSHAPE		"RFAColorByShape"
using namespace cop;

RFAPointDescrByShape::RFAPointDescrByShape(void)
{
}

RFAPointDescrByShape::RFAPointDescrByShape(XMLTag* tag)
{
}

RFAPointDescrByShape::~RFAPointDescrByShape(void)
{
}

Descriptor* RFAPointDescrByShape::Perform(std::vector<Sensor*> cam, RelPose* pose, Signature& sig, int &numOfObjects, double& qualityMeasure)
{
#ifdef DESCRIPTOR_AVAILABLE
	PointDescrModel* points  = NULL;
	ShapeModel* sm = (ShapeModel*)sig.GetElement(0, DESCRIPTOR_SHAPE);
	std::string fileName = sm->GetDXFFileName();
#ifndef DXFLIB
	DXFReader* di = new DXFReader(sig.GetObjectPose()); //TODO: take not this relpose, could be newer, save the repose in the ShapeModel
	DL_Dxf* dxf = new DL_Dxf();

	if (!dxf->in(fileName, di)) { // if file open failed
        std::cerr << fileName << " could not be opened.\n";
        return points;
    }
	delete dxf;
	std::sort(di->m_3dFaceData.begin(), di->m_3dFaceData.end());
	try
	{
		points = new PointDescrModel(di, &sig);
		points->Evaluate(0.5);
	}
	catch(char const* ex)
	{
		printf("Learning of Descriptorbased model Failed: %s\n", ex);
	}
	delete di;
#endif
	return points;
#else
	return NULL;
#endif
}

double RFAPointDescrByShape::CheckSignature(Signature& sig)
{
	ShapeModel* pmod = (ShapeModel*)sig.GetElement(0, DESCRIPTOR_SHAPE);
	if(pmod != NULL && sig.GetElement(0, DESCRIPTOR_FEATURE) == NULL)
	{
		Image* img = pmod->GetLastMatchedImage();
		if(img != NULL)
		{
			return 1.0;
		}
	}
	return 0.0;
}

XMLTag* RFAPointDescrByShape::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_POINTSDESCRBYSHAPE);

	//TODO: parameter?
	return tag;
}
#endif
