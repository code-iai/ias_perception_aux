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


#ifndef BLOB_H
#define BLOB_H

#include "Descriptor.h"

#define XML_NODE_BLOB "Blob"

namespace cop
{
  class Blob :
    public Descriptor
  {
  public:
    Blob(int thresRedMin, int thresRedMax,
          int thresGreenMin, int thresGreenMax,
          int thresBlueMin, int thresBlueMax,
          int areaMin, int areaMax,
          double minRoundness, double sizeXY, double sizeZ);
    Blob();

    void SaveTo(XMLTag* tag);
    virtual void Show(RelPose* pose, Sensor* cam);

    virtual std::string GetNodeName() const{return XML_NODE_BLOB;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_BLOB;}
  protected:
    virtual void SetData(XMLTag* tag);
  public:
    ~Blob(void);

  public:
    int RedThresMin;
    int RedThresMax;
    int GreenThresMin;
    int GreenThresMax;
    int BlueThresMin;
    int BlueThresMax;
    int MinArea;
    int MaxArea;
    double SizeXY;
    double SizeZ;
    double MinRoundness;

  };
}
#endif /*BLOB_H*/
