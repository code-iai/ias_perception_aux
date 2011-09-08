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


#ifndef CHECKCOLORCLASS_H
#define CHECKCOLORCLASS_H

#include "LocateAlgorithm.h"
#include "cpp/HalconCpp.h"

#define XML_NODE_CHECKCOLORCLASS "CheckColorClass"
namespace cop
{
  class CheckColorClass :  public LocateAlgorithm
  {
  public:
    CheckColorClass(std::string path);
    CheckColorClass();
    ~CheckColorClass(void);

    XMLTag* Save();
    // Public attribute accessor methods
    //
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
    void Inner(Halcon::Hobject* img, Halcon::Hobject* region, std::string &color, std::map<std::string, double> &hist, double &max_score);
    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual std::string GetName(){return XML_NODE_CHECKCOLORCLASS;}

    virtual void SetData(XMLTag* tag);
  private:
    std::string m_stPath;
    Halcon::HTuple m_Colors;
    int m_MLPHandle;
  };
}
#endif /*FINDCALTAB_H*/
