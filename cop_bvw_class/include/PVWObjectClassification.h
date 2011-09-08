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


#ifndef PVWOBJECTCLASSIFICATION_H
#define PVWOBJECTCLASSIFICATION_H

#define XML_NODE_PVW "PVWObjectClassification"

#include "RefineAlgorithm.h"

namespace cop
{

  class PVWObjectClassification :
    public RefineAlgorithm
  {
  public:
    PVWObjectClassification();

    void SetData(XMLTag* tag);

    ~PVWObjectClassification(void);

    virtual Descriptor* Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual XMLTag* Save();

    virtual std::string GetName(){return XML_NODE_PVW;}
  private:
     long m_svmHandle;
     long m_descrHandle;
     std::string m_svm_filename;
     std::string m_descr_filename;
     std::string m_tuple_filename;

     std::vector<std::string> m_classNames;

  };
}
#endif /*PVWOBJECTCLASSIFICATION_H*/
