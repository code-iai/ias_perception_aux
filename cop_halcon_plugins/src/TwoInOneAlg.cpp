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


#include "TwoInOneAlg.h"
#include "XMLTag.h"

using namespace cop;


TwoInOneAlg::TwoInOneAlg(LocateAlgorithm* first, LocateAlgorithm* second) :
  m_firstAlg(first),
  m_secondAlg(second)
{
}

TwoInOneAlg::TwoInOneAlg()
{
}

void TwoInOneAlg::SetData(XMLTag* tag)
{
  printf("Loading Algorithm TwoInOneAlg\n");
  if(tag != NULL && tag->CountChildren() > 1)
  {
    m_firstAlg = LocateAlgorithm::LocAlgFactory(tag->GetChild(0));
    m_secondAlg = LocateAlgorithm::LocAlgFactory(tag->GetChild(1));
  }
  else
     throw "Error loading TwoInOneAlg";
}


TwoInOneAlg::~TwoInOneAlg(void)
{
  delete m_firstAlg;
  delete m_secondAlg;
}


XMLTag* TwoInOneAlg::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_TWOINONEALG);
  tag->AddChild(m_firstAlg->Save());
  tag->AddChild(m_secondAlg->Save());
  return tag;
}
// Public attribute accessor methods
//
std::vector<RelPose*> TwoInOneAlg::Perform(std::vector<Sensor*> cam, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  int numOfObjects_first = max(numOfObjects, 10);
  double qualityMeasure_first = qualityMeasure;
  std::vector<RelPose*> results;
  std::vector<RelPose*> result_first = m_firstAlg->Perform(cam, pose, object, numOfObjects_first, qualityMeasure_first);
  numOfObjects = 0;
  if(numOfObjects_first > 0)
  {
    int numOfObjects_second = 1;
    double qualityMeasure_second = qualityMeasure;
    for(unsigned int i = 0; i < result_first.size(); i++)
    {
      std::vector<RelPose*> result_second = m_secondAlg->Perform(cam, result_first[i], object, numOfObjects_second, qualityMeasure_second);
      if(numOfObjects_second > 0)
      {
        RelPose* pose = result_second[0];
        pose->m_qualityMeasure = result_first[i]->m_qualityMeasure * qualityMeasure_second;
        results.push_back(pose);
        if(i == 0)
        {
           qualityMeasure = qualityMeasure_first * qualityMeasure_second;
           numOfObjects = 1;
        }
        else
          numOfObjects++;
      }
    }
  }
  return results;
}

double TwoInOneAlg::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  printf("TwoInOneAlg::CheckSignature\n");
  double val1 = m_firstAlg->CheckSignature(object, sensors);
  double val2 = m_secondAlg->CheckSignature(object, sensors);
  if(val1 < 0.00001 || val2 < 0.00001)
    return 0.0;
  printf("return %f\n", val1 + val2);
  return  val1 + val2;
}
