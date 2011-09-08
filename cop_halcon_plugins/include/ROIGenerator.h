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


#ifndef ROIGENERATOR_H
#define ROIGENERATOR_H

#include "Camera.h"
#include "RegionOI.h"

namespace cop
{
  class ROIGenerator
  {
  public:
    ROIGenerator(  unsigned long timeout)
    {
      Touch();
      m_timeout = m_timestamp + timeout;
    }

    virtual RegionOI GetROI(Camera* cam)
    {
      RegionOI region;
      Halcon::gen_rectangle1(&region.m_reg, 0,0,cam->m_calibration.m_width, cam->m_calibration.m_width);
      return region;
    }

    virtual unsigned long date() const
    {
      return m_timestamp;
    }

    bool CheckTimeout()
    {
      return m_timeout > (unsigned long)time(NULL);
    }

    void Touch()
    {
      m_timestamp = (unsigned long)time(NULL);
    }
    unsigned long m_timestamp;
    unsigned long m_timeout;

    static ROIGenerator* ROIGeneratorFactory(XMLTag* tag);

  };


  class ROIManager
  {
   public:
      virtual RegionOI GetCurrentROI(Camera* cam)
      {
        RegionOI region;
          CheckTimeOut();
        if(m_roiGenerators.size() == 0)
        {
  #ifdef HalconImg
          Halcon::gen_rectangle1(&region.m_reg, 0,0,cam->m_calibration.m_width, cam->m_calibration.m_width);
  #endif /*HalconImg*/

        }
        else
        {
          RegionOI regTmp = m_roiGenerators[0]->GetROI(cam);
  #ifdef HalconImg
          Hlong num;
          Halcon::count_obj(regTmp.m_reg ,&num);
          Halcon::copy_obj(regTmp.m_reg, &region.m_reg, 1, num);
          for(int i = 1; i< m_roiGenerators.size(); i++)
          {
            Halcon::intersection(m_roiGenerators[0]->GetROI(cam).m_reg, region.m_reg, &region.m_reg);
          }
  #endif
        }
        return region;
      }
      void AddROIGenerator(ROIGenerator* gene)
      {
        if(gene != NULL)
          m_roiGenerators.push_back(gene);
      }
  private:
      void CheckTimeOut()
      {

          for(std::vector<ROIGenerator*>::iterator it = m_roiGenerators.begin();
              it != m_roiGenerators.end(); it++)
          {
            if((*it)->CheckTimeout())
               it = m_roiGenerators.erase(it);
          }
      }
      std::vector<ROIGenerator*> m_roiGenerators;
  };
}
#endif // ROIGENERATOR_H
