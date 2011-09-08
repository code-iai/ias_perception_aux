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



#ifndef CALTAB_H
#define CALTAB_H

#include "Descriptor.h"

#define XML_NODE_CALTAB "CalTab"

namespace cop
{
  class CalTab : public Descriptor
  {
  public:
    CalTab(std::string stCalTabDescriptionFile);
    CalTab();
    ~CalTab(void);

    void SaveTo(XMLTag* tag);

    /***********************************************************************
    * ShapeModel::Show                                                     */
    /************************************************************************
    * @brief if this Descriptor can be showed, show it.
    * @param pose A position where this descriptor should be displayed,
    * @param camera that took the picture where the descriptor was displayed
    *************************************************************************/
    virtual void Show(RelPose* pose, Sensor* cam);

    virtual std::string GetNodeName() const{return XML_NODE_CALTAB;}
    virtual ElemType_t GetType() const{return DESCRIPTOR_CALTAB;}
  protected:
    virtual void SetData(XMLTag* tag);
  public:
    std::string m_stCalTabDescriptionFile;
    double m_alpha;
    int m_st;
    double m_steps;
    int m_mt;
    int m_cont;
    int m_diam;
    int m_calThres;
    int m_holes;
  };
}
#endif /*CALTAB_H*/
