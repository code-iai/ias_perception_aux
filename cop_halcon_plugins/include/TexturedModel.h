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
                        TexturedModel.h - Copyright klank

**************************************************************************/


#ifndef TEXTUREDMODEL_H
#define TEXTUREDMODEL_H

#include "Descriptor.h"

namespace cop
{
  class XMLTag;


  #define XML_NODE_TEXMODEL "TextureModel"

  /**
    * class TexturedModel
    */

  class TexturedModel : public Descriptor
  {
  public:

    // Constructors/Destructors
    //


    /**
    * Empty Constructor
    */
    TexturedModel ( Class* pClass);
    TexturedModel ( );

    /**
    * Empty Destructor
    */
    virtual ~TexturedModel ( );

    // Static Public attributes
    //

    // Public attributes
    //


    // Public attribute accessor methods
    //
    virtual std::string GetNodeName() const{return XML_NODE_TEXMODEL;}


    // Public attribute accessor methods
    //


  protected:
    virtual void SetData(XMLTag* tag);
    // Static Protected attributes
    //

    // Protected attributes
    //


    // Protected attribute accessor methods
    //


    // Protected attribute accessor methods
    //


  private:

    // Static Private attributes
    //

    // Private attributes
    //


    // Private attribute accessor methods
    //


    // Private attribute accessor methods
    //



  };
}
#endif // TEXTUREDMODEL_H
