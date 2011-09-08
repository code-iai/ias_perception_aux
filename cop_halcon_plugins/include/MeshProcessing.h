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

 
#ifndef MESHPROCESSING_H
#define MESHPROCESSING_H

#include <vector>

typedef struct
{
  double x;
  double y;
  double z;
} Point3d_t;

typedef std::vector<Point3d_t> Polygon_t;

typedef std::vector<std::pair<Polygon_t, Polygon_t> > Mesh_t;

#endif /*MESHPROCESSING_H*/
