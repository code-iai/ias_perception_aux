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

 
#ifndef DXFREADER_H
#define DXFREADER_H

#include <dl_dxf.h>
#include <dl_creationadapter.h>
#include <lo/NewMatExhaustive.h>
#include "MeshProcessing.h"
#include <vector>
#ifndef OLD_DXFLIB_VERSION
#define OLD_DXFLIB_VERSION
#endif
/**
*   @brief describes all data for a face that was read from an dxf file and the transformed actual representation
*/
struct face3dTransformed
{
    face3dTransformed(	double distToZ, double x1,double y1,double z1,
                    double x2, double y2, double z2,double x3,double y3,double z3,
                    double x4, double y4, double z4,double ox1,double oy1,double oz1,
                    double ox2, double oy2, double oz2,double ox3,double oy3,double oz3,
                    double ox4, double oy4, double oz4) :
    m_distToZ(distToZ),
    m_x1(x1),
    m_y1(y1),
    m_z1(z1),
    m_x2(x2),
    m_y2(y2),
    m_z2(z2),
    m_x3(x3),
    m_y3(y3),
    m_z3(z3),
    m_x4(x4),
    m_y4(y4),
    m_z4(z4),
    m_ox1(ox1),
    m_oy1(oy1),
    m_oz1(oz1),
    m_ox2(ox2),
    m_oy2(oy2),
    m_oz2(oz2),
    m_ox3(ox3),
    m_oy3(oy3),
    m_oz3(oz3),
    m_ox4(ox4),
    m_oy4(oy4),
    m_oz4(oz4)
    {
    }
    double m_distToZ;
    double m_x1;
    double m_y1;
    double m_z1;
    double m_x2;
    double m_y2;
    double m_z2;
    double m_x3;
    double m_y3;
    double m_z3;
    double m_x4;
    double m_y4;
    double m_z4;
    double m_ox1;
    double m_oy1;
    double m_oz1;
    double m_ox2;
    double m_oy2;
    double m_oz2;
    double m_ox3;
    double m_oy3;
    double m_oz3;
    double m_ox4;
    double m_oy4;
    double m_oz4;
};

bool operator < (const face3dTransformed& f1, const face3dTransformed &f2);

bool operator < (const std::pair<Polygon_t, Polygon_t> &f1, const std::pair<Polygon_t, Polygon_t> &f2);
/********************************************************************
*   class DXFReader                                          */
/** *******************************************************************
*   @brief Implemnts DL_CreationAdapter of dxflib for opening df files
*   Should return all triangles in a dxf mesh
*
*********************************************************************/
class DXFReader : public DL_CreationAdapter {

public:
    DXFReader(Matrix relPose, double scale = 100) :
            m_relpose(relPose),
            m_scale(scale)
    {}
    virtual void addPolyline(const DL_PolylineData&);
    virtual void add3dFace(const DL_3dFaceData& data);
    virtual void addVertex(const DL_VertexData& data);
    virtual void endEntity(){}
protected:
    Matrix m_relpose;
    double m_scale;
public:
    std::vector<face3dTransformed> m_3dFaceData;
};


/********************************************************************
*   class DXFReader2                                          */
/** *******************************************************************
*   @brief Implemnts DL_CreationAdapter of dxflib for opening df files
*   Should return all triangles in a dxf mesh
*
*********************************************************************/
class DXFReader2 : public DL_CreationAdapter {

public:
    DXFReader2(Matrix relPose, double scale = 100) :
            m_relpose(relPose),
            m_scale(scale),
            count_points(0)
    {}
    virtual void addPolyline(const DL_PolylineData& data)
    {
        n = data.n;
        m = data.m;
        number = data.number;
        flags = data.flags;
    }
void TransformPointLocally(const Matrix m, double x_in, double y_in, double z_in, double &x_out,double &y_out,double &z_out, double scale)
{
  ColumnVector d(4);
  d << x_in / scale << y_in  / scale << z_in / scale<< 1;
  ColumnVector f = m * d;
//  cout << m << " \n* \n" << d << "\n=\n" << f;
  x_out = f.element(0);
  y_out = f.element(1);
  z_out = f.element(2);
}

    virtual void add3dFace(const DL_3dFaceData& data)
    {
        Point3d_t p1, p2, p3, p4;
        Point3d_t p1_orig, p2_orig, p3_orig, p4_orig;
    #ifdef OLD_DXFLIB_VERSION
        TransformPointLocally(m_relpose, data.x1,data.y1,data.z1,p1.x,p1.y,p1.z, m_scale);
        TransformPointLocally(m_relpose, data.x2,data.y2,data.z2,p2.x,p2.y,p2.z, m_scale);
        TransformPointLocally(m_relpose, data.x3,data.y3,data.z3,p3.x,p3.y,p3.z, m_scale);
        TransformPointLocally(m_relpose, data.x4,data.y4,data.z4,p4.x,p4.y,p4.z, m_scale);
    #else
        TransformPointLocally(m_relpose, data.x[0],data.y[0],data.z[0], p1.x,p1.y,p1.z, m_scale);
        TransformPointLocally(m_relpose, data.x[1],data.y[1],data.z[1], p2.x,p2.y,p2.z, m_scale);
        TransformPointLocally(m_relpose, data.x[2],data.y[2],data.z[2], p3.x,p3.y,p3.z, m_scale);
        TransformPointLocally(m_relpose, data.x[3],data.y[3],data.z[3], p4.x,p4.y,p4.z, m_scale);
    #endif
        p1_orig.x = data.x1 / m_scale;
        p1_orig.y = data.y1 / m_scale;
        p1_orig.z = data.z1 / m_scale;
        p2_orig.x = data.x2 / m_scale;
        p2_orig.y = data.y2 / m_scale;
        p2_orig.z = data.z2 / m_scale;
        p3_orig.x = data.x3 / m_scale;
        p3_orig.y = data.y3 / m_scale;
        p3_orig.z = data.z3 / m_scale;
        p4_orig.x = data.x4 / m_scale;
        p4_orig.y = data.y4 / m_scale;
        p4_orig.z = data.z4 / m_scale;

        Polygon_t poly;
        poly.push_back(p1);
        poly.push_back(p2);
        poly.push_back(p3);
        poly.push_back(p4);
        Polygon_t poly_orig;
        poly_orig.push_back(p1_orig);
        poly_orig.push_back(p2_orig);
        poly_orig.push_back(p3_orig);
        poly_orig.push_back(p4_orig);
        m_3dFaceData.push_back(std::pair< Polygon_t,Polygon_t>(poly, poly_orig));
        count_points += 4;
    }
    virtual void addVertex(const DL_VertexData& data)
    {
      if(data.flags == 128)
      {
        Polygon_t poly_orig, poly;
        poly.push_back(tmp_point_list[abs(data.i1) - 1]);
        poly.push_back(tmp_point_list[abs(data.i2) - 1]);
        poly.push_back(tmp_point_list[abs(data.i3) - 1]);
        
        poly_orig.push_back(tmp_point_list_orig[abs(data.i1) - 1]);
        poly_orig.push_back(tmp_point_list_orig[abs(data.i2) - 1]);
        poly_orig.push_back(tmp_point_list_orig[abs(data.i3) - 1]);

        m_3dFaceData.push_back(std::pair< Polygon_t,Polygon_t>(poly, poly_orig));
        count_points+= 3;
      }
      else
      {
        Point3d_t p_n, p_n_orig;
        p_n_orig.x = data.x / m_scale;
        p_n_orig.y = data.y / m_scale;
        p_n_orig.z = data.z / m_scale;
       tmp_point_list_orig.push_back(p_n_orig);
        TransformPointLocally(m_relpose, data.x,data.y,data.z,p_n.x,p_n.y,p_n.z, m_scale);
        tmp_point_list.push_back(p_n);
      }
    }
    virtual void endEntity(){}
protected:
    Matrix m_relpose;
    double m_scale;
    Polygon_t tmp_point_list;
    Polygon_t tmp_point_list_orig;
    unsigned  int n , m , number ;
    int flags ;
public:
    int count_points ;
    Mesh_t m_3dFaceData;

};

Mesh_t ReadMesh(std::string filename, double scale, Matrix m, int &points);




#endif /*DXFREADER_H*/
