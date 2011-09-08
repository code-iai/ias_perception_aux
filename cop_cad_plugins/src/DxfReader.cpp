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


#include "DxfReader.h"
#include <algorithm>


bool operator < (const face3dTransformed& f1, const face3dTransformed &f2)
{
  return f1.m_distToZ < f2.m_distToZ;
}

bool operator < (const std::pair<Polygon_t, Polygon_t> &f1, const std::pair<Polygon_t, Polygon_t> &f2)
{
  Point3d_t p1 = f1.first[0];
  Point3d_t p2 = f1.first[1];
  Point3d_t p3 = f1.first[2];

  Point3d_t q1 = f2.first[0];
  Point3d_t q2 = f2.first[1];
  Point3d_t q3 = f2.first[2];


  return (p1.x* p1.x+ p1.y * p1.y + p1.z * p1.z) +
                          (p2.x * p2.x + p2.y * p2.y + p2.z * p2.z) +
                          (p3.x * p3.x + p3.y * p3.y + p3.z * p3.z)
                              < (q1.x * q1.x + q1.y * q1.y + q1.z * q1.z) +
                          (q2.x * q2.x + q2.y * q2.y + q2.z * q2.z) +
                          (q3.x * q3.x + q3.y * q3.y + q3.z * q3.z);
}


void TransformPointLocally(Matrix m, double x_in, double y_in, double z_in, double &x_out,double &y_out,double &z_out, double scale)
{
  ColumnVector d(4);
  d << x_in / scale << y_in  / scale << z_in / scale<< 1;
  ColumnVector f = m * d;
//  cout << m << " \n* \n" << d << "\n=\n" << f;
  x_out = f.element(0);
  y_out = f.element(1);
  z_out = f.element(2);

}

void DXFReader::add3dFace(const DL_3dFaceData& data)
{
    double x1 = 0.0, y1 = 0.0, z1 = 0.0, x2 = 0.0, y2 = 0.0, z2 = 0.0, x3 = 0.0, y3 = 0.0, z3 = 0.0, x4 = 0.0, y4 = 0.0, z4 = 0.0;
#ifdef OLD_DXFLIB_VERSION
    TransformPointLocally(m_relpose, data.x1,data.y1,data.z1,x1,y1,z1, m_scale);
    TransformPointLocally(m_relpose, data.x2,data.y2,data.z2,x2,y2,z2, m_scale);
    TransformPointLocally(m_relpose, data.x3,data.y3,data.z3,x3,y3,z3, m_scale);
    TransformPointLocally(m_relpose, data.x4,data.y4,data.z4,x4,y4,z4, m_scale);
#else
    TransformPointLocally(m_relpose, data.x[0],data.y[0],data.z[0],x1,y1,z1, m_scale);
    TransformPointLocally(m_relpose, data.x[1],data.y[1],data.z[1],x2,y2,z2, m_scale);
    TransformPointLocally(m_relpose, data.x[2],data.y[2],data.z[2],x3,y3,z3, m_scale);
    TransformPointLocally(m_relpose, data.x[3],data.y[3],data.z[3],x4,y4,z4, m_scale);
#endif

#ifdef _DEBUG
    /*printf("3DFACE:\n  %f %f %f -> %f, %f, %f\n %f %f %f -> %f, %f, %f\n %f %f %f -> %f, %f, %f\n %f %f %f -> %f, %f, %f\n ",
                        data.x1,data.y1,data.z1,x1,y1,z1,
                        data.x2,data.y2,data.z2,x2,y2,z2,
                        data.x3,data.y3,data.z3,x3,y3,z3,
                        data.x4,data.y4,data.z4,x4,y4,z4);*/
#endif
    double avgDistTo0 = (x1 * x1 + y1 * y1 + z1 * z1) +
                        (x2 * x2 + y2 * y2 + z2 * z2) +
                        (x3 * x3 + y3 * y3 + z3 * z3) +
                        (x4 * x4 + y4 * y4 + z4 * z4);
#ifdef OLD_DXFLIB_VERSION
    m_3dFaceData.push_back(face3dTransformed(avgDistTo0,x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4,data.x1 / m_scale,data.y1/ m_scale,data.z1/ m_scale,
                                                                                                       data.x2/ m_scale,data.y2/ m_scale,data.z2/ m_scale,
                                                                                                       data.x3/ m_scale,data.y3/ m_scale,data.z3/ m_scale,
                                                                                                       data.x4/ m_scale,data.y4/ m_scale,data.z4/ m_scale));
#else
    m_3dFaceData.push_back(face3dTransformed(avgDistTo0,x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4,data.x[0]/ m_scale,data.y[0]/ m_scale,data.z[0]/ m_scale,
                                                                                                       data.x[1]/ m_scale,data.y[1]/ m_scale,data.z[1]/ m_scale,
                                                                                                       data.x[2]/ m_scale,data.y[2]/ m_scale,data.z[2]/ m_scale,
                                                                                                       data.x[3]/ m_scale,data.y[3]/ m_scale,data.z[3]/ m_scale));
#endif
}
void DXFReader::addPolyline(const DL_PolylineData& data)
{
    double x1 = 0.0, y1 = 0.0, z1 = 0.0, x2 = 0.0, y2 = 0.0, z2 = 0.0, x3 = 0.0, y3 = 0.0, z3 = 0.0, x4 = 0.0, y4 = 0.0, z4 = 0.0;
    double avgDistTo0 = 0.0;
    m_3dFaceData.push_back(face3dTransformed(avgDistTo0,x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4,1 / m_scale,1/ m_scale,1/ m_scale,
                                                                                                       1 / m_scale,1/ m_scale,1/ m_scale,
                                                                                                       1 / m_scale,1/ m_scale,1/ m_scale,
                                                                                                       1 / m_scale,1/ m_scale,1/ m_scale));

}

void DXFReader::addVertex(const DL_VertexData& data)
{
    double x1 = 0.0, y1 = 0.0, z1 = 0.0;
    face3dTransformed& face = m_3dFaceData.back();
    TransformPointLocally(m_relpose, data.x,data.y,data.z,x1,y1,z1, m_scale);
    int n = 8;
    if(face.m_distToZ == 0.0 || face.m_distToZ == 2.0 || face.m_distToZ == 4.0)
    {
        n = (int)face.m_distToZ;
    }
    switch(n )
    {
        case 0:
            face.m_x1 = x1;
            face.m_y1 = y1;
            face.m_z1 = z1;
            face.m_ox1 = data.x / m_scale;
            face.m_oy1 = data.y  / m_scale;
            face.m_oz1 = data.z / m_scale;
            face.m_distToZ = 2.0;
            break;
        case 2:
            face.m_x2 = x1;
            face.m_y2 = y1;
            face.m_z2 = z1;
            face.m_ox2 = data.x / m_scale;
            face.m_oy2 = data.y / m_scale;
            face.m_oz2 = data.z / m_scale;
            face.m_distToZ  = 4.0;
            break;
        case 4:
            face.m_x3 = x1;
            face.m_y3 = y1;
            face.m_z3 = z1;
            face.m_ox3 = data.x / m_scale;
            face.m_oy3 = data.y / m_scale;
            face.m_oz3 = data.z / m_scale;
            face.m_x4 = x1;
            face.m_y4 = y1;
            face.m_z4 = z1;
            face.m_ox4 = data.x / m_scale;
            face.m_oy4 = data.y / m_scale;
            face.m_oz4 = data.z / m_scale;
            face.m_distToZ = (face.m_x1 * face.m_x1 + face.m_y1 * face.m_y1 + face.m_z1 * face.m_z1) +
                                (face.m_x2 * face.m_x2 + face.m_y2 * face.m_y2 + face.m_z2 * face.m_z2) +
                                (face.m_x3 * face.m_x3 + face.m_y3 * face.m_y3 + face.m_z3 * face.m_z3) +
                                (face.m_x4 * face.m_x4 + face.m_y4 * face.m_y4 + face.m_z4 * face.m_z4);
            break;
        default:
            face.m_x4 = x1;
            face.m_y4 = y1;
            face.m_z4 = z1;
            face.m_ox4 = data.x / m_scale;
            face.m_oy4 = data.y / m_scale;
            face.m_oz4 = data.z / m_scale;
            face.m_distToZ = (face.m_x1 * face.m_x1 + face.m_y1 * face.m_y1 + face.m_z1 * face.m_z1) +
                                (face.m_x2 * face.m_x2 + face.m_y2 * face.m_y2 + face.m_z2 * face.m_z2) +
                                (face.m_x3 * face.m_x3 + face.m_y3 * face.m_y3 + face.m_z3 * face.m_z3) +
                                (face.m_x4 * face.m_x4 + face.m_y4 * face.m_y4 + face.m_z4 * face.m_z4);

            m_3dFaceData.push_back(face3dTransformed(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,00,0,0,0,0,0,0,0,0));
            break;
    }
    return;
}

#include <iostream>
#include <string>
#include <MeshProcessing.h>

using namespace std;


vector<double> Tokenize(const string& str,const string& delimiters)
{
 vector<double> tokens;
  string::size_type delimPos = 0, tokenPos = 0, pos = 0;

   if(str.length()<1)  return tokens;
   while(1)
   {
     delimPos = str.find_first_of(delimiters, pos);
     tokenPos = str.find_first_not_of(delimiters, pos);

     if(string::npos != delimPos)
     {
       if(string::npos != tokenPos)
       {
         if(tokenPos<delimPos)
         {
          tokens.push_back(atof(str.substr(pos,delimPos-pos).c_str()));
         }
       }
       pos = delimPos+1;
     }
     else
     {
       if(string::npos != tokenPos)
       {
           tokens.push_back(atof(str.substr(pos).c_str() ));
       }
       break;
     }
   }
   return tokens;
}


vector<int> TokenizeInt(const string& str,const string& delimiters)
{
 vector<int> tokens;
   string::size_type delimPos = 0, tokenPos = 0, pos = 0;

      if(str.length()<1)  return tokens;
      while(1)
      {
         delimPos = str.find_first_of(delimiters, pos);
         tokenPos = str.find_first_not_of(delimiters, pos);
         if(string::npos != delimPos)
         {
            if(string::npos != tokenPos)
            {
               if(tokenPos<delimPos)
               {
                  tokens.push_back(atoi(str.substr(pos,delimPos-pos).c_str()));
               }
            }
            pos = delimPos+1;
          }
          else
          {
            if(string::npos != tokenPos)
            {
             tokens.push_back(atoi(str.substr(pos).c_str()));
            }
            break;
         }
      }
    return tokens;
  }


#ifndef WIN32
/*Mesh_t ReadDae(std::string filename)
{

  DAE dae;
  Mesh_t mesh_final;
  printf("Trying to read: %s\n", filename.c_str());
  daeElement* root = dae.open(filename.c_str());
  if (!root) {
      std::cout << "Document import failed.\n";
      return mesh_final;
  }


  daeElement* node = root->getDescendant("library_geometries");
  if(node == NULL)
  {
    printf("no library geometry\n");
    return mesh_final;
  }
  daeTArray<daeSmartRef<daeElement> >  meshes = node->getChildren();

  for(unsigned int k = 0; k <  meshes.getCount(); k++)
  {
    if(strcmp(meshes[k]->getElementName(), "geometry") != 0)
    {
      printf("%s\n", meshes[k]->getElementName());
      continue;
    }

    domGeometry* geomElement = (domGeometry*)(domElement*)meshes[k];

    domMesh *meshEl = geomElement->getMesh();

    unsigned int countPtriangles = meshEl->getTriangles_array().getCount();
    if(meshEl->getSource_array().getCount() == 0)
      continue;

    domSource *source = meshEl->getSource_array()[0];

    if(source->getFloat_array() == NULL)
      continue;

    domFloat_array& floatArray = source->getFloat_array()[0];
    for(unsigned int j = 0; j < countPtriangles; j++)
    {
      domTriangles* polyArray = meshEl->getTriangles_array()[j];
    //  floatArray->getValue()[i];*/
  /*     domP *poly = polyArray->getP();
       unsigned int count_poly = poly->getValue().getCount();
       unsigned int count_floats =  floatArray.getValue().getCount();
      for(unsigned int i = 0; i < count_poly / 9; i++)
      {
         pair<Polygon_t, Polygon_t> p1;
         Point3d_t point1, point2, point3;
         if(i * 9 + 6 >= count_poly)
           continue;

         domUint indexp1 = 3 * poly->getValue()[i * 9    ];
         domUint indexp2 = 3 * poly->getValue()[i * 9 + 3];
         domUint indexp3 = 3 * poly->getValue()[i * 9 + 6];

         if(indexp3 +2 >= count_floats || indexp2 + 2 >= count_floats  || indexp1 + 2 >= count_floats )
         {
           printf("Probelm in DAE file: Usage of wrong floats, skipped\n");
           break;
         }
         point1.x = floatArray.getValue()[indexp1];
         point1.y = floatArray.getValue()[indexp1 + 1];
         point1.z = floatArray.getValue()[indexp1 + 2];
         point2.x = floatArray.getValue()[indexp2    ];
         point2.y = floatArray.getValue()[indexp2  + 1];
         point2.z = floatArray.getValue()[indexp2  + 2];
         point3.x = floatArray.getValue()[indexp3 ];
         point3.y = floatArray.getValue()[indexp3  + 1];
         point3.z = floatArray.getValue()[indexp3  + 2];

         p1.second.push_back(point1);
         p1.second.push_back(point2);
         p1.second.push_back(point3);
         mesh_final.push_back(p1);
      }
    }
    break;
  }
  return mesh_final;

}*/
#endif
Mesh_t ReadMesh(std::string filename, double scale_halcon, Matrix m, int &points)
{
  Mesh_t mesh;
  double scale = 1 / scale_halcon;
  std::string::size_type idx;

  idx = filename.rfind('.');

  if(idx != std::string::npos)
  {
    std::string extension = filename.substr(idx+1);
    if(extension.compare("dxf") == 0 ||extension.compare("DXF") == 0)
    {
#ifdef WIN32
      DXFReader2* di = new DXFReader2(m, scale);
      DL_Dxf* dxf = new DL_Dxf();

      if (!dxf->in(filename, di)) { // if file open failed
        points = 0;
        return mesh;
      }
      points = di->count_points;
      mesh = di->m_3dFaceData;
      delete dxf;
      delete di;
#else
      DXFReader2 di(m, scale);
      DL_Dxf dxf;

      if (!dxf.in(filename, &di)) { // if file open failed
        points = 0;
        return mesh;
      }
      points = di.count_points;
      mesh = di.m_3dFaceData;
#endif
    }
    else if(extension.compare("dae") == 0)
    {
#ifndef WIN32
/*        mesh = ReadDae(filename);
      for(unsigned int i = 0; i < mesh.size(); i++)
      {
        Polygon_t& poly_in = mesh[i].second;
        Polygon_t& poly_out = mesh[i].first;
        poly_out.clear();
        for(unsigned int j = 0; j < poly_in.size(); j++)
        {
          Point3d_t p;
          TransformPointLocally(m, poly_in[j].x, poly_in[j].y, poly_in[j].z, p.x, p.y, p.z, scale);
          poly_out.push_back(p);
        }
      }
    }
#else
        Point3d_t pt_tmp_orig, pt_tmp;
        Polygon_t tmp_pt_list_orig;
        Polygon_t tmp_pt_list;
        Polygon_t point_cloud;
        ifstream infile(filename.c_str());
        string line;
        size_t pos,i,count;

        long  no_of_polys=0; // will hold the no. of faces in the original KML file
        size_t offset = 0;
        while(infile >> line)    {
          pos=line.find("<mesh>");
          if (pos!=-1) {
                       // Getting the VERTICES for the current mesh
                       while (infile >> line) {
                             pos=line.find("count");
                             if (pos!=-1)           {
                                              sscanf(line.c_str(),"count=\"%ld\">%lf",&count,&pt_tmp_orig.x);
                                              infile >> line;
                                              sscanf(line.c_str(),"%lf",&pt_tmp_orig.y);
                                              infile >> line;
                                              sscanf(line.c_str(),"%lf",&pt_tmp_orig.z);
                                              pt_tmp_orig.x /= scale;
                                              pt_tmp_orig.y /= scale;
                                              pt_tmp_orig.z /= scale;
                                              TransformPointLocally(m, pt_tmp_orig.x,pt_tmp_orig.y,pt_tmp_orig.z, pt_tmp.x,pt_tmp.y, pt_tmp.z, 1);
                                              tmp_pt_list_orig.push_back(pt_tmp_orig);
                                              tmp_pt_list.push_back(pt_tmp);
                                              for (i=0;i!=(count/3)-1;i++) {
                                                  infile >> line;
                                                  sscanf(line.c_str(),"%lf",&pt_tmp_orig.x);
                                                  infile >> line;
                                                  sscanf(line.c_str(),"%lf",&pt_tmp_orig.y);
                                                  infile >> line;
                                                  sscanf(line.c_str(),"%lf",&pt_tmp_orig.z);
                                                  pt_tmp_orig.x /= scale;
                                                  pt_tmp_orig.y /= scale;
                                                  pt_tmp_orig.z /= scale;
                                                  TransformPointLocally(m, pt_tmp_orig.x,pt_tmp_orig.y,pt_tmp_orig.z, pt_tmp.x,pt_tmp.y, pt_tmp.z, 1);
                                                  tmp_pt_list_orig.push_back(pt_tmp_orig);
                                                  tmp_pt_list.push_back(pt_tmp);
                                              }
                                              cout << "No of verts = " << tmp_pt_list.size() << "No of pts : " << count << endl;
                                              break;
                             }
                       }

                       while (infile >> line) {
                             pos=line.find("</mesh>");
                             if (pos!=-1) {
                                   cout << "end of mesh" << endl;
                                     break;
                             }
                             pos=line.find("<triangles");        // extracting one of the sets of triangles in the current mesh
                             if (pos!=-1) {

                                // If this mesh is not empty of triangles
                                for (i=0;i!=tmp_pt_list.size();i++)
                                    point_cloud.push_back(tmp_pt_list[i]);
                                tmp_pt_list.erase(tmp_pt_list.begin(),tmp_pt_list.end());  // otherwise the same vertices will get added
                                                                         // to the mesh as many times as there are triangles
                                                                         // in this mesh!

                                while (infile >> line) {
                                      pos=line.find("count");
                                      if (pos!=-1)           {
                                              sscanf(line.c_str(),"count=\"%ld\"",&count);     // no. of triangles in current set
                                              cout << "tri " << count << endl;
                                              break;
                                      }
                                }

                                bool texture_information_present_in_file = true;

                                while (infile >> line) {
                                    pos = line.find("TEXCOORD");
                                    if (pos!=-1)	{
                                        texture_information_present_in_file = true;
                                        break;
                                    }
                                    pos = line.find("<p>");
                                    if (pos != -1)	{
                                        texture_information_present_in_file = false;
                                        break;
                                    }

                                }

                                do {
                                      pos=line.find("<p>");
                                      if (pos!=-1)           {
                                         sscanf(line.c_str(),"<p>%ld",&pos);
                                         Polygon_t triangle, triangle_orig;
                                         triangle.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         //cout << "pos = " << pos << verts[pos] << endl;
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number

                                         infile >> line;
                                         sscanf(line.c_str(),"%ld",&pos);
                                         triangle.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number

                                         infile >> line;
                                         sscanf(line.c_str(),"%ld",&pos);
                                         triangle.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number
                                         mesh.push_back(std::pair< Polygon_t,Polygon_t>(triangle, triangle_orig));
                                         points += 3;
                                         break;
                                      }
                                } while (infile >> line);

                                for (i=0;i!=count-1;i++)  {
                                          Polygon_t tr_tmp,triangle_orig;

                                         infile >> line;
                                         sscanf(line.c_str(),"%ld",&pos);
                                         tr_tmp.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number

                                         infile >> line;
                                         sscanf(line.c_str(),"%ld",&pos);
                                         tr_tmp.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number

                                         infile >> line;
                                         sscanf(line.c_str(),"%ld",&pos);
                                         tr_tmp.push_back(point_cloud[pos + offset]);
                                         triangle_orig.push_back(tmp_pt_list_orig[pos + offset]);
                                         infile >> line;  // useless number for us
                                         if (texture_information_present_in_file == true) {
                                             infile >> line;  } // also a useless number
                                         mesh.push_back(std::pair< Polygon_t,Polygon_t>(tr_tmp, triangle_orig));
                                         points += 3;
                                }
                             }
                       }
          }
      }
      infile.close();*/
    }
#endif /*WIN32*/
  }
  return mesh;
}

#ifdef EXTENSIONPACKAGE
extern "C"
{
#include <Halcon.h>

    Herror Proj(Hproc_handle proc_id, char* filename, double* hommat, double**x, double**y, double**z,double**ox, double**oy, double**oz, INT* matches, double scale)
    {
        Matrix m(4,4);
        m << hommat[0] << hommat[1] << hommat[2] << hommat[3]
          << hommat[4] << hommat[5] << hommat[6] <<  hommat[7]
          << hommat[8] << hommat[9] << hommat[10] << hommat[11]
          << 0 << 0 << 0 <<1;
        INT points = 0;
        Mesh_t vec = ReadMesh(filename, scale, m, points);
        std::sort(vec.begin(), vec.end());
            /* Visibility Test for all points...*/
        HCkP(HAllocLocal(proc_id,points*sizeof(**ox),ox));
        HCkP(HAllocLocal(proc_id,points*sizeof(**oy),oy));
        HCkP(HAllocLocal(proc_id,points*sizeof(**oz),oz));
        HCkP(HAllocLocal(proc_id,points*sizeof(**x),x));
        HCkP(HAllocLocal(proc_id,points*sizeof(**y),y));
        HCkP(HAllocLocal(proc_id,points*sizeof(**z),z));
        int i = 0;
        for(int j = 0; j < vec.size(); j++)
        {
            Polygon_t& temp = vec[j].first;
            Polygon_t& temp_orig = vec[j].second;
            for(int k = 0; k < temp.size(); k++)
            {
              (*ox)[i] = temp_orig[k].x;
              (*oy)[i] = temp_orig[k].y;
              (*oz)[i] = temp_orig[k].z;
              (*x)[i] = temp[k].x;
              (*y)[i] = temp[k].y;
              (*z)[i] = temp[k].z;
              i++;
            }
        }
        *matches = points;
            return H_MSG_OK;
    }
}
#endif
