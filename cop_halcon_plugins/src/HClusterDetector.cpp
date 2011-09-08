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

#include "HClusterDetector.h"
#include "SegmentPrototype.h"
#include "XMLTag.h"
#include "RangeSensor.h"
#include "BoostUtils.h"

#include "RelPoseHTuple.h"

#include <sys/time.h>
#include <boost/date_time/posix_time/posix_time.hpp>


#define XML_ATTRIBUTE_SR4LO "sr_loid"
#define XML_ATTRIBUTE_PTULO "ptu_loid"

#define XML_ATTRIBUTE_TABLENOISE      "table_noise"
#define XML_ATTRIBUTE_OBJECTSPLITSIZE "object_split_size"
#define XML_ATTRIBUTE_TABLEPERCENTAGE "table_percentage"

using namespace cop;

using namespace std;
using namespace ros;
//using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;

#include <cpp/HalconCpp.h>


#ifndef NORM2
#define NORM2(A,B,C) sqrt((A)*(A)+(B)*(B)+(C)*(C))
#endif


inline void add_triple_tuple_to_pcd(const Halcon::HTuple &X, const Halcon::HTuple &Y, const Halcon::HTuple &Z, PointCloud &pcd)
{
  for(int i = 0; i < X.Num(); i++)
  {
    Point32 p;
    p.x = X[i].D();
    p.y = Y[i].D();
    p.z = Z[i].D();
    pcd.points.push_back(p);
  }
}


inline PointCloud triple_tuple_to_pcd(const Halcon::HTuple &X, const Halcon::HTuple &Y, const Halcon::HTuple &Z)
{
  PointCloud ret;
  for(int i = 0; i < X.Num(); i++)
  {
    Point32 p;
    p.x = X[i].D();
    p.y = Y[i].D();
    p.z = Z[i].D();
    ret.points.push_back(p);
  }
  return ret;
}

inline void update_cov_with_pcd(Halcon::HTuple *cov33, const int &index_start, const PointCloud &pcd, double &mean_x, double &mean_y, double &mean_z)
{
  using namespace Halcon;
  Point32 p;
  HTuple Xreg, Yreg, Zreg, mean_x_inner, mean_y_inner, mean_z_inner, x_off, y_off, z_off;

  for(size_t i = 0; i < pcd.points.size(); i++)
  {
    p = pcd.points[i];
    Xreg[i] = p.x;
    Yreg[i] = p.y;
    Zreg[i] = p.z;
  }

  tuple_mean(Xreg, &mean_x_inner);
  tuple_mean(Yreg, &mean_y_inner);
  tuple_mean(Zreg, &mean_z_inner);

  mean_x = mean_x_inner[0].D();
  mean_y = mean_y_inner[0].D();
  mean_z = mean_z_inner[0].D();

  z_off = Zreg - mean_z_inner;
  y_off = Yreg - mean_y_inner;
  x_off = Xreg - mean_x_inner;

  tuple_abs(x_off, &x_off);
  tuple_abs(y_off, &y_off);
  tuple_abs(z_off, &z_off);

  HTuple cov0,cov1, cov2,cov3,cov4,cov5,cov6,cov7,cov8,cov9, x_off_sorted, y_off_sorted, z_off_sorted, indices;
  tuple_sort_index(x_off, &x_off_sorted);
  tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.03).Int(), HTuple(x_off_sorted.Num()*0.97).Int(), &x_off_sorted);

  tuple_sort_index(y_off, &y_off_sorted);
  tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.03).Int(), HTuple(y_off_sorted.Num()*0.97).Int(), &y_off_sorted);

  tuple_sort_index(z_off, &z_off_sorted);
  tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.03).Int(), HTuple(z_off_sorted.Num()*0.97).Int(), &z_off_sorted);

  tuple_concat(x_off_sorted,y_off_sorted, &indices);
  tuple_concat(indices,z_off_sorted, &indices);

  tuple_select(x_off, indices, &x_off);
  tuple_select(y_off, indices, &y_off);
  tuple_select(z_off, indices, &z_off);

  //printf("Length before: %ld, length after: %ld\n", x_off_sorted.Num(), x_off.Num());

  tuple_sum(x_off*x_off, &cov0);
  tuple_sum(y_off*x_off, &cov1);
  tuple_sum(z_off*x_off, &cov2);
  tuple_sum(x_off*y_off, &cov3);
  tuple_sum(y_off*y_off, &cov4);
  tuple_sum(z_off*y_off, &cov5);
  tuple_sum(x_off*z_off, &cov6);
  tuple_sum(y_off*z_off, &cov7);
  tuple_sum(z_off*z_off, &cov8);
  /** compensate for the removed points*/
  (*cov33)[index_start*9 + 0] = sqrt(cov0[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 1] = sqrt(cov1[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 2] = sqrt(cov2[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 3] = sqrt(cov3[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 4] = sqrt(cov4[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 5] = sqrt(cov5[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 6] = sqrt(cov6[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 7] = sqrt(cov7[0].D() / x_off.Num() * 1.1);
  (*cov33)[index_start*9 + 8] = sqrt(cov8[0].D() / x_off.Num() * 1.1);

}




inline double SymmMahalanobisDistance(double mean_1_x, double mean_1_y, double mean_1_z, double cov1_xx, double cov1_xy, double cov1_xz,
                               double cov1_yx, double cov1_yy, double cov1_yz, double cov1_zx, double cov1_zy, double cov1_zz,
                               double mean_2_x, double mean_2_y, double mean_2_z, double cov2_xx, double cov2_xy, double cov2_xz,
                               double cov2_yx, double cov2_yy, double cov2_yz, double cov2_zx, double cov2_zy, double cov2_zz)
{
  double vectempx, vectempy, vectempz, dist1;
  vectempx = ((mean_2_x - mean_1_x) * cov1_xx + (mean_2_y - mean_1_y) * cov1_yx +(mean_2_z - mean_1_z) * cov1_zx);
  vectempy = ((mean_2_x - mean_1_x) * cov1_xy + (mean_2_y - mean_1_y) * cov1_yy +(mean_2_z - mean_1_z) * cov1_zy);
  vectempz = ((mean_2_x - mean_1_x) * cov1_xz + (mean_2_y - mean_1_y) * cov1_yz +(mean_2_z - mean_1_z) * cov1_zz);


  dist1 = sqrt(fabs(vectempx * (mean_2_x - mean_1_x) + vectempy * (mean_2_y - mean_1_y) + vectempz * (mean_2_z - mean_1_z)));

  vectempx = ((mean_1_x - mean_2_x) * cov2_xx + (mean_1_y - mean_2_y) * cov2_yx +(mean_1_z - mean_2_z) * cov2_zx);
  vectempy = ((mean_1_x - mean_2_x) * cov2_xy + (mean_1_y - mean_2_y) * cov2_yy +(mean_1_z - mean_2_z) * cov2_zy);
  vectempz = ((mean_1_x - mean_2_x) * cov2_xz + (mean_1_y - mean_2_y) * cov2_yz +(mean_1_z - mean_2_z) * cov2_zz);

  dist1 = (dist1 + sqrt(fabs(vectempx * (mean_2_x - mean_1_x) + vectempy * (mean_2_y - mean_1_y) + vectempz * (mean_2_z - mean_1_z)))) / 2;
  return dist1;
}

inline double RatioOfCloseMaxima(const Halcon::HTuple& Max1, const Halcon::HTuple& Min1, Halcon::HTuple &SmoothedFunction, double min_object_size_for_split, Halcon::HTuple &num_max)
{
  using namespace Halcon;
  double ret = 0.0;
  HTuple Min, Max, Y1;
  local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
  get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
  //printf("%ld maxima in split dimension\n", Y1.Num());
  if(Y1.Num() > 0)
  {
    double curr = (Min1+((HTuple(Max[0])*(Max1-Min1))/255))[0].D();
    for(int j = 0; j < Y1.Num() - 1; j++)
    {
      double next = (Min1+((HTuple(Max[j+1])*(Max1-Min1))/255))[0].D();
      if(fabs(curr - next) < min_object_size_for_split)
      {
        ret += 1.0;
      }
    }
    ret /= Y1.Num();
  }
  num_max = Y1.Num();
  return ret;
}



double get_value_by_hist_entry(const double &entry, const double &Min1, const double &Max1)
{
  return (Min1+((entry*(Max1-Min1))/255.0));
}

bool test_min(Halcon::HTuple Max, Halcon::HTuple Min, Halcon::HTuple SmoothedFunction, int index1, int index2, double threshold, int first_max, double Min1, double Max1, double &sel_min)
{
  Halcon::HTuple t1,t2, t3;

  get_y_value_funct_1d(SmoothedFunction, Max[index1].D(), "constant", &t1);
  get_y_value_funct_1d(SmoothedFunction, Max[index2].D(), "constant", &t2);

  double temp_max1 = t1[0].D();
  double temp_max2 = t2[0].D();

  int i_sel = Min.Num();
  double min = 1.0;
  for(int i = 0; i < Min.Num(); i++)
  {
    if(Max[index1].D() <= Min[i].D() && Min[i].D() <= Max[index2].D())
    {
      get_y_value_funct_1d(SmoothedFunction, Min[i].D(), "constant", &t3);
      if(min > t3[0].D())
      {
        min =  t3[0].D();
        i_sel = i;
      }
    }
  }

  while(index2 - index1 >= 1 && i_sel < Min.Num())
  {
    double temp_min;
    get_y_value_funct_1d(SmoothedFunction, Min[i_sel].D(), "constant", &t3);
    temp_min =  t3[0].D();
    if(temp_min < temp_max1 * threshold && temp_min < temp_max2 * threshold)
    {
      sel_min = get_value_by_hist_entry(Min[i_sel].D(), Min1, Max1);
      return true;
    }
    index1++;
  }

  return false;
}

inline bool splitDim(Halcon::Hobject Image, Halcon::Hobject Region, const Halcon::HTuple &Mean, double min_object_size_for_split, Halcon::HTuple &split_x, double threshold)
{
  using namespace Halcon;
  bool ret = false;
  HTuple histo_abs, histo, Min1, Max1, Range, Min, Max, Function, SmoothedFunction, Y1,Y2, num_max;
  int count = 5;
  int first_max = 0, index1, index2;
  gray_histo(Region, Image, &histo_abs, &histo);
  min_max_gray(Region, Image, 0, &Min1, &Max1, &Range);
  create_funct_1d_array(histo, &Function);
  smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);

  while((RatioOfCloseMaxima(Max1, Min1, SmoothedFunction, min_object_size_for_split, num_max)) > 0.1 && count > 0)
  {
    /*temp_result = RatioOfCloseMaxima(Max1, Min1, SmoothedFunction, min_object_size_for_split, num_max);*/
    /*printf("More Smoothing (%d Maxima, %f ratio) \n", num_max[0].I(), temp_result);*/
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    count--;
  }


  local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
  get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);
  if(Min.Num() > 0 )
  {
    get_y_value_funct_1d(SmoothedFunction, Min, "constant", &Y2);


  /**between two max there must be a min
     => either it starts with max or with min,
     and then it iterates between min and max... */
    if(Y1.Num() > Y2.Num())
      first_max = 0;
    else if (Y1.Num() < Y2.Num())
      first_max = 1;
    else
    {
      double firstmax = get_value_by_hist_entry(Max[0].D(), Min1[0].D(), Max1[0].D());
      double firstmin = get_value_by_hist_entry(Min[0].D(), Min1[0].D(), Max1[0].D());
      if(firstmax > firstmin)
        first_max = 1;
      else
        first_max = 0;
    }
  }


  if(Y1.Num() > 2)
  {
    index1 = 0;
    for(int j = 0; j < Y1.Num() - 1; j++)
    {
      index2 = j + 1;
      double decision;
      bool test_min_d = test_min(Max, Min, SmoothedFunction, index1, index2, threshold, first_max, Min1[0].D(), Max1[0].D(), decision);
      if(test_min_d)
      {
        /*printf("Add Split\n");*/
        split_x.Append(decision);
        ret = true;
        index1 = index2;
        printf("Additional split at: %f\n", decision);
        printf("split_x.Num()= %ld\n", split_x.Num());
      }
    }
  }
  return ret;
}


void extract_table_height (Halcon::Hobject &Region,  Halcon::Hobject &ImageZ, Halcon::HTuple *table_height)
{
  using namespace Halcon;
  HTuple AbsoluteHisto, RelativeHisto, Min1, Max1,
         Range, Function, SmoothedFunction, Min, Max, Y1;
  //

  //threshold (ImageZ, Region, -0.3, 2.0)
  gray_histo(Region, ImageZ, &AbsoluteHisto, &RelativeHisto);
  min_max_gray(Region, ImageZ, 0, &Min1, &Max1, &Range);
  //printf("Min-max: %f - %f\n",Min1[0].D(), Max1[0].D());
  create_funct_1d_array(RelativeHisto, &Function);
  smooth_funct_1d_gauss(Function, 2, &SmoothedFunction);
  local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
  int count_smooth = 0;
  while(Max.Num() >  5 && count_smooth < 10)
  {
    smooth_funct_1d_gauss(SmoothedFunction, 2, &SmoothedFunction);
    local_min_max_funct_1d(SmoothedFunction, "strict_min_max", "true", &Min, &Max);
    count_smooth++;
  }
  get_y_value_funct_1d(SmoothedFunction, Max, "constant", &Y1);

  (*table_height) = HTuple();
  HTuple max_y1;
  tuple_sort_index(Y1, &max_y1);
  (*table_height) = (Min1+((HTuple(Max[ max_y1[max_y1.Num() - 1].I() ])*(Max1-Min1))/255));

  {
    double weight = Y1[max_y1[ max_y1.Num() - 1].I()] .D();
    int i = 2;

    while(Y1.Num() >= i && (weight - Y1[max_y1[max_y1.Num() - i].I()].D() < (weight/4.0) || (*table_height)[0].D() > 1.5 || (*table_height)[0].D() < 0.1) )
    {
      if(fabs((*table_height)[0].D() - 0.9) >
         fabs(  (Min1+((HTuple(Max[ max_y1[max_y1.Num() - i].I() ])*(Max1-Min1))/255))[0].D() -0.9))
      {
        (*table_height) = (Min1+((HTuple(Max[ max_y1[max_y1.Num() - i].I() ])*(Max1-Min1))/255));
        weight =  Y1[max_y1[max_y1.Num() - i].I()].D();
      }
      i++;
    }
  }

  printf("table height estimated at z=%f\n", (*table_height)[0].D());
  if((*table_height)[0].D() > 1.5 || (*table_height)[0].D() < 0.0)
  {
     printf("Ignored\n");
     return;
  }
}

void extract_clusters (Halcon::HTuple HomMat3d, Halcon::HTuple X, Halcon::HTuple Y,
    Halcon::HTuple Z, Halcon::HTuple CamParam, Halcon::HTuple *table_height, Halcon::HTuple *Mean_X,
    Halcon::HTuple *Mean_Y, Halcon::HTuple *Mean_Z, Halcon::HTuple *cov33, std::vector<sensor_msgs::PointCloud> &pcds,
    sensor_msgs::PointCloud &table_pcd,
    double table_noise = 0.01, double min_object_size_for_split = 0.05, double min_table_percentage = 0.05)
{
  using namespace Halcon;
  //printf("inside \n");
  // Local iconic variables
  Hobject  ImageZ, ImageX, ImageY, Region;
  Hobject  Region1, RegionDifference1, RegionMorph, ConnectedRegions;
  Hobject  SelectedRegions, ObjectSelected;


  // Local control variables
  HTuple  Qxt, Qyt, Qzt, Qxo, Qyo, Qzo,
  Qx, Qy, Qz, Row, Column, AbsoluteHisto, SizeCluster;
  HTuple  RelativeHisto, Min1, Max1, Range, Function, SmoothedFunction;
  HTuple  Min, Max, Y1, Index, i, thres, start, Or, Number;
  HTuple  index, Rows, Columns, Xreg, Yreg, Zreg, MeanX, MeanY;
  HTuple  MeanZ, Deviation, DeviationY, DeviationX;
  HTuple  Row_add, Row_sgn, Index_R, Col_add, Col_sgn, Index_C;
  //printf("Length of X: %ld\n", X.Num());
  boost::system_time t0;
  boost::system_time t1;
  boost::posix_time::time_duration td;

  t0 = boost::get_system_time();


  affine_trans_point_3d(HomMat3d, X, Y, Z, &Qxo, &Qyo, &Qzo);

  printf("CamParam: %f %f %f %f %f %f\n", CamParam[0].D(),CamParam[1].D(),CamParam[2].D(),CamParam[3].D(),CamParam[4].D(),CamParam[5].D());
  try
  {
    HTuple zeros;
    tuple_find(Z, HTuple(0.0), &zeros);
    printf("Numpoints %ld\n", Z.Num());
    if(!(zeros.Num() == 1 && zeros[0].I() == -1))
    {
      printf("Found %ld zeros\n", zeros.Num());
      for(int i = 0; i  < zeros.Num(); i++)
      {
        Z[zeros[i].I()] = 0.0000001;
      }
    }
    project_3d_point(X, Y, Z, CamParam, &Row, &Column);
  }
  catch(HException ex)
  {
    printf("Fix failed: %s but s_x= %f s_y =%f\n", ex.message, CamParam[2].D(), CamParam[3].D()  );
  }

  gen_image_const(&ImageZ, "float", CamParam[6].I(), CamParam[7].I());
  gen_image_const(&ImageX, "float", CamParam[6].I(), CamParam[7].I());
  gen_image_const(&ImageY, "float", CamParam[6].I(), CamParam[7].I());

  t1 = boost::get_system_time();
  td = t1 - t0;
  printf("Calc time for trans + project: %s\n", boost::posix_time::to_simple_string(td).c_str());
  t0 = boost::get_system_time();

  HTuple runs(Row.Num(), 0), cb(Row.Num(), 0), ce(Row.Num(), 0),t,w,h;
  HTuple px, py, pz;
  float* pfx, *pfy, *pfz;
  int count_runs = 0;
  runs[0] = Row[0];
  cb[0] = Column[0];
  ce[0] = Column[0];
  get_image_pointer1(ImageX, &px, &t, &w, &h);
  get_image_pointer1(ImageY, &py, &t, &w, &h);
  get_image_pointer1(ImageZ, &pz, &t, &w, &h);
  pfx = (float*)px[0].L();
  pfy = (float*)py[0].L();
  pfz = (float*)pz[0].L();

  for(int r_ind = 1; r_ind  < Row.Num(); r_ind++)
  {
      if(Row[r_ind].I() >= 0 && Column[r_ind].I() >= 0 &&
         Row[r_ind].I() < CamParam[7].I() && Column[r_ind].I() < CamParam[6].I())
      {
        if(ce[count_runs] != Column[r_ind])
        {
          count_runs++;
          runs[count_runs] = Row[r_ind];
          cb[count_runs] = Column[r_ind];
          ce[count_runs] = Column[r_ind];
        }
        else
          ce[count_runs] = Column[r_ind];

        int index_image = Row[r_ind].I()* CamParam[6].I() + Column[r_ind].I();
        pfx[index_image] = (float)Qxo[r_ind].D();
        pfy[index_image] = (float)Qyo[r_ind].D();
        pfz[index_image] = (float)Qzo[r_ind].D();
      }
  }
  tuple_select_range(runs, 0, count_runs , &runs);
  tuple_select_range(cb, 0, count_runs , &cb);
  tuple_select_range(ce, 0, count_runs , &ce);
  gen_region_runs(&Region, runs, cb, ce);
  t1 = boost::get_system_time();
  td = t1 - t0;
  printf("Calc time to create 3D image in a new way: %s\n", boost::posix_time::to_simple_string(td).c_str());

  t0 = boost::get_system_time();

  //
  extract_table_height(Region, ImageZ, table_height);

  (*Mean_X) = HTuple();
  (*Mean_Y) = HTuple();
  (*Mean_Z) = HTuple();
  (*cov33) = HTuple();

  t1 = boost::get_system_time();
  td = t1 - t0;
  printf("Calc time table height: %s\n", boost::posix_time::to_simple_string(td).c_str());
  //printf("number of maxima: %ld\n", (*table_height).Num());
  //
  for (i=0; i<=((*table_height).Num())-1; i+=1)
  {
    if (i==(((*table_height).Num())-1))
    {
      thres = 2.5;
    }
    else
    {
      thres = HTuple((*table_height)[i+1])-(0.1*HTuple((*table_height)[i+1]));
    }
    start = HTuple((*table_height)[i])+table_noise;
    threshold(ImageZ, &Region1, start, thres);

    /** Extract table points*/
    Hobject RegionTable;
    HTuple rowt, colt, xt, yt, zt;
    threshold(ImageZ, &RegionTable,  HTuple((*table_height)[i])-table_noise, HTuple((*table_height)[i])+table_noise -0.001);
    get_region_points(RegionTable, &rowt, &colt);
    get_grayval(ImageX, rowt, colt, &xt);
    get_grayval(ImageY, rowt, colt, &yt);
    get_grayval(ImageZ, rowt, colt, &zt);
    table_pcd = triple_tuple_to_pcd(xt, yt, zt);

    intersection(Region1, Region, &RegionDifference1);
    closing_circle(RegionDifference1, &RegionMorph, 2);
    connection(RegionMorph, &ConnectedRegions);
    intersection(ConnectedRegions, RegionDifference1, &ConnectedRegions);
    write_image(ImageZ, "tiff", 0, "testimage_bla.tiff");
    write_region(ConnectedRegions, "testregion_bla.reg");

    select_shape(ConnectedRegions, &SelectedRegions, "area", "and", 50, 1999999);
    {
      count_obj(SelectedRegions, &Number);
      printf("\n\n%d regions to search for clusters\n\n", Number[0].I());
      for (index=1; index<=Number[0].I(); index+=1)
      {
        HTuple ZregOuter, RowsZ, ColumnsZ, MeanZOut, split_z;
        t0 = boost::get_system_time();

        //printf("enter region %d\n", index[0].I());
        select_obj(SelectedRegions, &ObjectSelected, index);
        get_region_points(ObjectSelected, &RowsZ, &ColumnsZ);

        get_grayval(ImageZ, RowsZ, ColumnsZ, &ZregOuter);
        if(ZregOuter.Num() < 2)
        {
          continue;
        }
        tuple_mean(ZregOuter, &MeanZOut);
        if(ZregOuter.Num() > X.Num() / (CamParam[6].D()*2))
        {
          printf("Trying split in Z\n");
          splitDim(ImageZ, ObjectSelected, MeanZOut, min_object_size_for_split, split_z, 0.4);
        }
        double lower_bound = -100000.0;
        double upper_bound = -100000.0;

        for(int zsplit = 0; zsplit < split_z.Num() + 1; zsplit++)
        {
          HTuple RowY, ColY;
          printf("Processing splitter %d/%ld in Z\n", zsplit, split_z.Num() + 1);
          lower_bound = upper_bound;
          if(zsplit < split_z.Num())
            upper_bound = split_z[zsplit];
          else
            upper_bound = 100000.0;
          Rows = HTuple();
          Columns = HTuple();
          for (int rowsz = 0; rowsz < RowsZ.Num(); rowsz++)
          {
            if( lower_bound < ZregOuter[rowsz].D() &&  ZregOuter[rowsz].D() <  upper_bound && ZregOuter[rowsz].D() != 0.0)
            {
              Rows.Append(RowsZ[rowsz]);
              Columns.Append(ColumnsZ[rowsz]);
            }
          }


         printf("back to normal\n");
          get_grayval(ImageY, Rows, Columns, &Yreg);
          if(Yreg.Num() < 2)
          {
            printf("Miniregion, skip\n");
            continue;
          }

          tuple_mean(Yreg, &MeanY);

          double cov[9];
          memset(cov, 0.0, sizeof(*cov)*9);

          HTuple max, histo, histo_abs, split_y;

          tuple_max(Yreg - MeanY, &max);
          bool splity = false;
          if(max > min_object_size_for_split && Yreg.Num() > X.Num() / (CamParam[6].D()*2))
          {
            printf("Trying split in Y\n");
            splity = splitDim(ImageY, ObjectSelected, MeanY, min_object_size_for_split, split_y, 0.4);
          }
          double lower_bound_y = -100000.0;
          double upper_bound_y = -100000.0;
          for(int ysplit = 0; ysplit < split_y.Num() + 1; ysplit++)
          {
            HTuple RowY, ColY;
            printf("Processing splitter %d in Y\n", ysplit);
            lower_bound_y = upper_bound_y;
            if(ysplit < split_y.Num())
              upper_bound_y = split_y[ysplit];
            else
              upper_bound_y = 100000.0;

            for (int rows = 0; rows < Rows.Num(); rows++)
            {
              if( lower_bound_y < Yreg[rows].D() &&  Yreg[rows].D() <  upper_bound_y && Yreg[rows].D() != 0.0)
              {
                RowY.Append(Rows[rows]);
                ColY.Append(Columns[rows]);
              }
            }
            //printf("ysplit %d left %ld / %ld\n", ysplit, RowY.Num(), Rows.Num());
            //printf("decision y: ");
            //printf( "%f < y < %f\n", lower_bound, upper_bound);
            HTuple split_x;
            HTuple x_off, RowXY, ColXY;
            if(RowY.Num() < 2)
            {
              printf("Miniregion, skip (2)\n");
              continue;
            }
            get_grayval(ImageX, RowY, ColY, &Xreg);
            tuple_mean(Xreg, &MeanX);
            tuple_max(Xreg - MeanX, &max);
            bool splitx = false;
            if(max > min_object_size_for_split && Xreg.Num() > X.Num() / (CamParam[6].D()*2))
            {
              printf("Trying to split in x\n");
              splitx = splitDim(ImageX, ObjectSelected, MeanX, min_object_size_for_split, split_x, 0.4);
            }
            double lower_bound_x = -100000.0;
            double upper_bound_x = -100000.0;

            for(int xsplit = 0; xsplit < split_x.Num() + 1; xsplit++)
            {
              printf("Processing %dth Xsplitter\n", xsplit);
              RowXY = HTuple();
              ColXY = HTuple();
              lower_bound_x = upper_bound_x;
              if(xsplit < split_x.Num())
                upper_bound_x = split_x[xsplit];
              else
                upper_bound_x = 100000.0;
              for (int rows = 0; rows < RowY.Num(); rows++)
              {
                if(lower_bound_x < Xreg[rows].D() && Xreg[rows].D() < upper_bound_x && Xreg[rows].D() != 0.0)
                {
                  RowXY.Append(RowY[rows]);
                  ColXY.Append(ColY[rows]);
                }
              }

              //printf("xsplit: %d () xsplit: %d size: %ld / %ld\n", xsplit, xsplit, RowXY.Num(),  RowY.Num());
              //printf("decision x: ");
              //printf( "%f < x < %f\n",lower_bound_x,upper_bound_x);

              HTuple YregInner, XregInner, z_off, x_off, y_off, mean_x_inner, mean_y_inner;

              if(RowXY.Num() < 2)
              {
                printf("Miniregion, skip (3)\n");
                continue;
              }
              get_grayval(ImageX, RowXY, ColXY, &XregInner);
              get_grayval(ImageY, RowXY, ColXY, &YregInner);
              get_grayval(ImageZ, RowXY, ColXY, &Zreg);

              tuple_mean(XregInner, &mean_x_inner);
              tuple_mean(YregInner, &mean_y_inner);
              tuple_mean(Zreg, &MeanZ);


              z_off = Zreg - MeanZ;
              y_off = YregInner - mean_y_inner;
              x_off = XregInner - mean_x_inner;
              tuple_abs(x_off, &x_off);
              tuple_abs(y_off, &y_off);
              tuple_abs(z_off, &z_off);
              HTuple cov0,cov1, cov2,cov3,cov4,cov5,cov6,cov7,cov8,cov9, x_off_sorted, y_off_sorted, z_off_sorted, indices;
              tuple_sort_index(x_off, &x_off_sorted);
              tuple_select_range(x_off_sorted, HTuple(x_off_sorted.Num()*0.03).Int(), HTuple(x_off_sorted.Num()*0.97).Int(), &x_off_sorted);

              tuple_sort_index(y_off, &y_off_sorted);
              tuple_select_range(y_off_sorted, HTuple(y_off_sorted.Num()*0.03).Int(), HTuple(y_off_sorted.Num()*0.97).Int(), &y_off_sorted);

              tuple_sort_index(z_off, &z_off_sorted);
              tuple_select_range(z_off_sorted, HTuple(z_off_sorted.Num()*0.03).Int(), HTuple(z_off_sorted.Num()*0.97).Int(), &z_off_sorted);

              tuple_concat(x_off_sorted,y_off_sorted, &indices);
              tuple_concat(indices,z_off_sorted, &indices);

              tuple_select(x_off, indices, &x_off);
              tuple_select(y_off, indices, &y_off);
              tuple_select(z_off, indices, &z_off);

              //printf("Length before: %ld, length after: %ld\n", x_off_sorted.Num(), x_off.Num());

              tuple_sum(x_off*x_off, &cov0);
              tuple_sum(y_off*x_off, &cov1);
              tuple_sum(z_off*x_off, &cov2);
              tuple_sum(x_off*y_off, &cov3);
              tuple_sum(y_off*y_off, &cov4);
              tuple_sum(z_off*y_off, &cov5);
              tuple_sum(x_off*z_off, &cov6);
              tuple_sum(y_off*z_off, &cov7);
              tuple_sum(z_off*z_off, &cov8);
              /*compensate for the removed extreme points*/
              cov0 = sqrt(cov0[0].D() / x_off.Num() * 1.1);
              cov1 = sqrt(cov1[0].D() / x_off.Num() * 1.1);
              cov2 = sqrt(cov2[0].D() / x_off.Num() * 1.1);
              cov3 = sqrt(cov3[0].D() / x_off.Num() * 1.1);
              cov4 = sqrt(cov4[0].D() / x_off.Num() * 1.1);
              cov5 = sqrt(cov5[0].D() / x_off.Num() * 1.1);
              cov6 = sqrt(cov6[0].D() / x_off.Num() * 1.1);
              cov7 = sqrt(cov7[0].D() / x_off.Num() * 1.1);
              cov8 = sqrt(cov8[0].D() / x_off.Num() * 1.1);

              /** Test if there is already a similar cluster
                  This can happen if the axis aligned split cut an object in half
              */
              bool rejected = false;
              for(int test_near = 0; test_near < (*Mean_X).Num(); test_near++)
              {

                double malha_dist = SymmMahalanobisDistance((*Mean_X)[test_near].D(), (*Mean_Y)[test_near].D() , (*Mean_Z)[test_near].D() , (*cov33)[test_near*9].D(),
                                                  (*cov33)[test_near*9 + 1].D(), (*cov33)[test_near*9 + 2].D(), (*cov33)[test_near*9 + 3].D(), (*cov33)[test_near*9 + 4 ].D(), (*cov33)[test_near*9 + 5].D(),
                                                  (*cov33)[test_near*9 + 6].D(), (*cov33)[test_near*9 + 7].D(), (*cov33)[test_near*9 + 8].D(),
                                                  mean_x_inner[0].D(),  mean_y_inner[0].D(),  MeanZ[0].D(), cov0[0].D(), cov1[0].D(), cov2[0].D(), cov3[0].D(), cov4[0].D(),
                                                  cov5[0].D(), cov6[0].D(), cov7[0].D(), cov8[0].D());

               double dist = NORM2(((*Mean_X)[test_near].D() - mean_x_inner[0].D()),
                                  ((*Mean_Y)[test_near].D() - mean_y_inner[0].D()),
                                  ((*Mean_Z)[test_near].D() - MeanZ[0].D()));
               /*printf("distance between two Cluster mahl_dist: %f dist: %f\n", malha_dist, dist);
               2*min_object_size_for_split
               */

                if(( malha_dist < 0.01) || dist < min_object_size_for_split)
                {
                  SizeCluster[test_near] = SizeCluster[test_near].I() + x_off.Num();

                  add_triple_tuple_to_pcd(XregInner, YregInner, Zreg, pcds[test_near]);
                  double meanx, meany, meanz;
                  update_cov_with_pcd(cov33, test_near, pcds[test_near], meanx, meany, meanz);
                  (*Mean_X)[test_near] = meanx;
                  (*Mean_Y)[test_near] = meany;
                  (*Mean_Z)[test_near] = meanz;
                  printf("Fused two clusters, which were splitted before\n");
                  rejected = true;
                  break;
                }
                /*printf("Accept distance %f between %d and new mean (%f %f %f)\n", NORM2(((*Mean_X)[test_near].D() - mean_x_inner[0].D()),
                                     ((*Mean_Y)[test_near].D() - mean_y_inner[0].D()),
                                                         ((*Mean_Z)[test_near].D() - MeanZ[0].D())),
                                                         test_near, mean_x_inner[0].D(),
                                                         mean_y_inner[0].D(),  MeanZ[0].D());*/
              }



              if(rejected)
              {
                printf("remove after fuse\n");
                continue;
              }
              /** density in particle per cubic meter*/
              double dens_threshold = 5000000, density;


              density = XregInner.Num() / (cov0 * cov4 * cov8)[0].D();
              printf("density is %f (thres: %f)(covs: %f %f %f., Num: %ld) (mean: %f %f %f)\n", density, dens_threshold, cov0[0].D(), cov4[0].D(), cov8[0].D(), x_off.Num(), mean_x_inner[0].D(), mean_y_inner[0].D(), MeanZ[0].D());
              if(density < dens_threshold)
              {
                printf("Rejecting Cluster due low density\n");
                continue;
              }
              bool flatness = (cov8 * 2.0 < table_noise) && ((MeanZ[0].D() - (*table_height)[i].D()) < table_noise*2.0);
              if(flatness)
              {
                printf("Too Flat: %f < %f and %f < %f\n", cov8[0].D() * 2.0 , table_noise, (MeanZ[0].D() - (*table_height)[i].D()),  table_noise*2.0);
                continue;
              }
              printf("Added a Cluster.\n");
              cov33->Append(cov0);
              cov33->Append(cov1);
              cov33->Append(cov2);

              cov33->Append(cov3);
              cov33->Append(cov4);
              cov33->Append(cov5);

              cov33->Append(cov6);
              cov33->Append(cov7);
              cov33->Append(cov8);


              (*Mean_X).Append(mean_x_inner);
              (*Mean_Y).Append(mean_y_inner);
              (*Mean_Z).Append(MeanZ);
              pcds.push_back(triple_tuple_to_pcd(XregInner, YregInner, Zreg));
              SizeCluster.Append(x_off.Num());
            }
          }
          //
          t1 = boost::get_system_time();
          td = t1 - t0;
          printf("Calc time connection: %s\n", boost::posix_time::to_simple_string(td).c_str());
        }
      }
    }
  }
  return;
}


HClusterDetector::HClusterDetector(int srid, int ptuid) :
  m_swissranger_jlo_id(srid),
  m_ptu_jlo_id(ptuid)
{
}

HClusterDetector::HClusterDetector()
{
}

void HClusterDetector::SetData(XMLTag* tag)
{

  std::string camparam = tag->GetProperty(XML_ATTRIBUTE_FILENAME, "");
  if(camparam.length() > 0)
  {
    Halcon::HTuple camparam_h;
    Halcon::read_cam_par(camparam.c_str(), &camparam_h);
    for(int i = 0; i < camparam_h.Num(); i++)
    {
      m_camparams.push_back(camparam_h[i].D());
    }
    //printf("Read camparam: %s\n", camparam.c_str());
 }

  if(tag != NULL)
  {
      m_swissranger_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_SR4LO, 0);
      if(m_swissranger_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_SR4LO, "/sr4");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_swissranger_jlo_id = 1;
        else
        {
          printf("\n\nRead HClusterDetector with %ld as camera position (%s)\n", pose->m_uniqueID, name.c_str());
          m_swissranger_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(&pose);
        }
      }
      m_ptu_jlo_id = tag->GetPropertyInt(XML_ATTRIBUTE_PTULO, 0);
      if(m_ptu_jlo_id == 0)
      {
        std::string name = tag->GetProperty(XML_ATTRIBUTE_PTULO, "/base_link");
        RelPose* pose = RelPoseFactory::GetRelPose(name);
        if(pose == NULL)
          m_ptu_jlo_id = 1;
        else
        {
          m_ptu_jlo_id =pose->m_uniqueID;
          RelPoseFactory::FreeRelPose(&pose);
        }
      }
      m_table_noise = tag->GetPropertyDouble(XML_ATTRIBUTE_TABLENOISE, 0.02);
      m_min_object_size_for_split = tag->GetPropertyDouble(XML_ATTRIBUTE_OBJECTSPLITSIZE, 0.05);
      m_min_table_percentage = tag->GetPropertyDouble(XML_ATTRIBUTE_TABLEPERCENTAGE, 0.05);
  }
}


HClusterDetector::~HClusterDetector()
{

}

std::vector<RelPose*> HClusterDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
  bool nosensor = true;
    //Calibration* calib = &cam[0]->m_calibration;
  SegmentPrototype* proto = (SegmentPrototype*)object.GetElement(0, DESCRIPTOR_SEGMPROTO);
  printf("HClusterDetector::Perform: Got SegmentPrototype\n");
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
    {
      try
      {
        nosensor = false;
        results = Inner(*it, pose, proto, numOfObjects, qualityMeasure);
      }
      catch (const char* text )
      {
         printf("Error in HClusterDetector: %s\n", text);
      }
      break;
    }
  }
  if(nosensor)
  {
    for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
    {
      if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
      {
        try
        {
          nosensor = false;
          results = Inner(*it, pose, proto, numOfObjects, qualityMeasure);
        }
        catch (const char* text )
        {
           printf("Error in HClusterDetector: %s\n", text);
        }
        break;
      }
    }
  }
  /*TODO plane clusters*/
  return results;
}



inline void normalize(double &a,double &b, double &c)
{
  double norm = sqrt(a*a + b*b + c*c);
  a /= norm;
  b /= norm;
  c /= norm;
}

inline double scalarproduct(const double &a,const double &b, const double &c, const double &d, const double &e, const double &f)
{
  return a * d + b* e + c*f;
}

inline void CrossProduct_l(const double b_x, const double b_y,const double b_z,const double c_x,const double c_y,const double c_z,double &a_x,double &a_y,double &a_z)
{
    a_x = b_y*c_z - b_z*c_y;
    a_y = b_z*c_x - b_x*c_z;
    a_z = b_x*c_y - b_y*c_x;
}


std::vector<RelPose*> HClusterDetector::Inner(Sensor* sens, RelPose* initial_pose_estimate, SegmentPrototype* obj_descr, int &numOfObjects, double& qualityMeasure)
{

  boost::system_time t0;
  boost::system_time t1;
  boost::posix_time::time_duration td;

  t0 = boost::get_system_time();

  std::vector<RelPose*> results;
  //printf("Inner\n");
  double threshold = qualityMeasure;
  obj_descr->ClearPointClouds();

  qualityMeasure = 0.0;
  bool parallel = true;
  unsigned long ref_frame = m_ptu_jlo_id;

  if(obj_descr != NULL)
  {
    ref_frame = obj_descr->GetFrameId();
    printf("Ref Frame: %ld\n", ref_frame);
    parallel = obj_descr->m_parallel;
  }

  try
  {
    Halcon::HTuple HomMat3d, HomMat3Dinv,  CamParam(8),  table_height, Mean_X, Mean_Y, Mean_Z, cov33;


    for(size_t i = 0; i < ((RangeSensor*)sens)->m_calibration.size(); i++)
    {
      CamParam[i] = ((RangeSensor*)sens)->m_calibration[i];
    }
    SwissRangerReading* read = (SwissRangerReading*)sens->GetReading();
    if(read == NULL)
      return results;
    RelPose* ptu = RelPoseFactory::FRelPose(ref_frame);
    sensor_msgs::PointCloud& pcd = read->m_image;
    std::vector<sensor_msgs::PointCloud> segment_pcds;
    sensor_msgs::PointCloud table_pcd;
    RelPoseHTuple::GetHommat(sens->GetRelPose(), &HomMat3d, ref_frame);
    printf("Sens to ref frame (%ld -> %ld)\n", sens->GetRelPose()->m_uniqueID, ref_frame);
    hom_mat3d_invert(HomMat3d, &HomMat3Dinv);
    std::vector<std::vector<double> > vec;
    Halcon::HTuple X,  Y,  Z;
    int  counter = 0;
    for(size_t i = 0; i < (pcd.points.size()); i++)
    {
      if( pcd.points[i].z !=  pcd.points[i].z || pcd.points[i].z == 0.0)
        continue;
      X[counter] = pcd.points[i].x;
      Y[counter] = pcd.points[i].y;
      Z[counter] = pcd.points[i].z;
      counter++;
    }
    t1 = boost::get_system_time();
    td = t1 - t0;
    printf("Calc time before extract_cluster: %s\n", boost::posix_time::to_simple_string(td).c_str());
    t0 = boost::get_system_time();

    //printf("Before extract cluster\n");
    extract_clusters(HomMat3d, X, Y, Z, CamParam, &table_height, &Mean_X, &Mean_Y, &Mean_Z, &cov33, segment_pcds,
            table_pcd,
            m_table_noise, m_min_object_size_for_split, m_min_table_percentage);
    t1 = boost::get_system_time();
    td = t1 - t0;
    printf("Calc time in extract_cluster: %s (num results = %ld)\n", boost::posix_time::to_simple_string(td).c_str(), Mean_X.Num());
    t0 = boost::get_system_time();

    for(int x= 0; x < Mean_X.Num(); x++)
    {
        double temp_qual = min(1.0, max(0.0, fabs( 2*cov33[9*x+4].D()*2*cov33[9*x+0].D()*2*cov33[9*x+8].D())));
        bool in_or_out_table = false;
        /*printf("compare table %f with Cluster Z %f (upper thres %f and lower %f)\n", table_height[0].D(), Mean_Z[x].D(),  m_table_noise + (cov33[9*x+8].D()) * 4, m_table_noise);*/
        in_or_out_table = ( Mean_Z[x].D() - table_height[0].D() > 6*m_table_noise + (cov33[9*x+8].D()) * 8)
                               ||
                        ( Mean_Z[x].D() - table_height[0].D() < m_table_noise);
        /*printf("Tabel height relation broken?  %s\n", in_or_out_table ? "true" : "false");*/

        if(Mean_Z[x].D() > 0.30 && Mean_Z[x].D() < 2.5 && !in_or_out_table)
        {
          Matrix rotmat(4,4);
          rotmat << 1 << 0<< 0  << Mean_X[x].D()
                 << 0 << 1 << 0 << Mean_Y[x].D()
                 << 0 << 0<< 1  << Mean_Z[x].D()
                 << 0 << 0 << 0 << 1;

          /*cout <<  "Matrix from plane_clusters:" << endl << rotmat << endl;*/
          Matrix cov (6,6);
          if(parallel)
          {
            cov << (cov33[9*x+0].D()) << (cov33[9*x+1].D()) << (cov33[9*x+2].D()) << 0   << 0   << 0
                << (cov33[9*x+3].D()) << (cov33[9*x+4].D()) << (cov33[9*x+5].D()) <<  0  << 0   << 0
                << (cov33[9*x+6].D()) << (cov33[9*x+7].D()) << (cov33[9*x+8].D()) << 0   << 0   << 0
                <<0    << 0    << 0    <<  ((obj_descr == NULL) ? 0.2 : obj_descr->m_covRotX) << 0   << 0
                <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0
                <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
           }
           else
           {
             cov << (cov33[9*x+3].D()) << (cov33[9*x+4].D()) << (cov33[9*x+5].D()) << 0  << 0   << 0
                << (cov33[9*x+0].D()) << (cov33[9*x+1].D()) << (cov33[9*x+2].D()) << 0  << 0   << 0
                << (cov33[9*x+6].D()) << (cov33[9*x+7].D()) << (cov33[9*x+8].D()) << 0   << 0   << 0
                <<0    << 0    << 0    << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotY) << 0   << 0
                <<0    << 0    << 0    << 0   << ((obj_descr == NULL) ? 0.2 :obj_descr->m_covRotX) << 0
                <<0    << 0    << 0    << 0   << 0   << ((obj_descr == NULL) ? 0.8 :obj_descr->m_covRotZ);
            }
          RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
          if(pose_temp == NULL)
          {
            throw("Error: Failed to create pose\n");
          }
          double corr_prob = initial_pose_estimate->ProbabilityOfCorrespondence(pose_temp);
          if(corr_prob < 0.0)
            corr_prob = 0.0;
          else if(corr_prob > 1.0)
            corr_prob = 1.0;
          pose_temp->m_qualityMeasure = temp_qual*corr_prob;
          ROS_INFO("\nPose corr: %ld   %ld: %f  (thres %f)\n", initial_pose_estimate->m_uniqueID, pose_temp->m_uniqueID, corr_prob, threshold);
          if(corr_prob >= threshold)
          {
            if(results.size() == 0)
              qualityMeasure =temp_qual*corr_prob;
            results.push_back(pose_temp);
            obj_descr->SetPointCloud(pose_temp->m_uniqueID, segment_pcds[x], sens->GetRelPose()->m_uniqueID);
          }
          else
          {
            RelPoseFactory::FreeRelPose(&pose_temp);
            printf("Rejecting pose caused by threshold: %f < %f \n", corr_prob, threshold);
          }
        }
        else
        {
          printf("Rejecting pose  for height  %s (table_height: %f, Z %f cov %f table_noise %f)\n", in_or_out_table ? "true" : "false", table_height[0].D(),  Mean_Z[x].D(), cov33[9*x+8].D(), m_table_noise);
        }
    }
    t1 = boost::get_system_time();
    td = t1 - t0;
    printf("Calc time after extract_cluster: %s\n", boost::posix_time::to_simple_string(td).c_str());
    t0 = boost::get_system_time();
    /**
      Add Table
    */
    if(results.size() > 0 && table_pcd.points.size() > 0)
    {
      Halcon::HTuple covtable;
      double  meanx, meany, meanz;
      update_cov_with_pcd(&covtable, 0, table_pcd, meanx, meany, meanz);
      Matrix rotmat(4,4);
      rotmat << 1 << 0<< 0  << meanx
             << 0 << 1 << 0 << meany
             << 0 << 0<< 1  << meanz
             << 0 << 0 << 0 << 1;

        /*cout <<  "Matrix from plane_clusters:" << endl << rotmat << endl;*/
        Matrix cov (6,6);
        cov << (covtable[0].D())  << (cov33[1].D()) << (cov33[2].D()) << 0   << 0   << 0
           << (covtable[3].D())   << (cov33[4].D()) << (cov33[5].D()) <<  0  << 0   << 0
           << (covtable[6].D())   << (cov33[7].D()) << (cov33[8].D()) << 0   << 0   << 0
           <<0    << 0    << 0    <<   0 << 0   << 0
           <<0    << 0    << 0    << 0   << 0   << 0
           <<0    << 0    << 0    << 0   << 0   << 0;
      RelPose* pose_temp = RelPoseFactory::FRelPose(ptu, rotmat, cov);
      obj_descr->SetTable(pose_temp->m_uniqueID, table_pcd);
      printf("Measured table noise: (%ld) \n", pose_temp->m_uniqueID);
      cout << cov << endl;
    }

    RelPoseFactory::FreeRelPose(&ptu);
    t1 = boost::get_system_time();
    td = t1 - t0;
    printf("Calc time to Add Table: %s\n", boost::posix_time::to_simple_string(td).c_str());
  }
  catch(Halcon::HException ex)
  {
    printf("HClusterExtractor: %s\n", ex.message);
  }


  return results;
}

double HClusterDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
  {
    if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
    {
      if(object.GetElement(0, DESCRIPTOR_SEGMPROTO) != NULL )
        return 0.1;
      else
        return 0.0;
    }
  }
  return 0.0;
}


bool HClusterDetector::TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose)
{
    return false;
}

XMLTag* HClusterDetector::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_HCLUSTERDETECTOR);
    tag->AddProperty(XML_ATTRIBUTE_SR4LO, m_swissranger_jlo_id);
    tag->AddProperty(XML_ATTRIBUTE_PTULO, m_ptu_jlo_id);
    tag->AddProperty(XML_ATTRIBUTE_TABLENOISE, m_table_noise);
    tag->AddProperty(XML_ATTRIBUTE_OBJECTSPLITSIZE, m_min_object_size_for_split);
    tag->AddProperty(XML_ATTRIBUTE_TABLEPERCENTAGE, m_min_table_percentage);
    return tag;
}


