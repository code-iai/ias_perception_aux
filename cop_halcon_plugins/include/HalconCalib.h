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


#ifndef HALCON_CALIB
#define HALCON_CALIB

#include "cpp/HalconCpp.h"
/*#include "cpp/HIOStream.h"*/



#define STRICT_CALTAB_SEGMENTATION true
#define CALTAB_SEARCH_TIMEOUT_MS 50

using namespace std;
using namespace Halcon;
using namespace cop;

// *** Gaussian Random routine taken from Numerical Recipes ********************

double gaussianRandom(void)
{
  static bool next_gaussian = false;
  static double saved_gaussian_value;

  double fac, rsq, v1, v2;

  if (!next_gaussian)
  {
    do
    {
      v1 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;
      v2 = 2.0 * ((double)rand() / (double)RAND_MAX) - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0 * log(rsq) / rsq);
    saved_gaussian_value = v1 * fac;
    next_gaussian = true;
    return v2 * fac;
  }
  else
  {
    next_gaussian = false;
    return saved_gaussian_value;
  }
}

// *** Function for generating random normal distributed thresholds ************

double getRandomThreshold(double mean, double std_dev, double min, double max)
{
  double threshold;
  while (true)
  {
    threshold = mean + gaussianRandom() * std_dev;
    if (threshold >= min && threshold <= max)
      break;
  }
  return threshold;
}

bool FindCaltab(HTuple& CamPars, HImage img,const char* caltab_name, HTuple& Row,
                HTuple& Col, HTuple& StartPose, CalTab* init_values)//, HTuple*CtabPose)
{

  HRegion   caltab_region;         // calibration plate region
  double ms_elapsed = 0;
  int caltabfirsttry = 0;
  try
  {
    printf("Initially calling: FindCaltab(%s, 7, %d %d)\n", caltab_name,  init_values->m_calThres, init_values->m_holes);
    caltab_region = img.FindCaltab(caltab_name, 7, init_values->m_calThres, init_values->m_holes);
    caltabfirsttry = 5;
    HTuple contour_row, contour_col, convex_row, convex_col;
    contour_row = caltab_region.GetRegionPolygon(5.0, &contour_col);
    convex_row = caltab_region.GetRegionConvex(&convex_col);
    /*double contour_size = (double)contour_row.Num();
    double convex_size = (double)convex_row.Num();*/
    printf("calling: FindMarksAndPose(..., %s, campar, %d, %f, %d, %f, %d, %d)\n",caltab_name, init_values->m_st, init_values->m_steps, init_values->m_mt, init_values->m_alpha, init_values->m_cont, init_values->m_diam);
    Row = img.FindMarksAndPose(caltab_region, caltab_name, CamPars,
                                         init_values->m_st, init_values->m_steps, init_values->m_mt, init_values->m_alpha, init_values->m_cont, init_values->m_diam,
                                         &Col, &StartPose);
    printf("Succes with initial values.\n");
  }
  catch (HException &e)
  {
    printf("Initial Parameters failed with ex: %s\n", e.message);
    cerr << "Detecting calibration plate in image " << " ";
    while (ms_elapsed < CALTAB_SEARCH_TIMEOUT_MS)
    {
      cerr << ".";
      // try to find calibration table (using random threshold)
      bool failed = false;
      long threshold = init_values->m_calThres;
      long marksize = init_values->m_holes;
      if(caltabfirsttry == 0)
      {
        threshold = (long)getRandomThreshold(init_values->m_calThres, 70, 40, 150);
        marksize =  (long)getRandomThreshold(init_values->m_holes, 1, 3, 15);
        try
        {
          caltab_region = img.FindCaltab(caltab_name, 7, threshold, marksize);
          printf("got something with thres %ld and marksize %ld\n", threshold, marksize);
        }
        catch (HException &e)
        {
         /*printf("Exception in FindCaltab: %s .Continue...", e.message);*/
         failed = true;
        }
        ms_elapsed++;
      }
      else
      {
       caltabfirsttry--;
      }
      // test if region is approximatly convex and the contour is straight

      if (STRICT_CALTAB_SEGMENTATION)
      {
        HTuple contour_row, contour_col, convex_row, convex_col;
        contour_row = caltab_region.GetRegionPolygon(5.0, &contour_col);
        convex_row = caltab_region.GetRegionConvex(&convex_col);
        double contour_size = (double)contour_row.Num();
        double convex_size = (double)convex_row.Num();
        if (caltab_region.Convexity() < 0.95 || contour_size > (1.05 * convex_size))
        {
          failed = true;
        }
      }
      // repeat if failed
      if (failed)
      {

        ms_elapsed += 10;
      }
      // try to find marks and poses in calibration table (use random alpha)
      failed = true;
      double alpha;
      double partly = 0.3;
      for (int j=0; j<10 && ms_elapsed < CALTAB_SEARCH_TIMEOUT_MS; j++)
      {
        ms_elapsed++;
        alpha = getRandomThreshold(       init_values->m_alpha, 0.2, 0.7, 0.8);
        long thres = (long) getRandomThreshold(init_values->m_st, 60*partly, 60, 180);
        long steps = (long)getRandomThreshold(init_values->m_steps, 10*partly, 1, 20);
        long min_thres = (long)getRandomThreshold(init_values->m_mt, 10*partly, 1, 30);
        double cont = getRandomThreshold(init_values->m_cont, init_values->m_cont*partly, 2.0, 30.0);
        /*double diam = getRandomThreshold(init_values->m_diam, 100.0, 50.0, 300.0);*/
        double diam = getRandomThreshold(init_values->m_diam, init_values->m_diam*partly, 10, 300);
        try
        {

          Halcon::set_check("~give_error");
          Row = img.FindMarksAndPose(caltab_region, caltab_name, CamPars,
                                         thres, steps, min_thres, alpha, cont, diam,
                                         &Col, &StartPose);
          if(Row.Num() != 49)
            continue;
          Halcon::set_check("give_error");
          printf("Succes with find mark params: %f %ld %ld %ld %f %f\n", alpha, thres, steps, min_thres, cont, diam);
          init_values->m_calThres = threshold;
          init_values->m_holes = marksize;
          init_values->m_st = thres;
          init_values->m_steps = steps;
          init_values->m_mt = min_thres;
          init_values->m_alpha = alpha;
          init_values->m_cont = cont;
          init_values->m_diam = diam;
        }
        catch (HException &e)
        {
          //printf("Exception in FindCaltab: %s\n .Continue...\n", e.message);
          continue;
        }
        failed = false;
        break;
      }

      // repeat if failed
      if (failed)
      {
        ms_elapsed += 10;
        continue;
      }
      // display calibration table, wait for user input, repeat if wanted

      /*if (g_checkplate)
        {
      p_window->Display(img);
      p_window->SetDraw("margin");
      p_window->SetColor("green");
      p_window->Display(caltab_region);
      p_window->SetDraw("fill");
      p_window->SetColor("magenta");
      p_window->DispCircle(Row, Col, (Row*0.0)+2.0);
      // wait for keypress
      cerr << " successfull (threshold = " << threshold << "; alpha = " << alpha << ")!" << endl
           << ">> Press left mouse button to continue, right mouse button to repeat <<" << endl
           << endl;
      HTuple BRow, BCol, BType;
      BRow = p_window->GetMbutton(&BCol, &BType);
      if (BType[0].I() == 4) // right mouse button
      {
        // reinitialize start time
        gettimeofday(&start, 0);
        ms_elapsed = 0;
        cerr << "Detecting calibration plate in image " << idx << " ";
        continue;
      }
      }*/
      break;
    }
    if (ms_elapsed >= CALTAB_SEARCH_TIMEOUT_MS) // failed
    {
        cerr<<"Caltab localization FAILED"<<endl;
        return false;
    }
  }
    return true;
}
#endif
