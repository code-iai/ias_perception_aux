#include "cpp/HalconCpp.h"
#include "PanCakeDetector.h"
#include "RelPoseHTuple.h"
using namespace Halcon;
using namespace cop;




// Local procedures
void find_pancake (Halcon::Hobject ImageFull, Halcon::Hobject ModelContours, Halcon::Hobject *SelectedRegions,
    Halcon::HTuple ModelID, Halcon::HTuple CamParamOut, Halcon::HTuple NFinalPose,
    Halcon::HTuple HomMat3D, Halcon::HTuple HomMat2dLast, Halcon::HTuple *Qx, Halcon::HTuple *Qy,
    Halcon::HTuple *Qz, Halcon::HTuple *HomMat2dLastOut, int min_area = 3000, int max_area = 30000, double min_compactness = 0.0, double max_compactness = 3.0);

/**
  Test if the already there are models learned
*/
double PanCakeDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sens)
{
  Descriptor* descr = (Descriptor*)object.GetElement(0,DESCRIPTOR_DEFORMSHAPE);
  if(descr != NULL && descr->GetNodeName().compare(XML_NODE_PANCAKEMAKERMODEL) == 0)
  {
    return 1.0;
  }
  else
    return 0.0;
}

HTuple HomMat2dLast;


    void  PanCakeDetector::SetData(XMLTag* tag)
    {
      printf("Loading PanCakeDetector\n");
    }
    /**
      Action function, prepares images
    */
std::vector<RelPose*> PanCakeDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  printf("Called PanCakeDetector\n");
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Image* img = cam->GetImage(-1);
    RelPose* camPose = cam->m_relPose;
    if(img != NULL && camPose != NULL )
    {
      HTuple camparam = cam->m_calibration.CamParam();
      HTuple cammatrix = cam->m_calibration.CamMatrix();

      if(img->GetType() == ReadingType_HalconImage)
      {          
          PanCakeMaker* dsm = (PanCakeMaker*)(object.GetElement(0, DESCRIPTOR_DEFORMSHAPE ));
          printf("Loaded a PanCakeMaker Model\n");
          if(dsm != NULL)
          {
            //hommatfirst = *dsm->GetHomMatFirst();
            try
            {
              Hobject imgs = *img->GetHImage();
              int handle = dsm->GetDeformShapeHandle("default");
              HTuple pose_in, hom_in, qx, qy, qz, Meanx, Meany, Meanz;
              Hobject modelcont;
              printf("Handle is %d\n", handle);
              if(handle == -1)
              {
                  handle = dsm->GetDeformShapeHandle(cam->GetSensorID());                
              }
              Hobject result_region;
              Halcon::get_deformable_model_contours(&modelcont, handle, 1);

              RelPoseHTuple::GetPose(pose, &pose_in, img->GetPose()->m_uniqueID);
              pose_to_hom_mat3d(pose_in, &hom_in);
              find_pancake(imgs, modelcont, &result_region, handle, camparam, pose_in, hom_in, HomMat2dLast, &qx, &qy, &qz, &HomMat2dLast,
                                            dsm->m_minArea, dsm->m_maxArea, dsm->m_minCompactness, dsm->m_maxCompactness);
              if(qx.Num() > 0 )
              {
                tuple_mean((qx), &Meanx);
                tuple_mean((qy), &Meany);
                tuple_mean((qz), &Meanz);
                HTuple hommat, inv, qx2, qy2, qz2;
                RelPoseHTuple::GetHommat(img->GetPose(), &hommat, 1);
                hom_mat3d_invert(hommat, &inv);
                Matrix m(4,4);
                m.element(0,0) = inv[0].D(); m.element(0,1) = inv[1].D(); m.element(0,2) = inv[2].D(); m.element(0,3) = Meanx[0].D();
                m.element(1,0) = inv[4].D(); m.element(1,1) = inv[5].D(); m.element(1,2) = inv[6].D(); m.element(1,3) = Meany[0].D();
                m.element(2,0) = inv[8].D(); m.element(2,1) = inv[9].D(); m.element(2,2) = inv[10].D();m.element(2,3) = Meanz[0].D();
                m.element(3,0) = 0.0;m.element(3,1) = 0.0;m.element(3,2) = 0.0;m.element(3,3) = 1.0;
                printf("Got a result at %f %f %f\n", Meanx[0].D(), Meany[0].D(), Meanz[0].D());
                Matrix cov(6,6);

                for(int k = 0; k < 6 ; k++)
                 for(int h = 0; h < 6 ; h++)
                   cov.element(k, h) = 0.0;
                dsm->m_lastX.resize(qx.Num());
                dsm->m_lastY.resize(qx.Num());
                dsm->m_lastZ.resize(qx.Num());
                affine_trans_point_3d(hommat, qx, qy, qz, &qx2, &qy2, &qz2);
                HTuple Meanx2, Meany2, Meanz2;
                tuple_mean((qx2), &Meanx2);
                tuple_mean((qy2), &Meany2);
                tuple_mean((qz2), &Meanz2);
               
                for(int i = 0; i < qx.Num(); i++)
                {
                  double xd = qx2[i].D() - Meanx2[0].D(),
                         yd = qy2[i].D() - Meany2[0].D(),
                         zd = qz2[i].D() - Meanz2[0].D();
                  
                  dsm->m_lastX[i] = qx2[i].D();
                  dsm->m_lastY[i] = qy2[i].D();
                  dsm->m_lastZ[i] = qz2[i].D();                
                  cov.element(0,0) = max(xd,  cov.element(0,0));
                  cov.element(1,1) = max(yd,  cov.element(1,1));
                  cov.element(2,2) = max(zd,  cov.element(2,2));
                }
                

                RelPose *pose = RelPoseFactory::FRelPose(img->GetPose(),m, cov);             
                result.push_back(pose);
                numOfObjects = 1;
                qualityMeasure = 1.0;
                pose->m_qualityMeasure = 1.0;
                
                cam->Publish3DData(dsm->m_lastX, dsm->m_lastY, dsm->m_lastZ);
              }
              else
              {
                printf("reconstruction was empty, rejected result\n");
                numOfObjects = 0;
                qualityMeasure = 0.0;
              }
            }
            catch(Halcon::HException ex)
            {
                ROS_ERROR("Error finding pancakes: %s", ex.message);
            }
          }
          else
          {
            printf("No Model");
          }
        }
        img->Free();
      }
  }
  else
  {
    printf("No Camera?\n");
  }
  return result;

}



// Local procedures
void find_pancake (Halcon::Hobject ImageFull, Halcon::Hobject ModelContours, Halcon::Hobject *SelectedRegions,
    Halcon::HTuple ModelID, Halcon::HTuple CamParamOut, Halcon::HTuple NFinalPose,
    Halcon::HTuple HomMat3D, Halcon::HTuple HomMat2dLast, Halcon::HTuple *Qx, Halcon::HTuple *Qy,
    Halcon::HTuple *Qz, Halcon::HTuple *HomMat2dLastOut, int min_area, int max_area, double min_compactness, double max_compactness)
{
  using namespace Halcon;

  // Local iconic variables
  Hobject  ContoursProjTrans, Rectangle, RegionUnion;
  Hobject  Circle, ImageReduced1, ImageGauss, Region1, RegionCloing;
  Hobject  ConnectedRegions;


  // Local control variables
  HTuple  HomMat2D, Score, Row, Column, Phi, Length1;
  HTuple  Length2, Row1, Column1, Radius, Number, Rows, Columns;
  HTuple  X1, Y1, Z, Meanx, Meany, Meanz;

  find_planar_uncalib_deformable_model(ImageFull, ModelID, -0.78, 0.78, 1, 1, 1,
      1, 0.5, 1, 1, 0, 0.9, HTuple(), HTuple(), &HomMat2D, &Score);
  printf("Number of matched Shape Models: %ld\n", Score.Num());
  if (0 != ((HomMat2D.Num())==0))
  {
    HomMat2D = HomMat2dLast;
  }
  if (0 != ((HomMat2D.Num())>0))
  {
    (*HomMat2dLastOut) = HomMat2D;
    projective_trans_contour_xld(ModelContours, &ContoursProjTrans, HomMat2D);
    smallest_rectangle2_xld(ContoursProjTrans, &Row, &Column, &Phi, &Length1, &Length2);


    gen_rectangle2(&Rectangle, Row, Column, Phi, Length1, Length2);
    union1(Rectangle, &RegionUnion);
    smallest_circle(RegionUnion, &Row1, &Column1, &Radius);
    gen_circle(&Circle, Row1, Column1, Radius);
    reduce_domain(ImageFull, Circle, &ImageReduced1);
    gauss_image(ImageReduced1, &ImageGauss, 5);
    threshold(ImageReduced1, &Region1, 100, 255);
    closing_circle(Region1, &RegionCloing, 5);
    connection(RegionCloing, &ConnectedRegions);
    select_shape(ConnectedRegions, &(*SelectedRegions), (HTuple("area").Append("compactness")),
        "and", (HTuple(min_area).Append(min_compactness)), (HTuple(max_area).Append(max_compactness)));
    count_obj((*SelectedRegions), &Number);
    if (0 != (Number==0))
    {
      printf("No Pancake Found\n");
      return;
    }
    get_region_points((*SelectedRegions), &Rows, &Columns);
    image_points_to_world_plane(CamParamOut, NFinalPose, Rows, Columns, "m", &X1,
        &Y1);
    tuple_gen_const(X1.Num(), 0.0, &Z);
    affine_trans_point_3d(HomMat3D, X1, Y1, Z, &(*Qx), &(*Qy), &(*Qz));

  }
  else
  {
    printf("No PanCakeMaker found!\n");
  }
  return;
}

