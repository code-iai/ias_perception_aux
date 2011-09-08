#include "cpp/HalconCpp.h"


#include "RelPoseHTuple.h"
#include "WWDetector.h"

using namespace Halcon;
using namespace cop;

#define XML_NODE_LASTX_VEC "points_x"
#define XML_NODE_LASTY_VEC "points_y"
#define XML_NODE_LASTZ_VEC "points_z"


void WeisswurstContainer::SaveTo(XMLTag* tag)
{
  SurfaceModel::SaveTo(tag);
  tag->AddChild(XMLTag::Tag(m_lastX, XML_NODE_LASTX_VEC));
  tag->AddChild(XMLTag::Tag(m_lastY, XML_NODE_LASTY_VEC));
  tag->AddChild(XMLTag::Tag(m_lastZ, XML_NODE_LASTZ_VEC));
}


void WeisswurstContainer::SetData(XMLTag* tag)
{
  SurfaceModel::SetData(tag);
  
  if(tag != NULL)
  {
    XMLTag* x_tag = tag->GetChild(XML_NODE_LASTX_VEC);
    if(x_tag != NULL)
    {
      m_lastX = XMLTag::Load(x_tag, &m_lastX);
    }
    XMLTag* y_tag = tag->GetChild(XML_NODE_LASTY_VEC);
    if(y_tag != NULL)
    {
      m_lastY = XMLTag::Load(y_tag, &m_lastY);
    }
    XMLTag* z_tag = tag->GetChild(XML_NODE_LASTZ_VEC);
    if(z_tag != NULL)
    {
      m_lastZ = XMLTag::Load(z_tag, &m_lastZ);
    }
  }
}


void TransformPointLocally(Matrix m, double x_in, double y_in, double z_in, double &x_out,double &y_out,double &z_out, double scale = 1.0)
{
  ColumnVector d(4);
  d << x_in  << y_in  << z_in << 1;
  ColumnVector f = m * d;
//  cout << m << " \n* \n" << d << "\n=\n" << f;
  x_out = f.element(0);
  y_out = f.element(1);
  z_out = f.element(2);

}

bool WeisswurstContainer::GetShape(GeometricShape &objectShape) const
{
  if(m_poseLastMatchReading == NULL)
  {
    printf("No pose\n");
  }
  objectShape.type = 4;/*undefined PCD;*/
  /*Matrix cov = m_poseLastMatchReading->GetCovarianceMatrix();*/
  LocatedObjectID_t id;
  if(GetLastMatchedPose() != NULL)
  {
     const RelPose* pose2 = GetLastMatchedPose();

     id = pose2->m_uniqueID;
     RelPose* pose = RelPoseFactory::FRelPose(id);
     Matrix m = pose->GetMatrix(ID_WORLD);
     RelPoseFactory::FreeRelPose(&pose);
     for(size_t i = 0;i < m_lastX.size(); i++)
     {
       cop::PointShape p;
       TransformPointLocally(m, m_lastX[i], m_lastY[i], m_lastZ[i], p.x, p.y , p.z );
       objectShape.vertices.push_back(p);
     }
     return true;
  }
  return false;
}

// Local procedures
void find_ww (Halcon::Hobject ImageFull, Halcon::Hobject *SelectedRegions,
    Halcon::HTuple CamParam, Halcon::HTuple NFinalPose,
    Halcon::HTuple HomMat3D, Halcon::HTuple *Qx, Halcon::HTuple *Qy,
    Halcon::HTuple *Qz,  Halcon::HTuple *Num,
    int min_area = 50, int max_area = 30000, int min_gray=140);

/**
  Test if the already there are models learned
*/
double WWDetector::CheckSignature(const Signature& object, const std::vector<Sensor*> &sens)
{
  printf("WWDetector::CheckSignature\n");
  
  Descriptor* descr = (Descriptor*)object.GetElement(0,DESCRIPTOR_SURFACE);
  if(descr != NULL && descr->GetNodeName().compare(XML_NODE_WEISSWURSTCONTAINER) == 0)
  {
    return 1.0;
  }
  else
    return 0.0;
}

HTuple HomMat2dLast;


void  WWDetector::SetData(XMLTag* tag)
{
  printf("Loading WWDetector\n");
  m_surfaceDetect = new SurfaceDetection();
  m_surfaceDetect->SetData(tag->GetChild(XML_NODE_SURFACEDETECTION));
}
    /**
      Action function, prepares images
    */
std::vector<RelPose*> WWDetector::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  printf("Called WWDetector\n");
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
          WeisswurstContainer* sfm = (WeisswurstContainer*)(object.GetElement(0, DESCRIPTOR_SURFACE));
          if( sfm != NULL)
          {
            printf("Loaded a WeissWurst Container model\n");
            //hommatfirst = *dsm->GetHomMatFirst();
            try
            {
              Hobject imgs = *img->GetHImage();
              Hobject result_region;
              HTuple pose_in, hom_in, qxa, qya, qza, Meanx, Meany, Meanz, Num, pose_base_link_ori;

              std::vector<RelPose*> pots = m_surfaceDetect->Perform(sensors, pose, object, numOfObjects, qualityMeasure);
              
              
              if(numOfObjects > 0 && pots.size() > 0)
              {
                RelPose* pose_bl = RelPoseFactory::GetRelPose("/base_link");
                Matrix m1 = IdentityMatrix(4), cov1(6,6);
                for(int k = 0; k < 6 ; k++)
                 for(int h = 0; h < 6 ; h++)
                   cov1.element(k, h) = 0.0;
                   
                RelPoseHTuple::GetPose(pots[0], &pose_in, pose_bl->m_uniqueID);
                m1.element(0,3) = pose_in[0];
                m1.element(1,3) = pose_in[1];
                if(pose_in[2].D() - m_surfaceDetect->m_lastTableHeight > 0.13)
                {
                  m1.element(2,3) = m_surfaceDetect->m_lastTableHeight + 0.08;
                }
                else
                  m1.element(2,3) = pose_in[2];
                
                RelPose* pose2 = RelPoseFactory::FRelPose(pose_bl->m_uniqueID, m1, cov1);
                

                RelPoseHTuple::GetPose(pose2, &pose_in, img->GetPose()->m_uniqueID);
                 
                RelPoseFactory::FreeRelPose(&pose_bl);
                RelPoseFactory::FreeRelPose(&pose2);
                
                
                pose_to_hom_mat3d(pose_in, &hom_in);
                
                find_ww(imgs,  &result_region, camparam, pose_in, hom_in, &qxa, &qya, &qza,&Num, 150, 30000, 140);
                if(Num.Num()==0)
                  find_ww(imgs,  &result_region, camparam, pose_in, hom_in, &qxa, &qya, &qza,&Num, 150, 30000, 80);
                int i = 0;
                int count = 0;
                for(i = 0 ;  i < Num.Num(); i++)
                {
                  HTuple qx, qy, qz;
                  tuple_select_range(qxa, count, Num[i].I() + count - 1, &qx);
                  tuple_select_range(qya, count, Num[i].I() + count - 1, &qy);
                  tuple_select_range(qza, count, Num[i].I() + count - 1, &qz);
                  count +=  Num[i].I();
                  tuple_mean((qx), &Meanx);
                  tuple_mean((qy), &Meany);
                  tuple_mean((qz), &Meanz);
                  HTuple hommat, inv, qx2, qy2, qz2;
                  RelPoseHTuple::GetHommat(img->GetPose(), &hommat, ID_WORLD);
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
                  sfm->m_lastX.resize(qx.Num());
                  sfm->m_lastY.resize(qx.Num());
                  sfm->m_lastZ.resize(qx.Num());
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

                    sfm->m_lastX[i] = qx2[i].D();
                    sfm->m_lastY[i] = qy2[i].D();
                    sfm->m_lastZ[i] = qz2[i].D();
                    cov.element(0,0) = max(xd,  cov.element(0,0));
                    cov.element(1,1) = max(yd,  cov.element(1,1));
                    cov.element(2,2) = max(zd,  cov.element(2,2));
                  }

                  m.element(0,0) = 1; m.element(0,1) = 0; m.element(0,2) = 0; m.element(0,3) = Meanx2[0].D();
                  m.element(1,0) = 0; m.element(1,1) = 1; m.element(1,2) = 0; m.element(1,3) = Meany2[0].D();
                  m.element(2,0) = 0; m.element(2,1) = 0; m.element(2,2) = 1; m.element(2,3) = Meanz2[0].D();
                  m.element(3,0) = 0.0;m.element(3,1) = 0.0;m.element(3,2) = 0.0;m.element(3,3) = 1.0;


                  RelPose *pose = RelPoseFactory::FRelPose(ID_WORLD,m, cov);
                  result.push_back(pose);
                  numOfObjects = 1;
                  qualityMeasure = 1.0;
                  pose->m_qualityMeasure = 1.0;

                  /*cam->Publish3DData(dsm->m_lastX, dsm->m_lastY, dsm->m_lastZ);*/
                }
                
                if(Num.Num() == 0)
                {
                  printf("reconstruction was empty, rejected result\n");
                  numOfObjects = 0;
                  qualityMeasure = 0.0;
                }
              }
            }
            catch(Halcon::HException ex)
            {
                ROS_ERROR("Error finding Weisswursts: %s", ex.message);
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
void find_ww (Halcon::Hobject ImageFull, Halcon::Hobject *SelectedRegions,
    Halcon::HTuple CamParam, Halcon::HTuple NFinalPose,
    Halcon::HTuple HomMat3D, Halcon::HTuple *Qx, Halcon::HTuple *Qy,
    Halcon::HTuple *Qz, Halcon::HTuple *Num, int min_area, int max_area, int min_gray)
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
  project_3d_point(NFinalPose[0], NFinalPose[1], NFinalPose[2], CamParam, &Row, &Column);
  Radius = 105;
  gen_circle(&Circle, Row, Column, Radius);
  reduce_domain(ImageFull, Circle, &ImageReduced1);
  printf("Circle around %f %f \n", Row[0].D(), Column[0].D() );
  //gauss_image(ImageReduced1, &ImageGauss, 5);
  threshold(ImageReduced1, &Region1, min_gray, 255);
  
  HTuple area, r, c;
  area_center(Region1, &area, &r, &c);
  printf("Area after first threshold %d\n", area[0].I());
  closing_circle(Region1, &RegionCloing, 5);

  area_center(RegionCloing, &area, &r, &c);
  printf("Area after closing %d\n", area[0].I());

  connection(RegionCloing, &ConnectedRegions);
  count_obj(ConnectedRegions, &Number);
  printf("Number of connected components %d\n", Number[0].I());
  select_shape(ConnectedRegions, &(*SelectedRegions), HTuple("area"),
      "and", HTuple(min_area), HTuple(max_area));
  count_obj((*SelectedRegions), &Number);
  if (0 != (Number==0))
  {
    printf("No Weisswurst Found\n");
    return;
  }
  for(int i = 0; i < Number; i++)
  {
    Hobject obj;
    HTuple  X1A, Y1A;
    select_obj((*SelectedRegions), &obj, i + 1);
    get_region_points(obj, &Rows, &Columns);
    image_points_to_world_plane(CamParam, NFinalPose, Rows, Columns, "m", &X1A,
      &Y1A);
    tuple_concat(X1, X1A, &X1);
    tuple_concat(Y1, Y1A, &Y1);
    tuple_concat((*Num), X1A.Num(), Num );
  }
  tuple_gen_const(X1.Num(), 0.0, &Z);
  affine_trans_point_3d(HomMat3D, X1, Y1, Z, &(*Qx), &(*Qy), &(*Qz));
  return;
}

