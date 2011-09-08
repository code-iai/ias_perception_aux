
#include "cpp/HalconCpp.h"
#include "DetectPlate.h"
#include "CircleDescriptor.h"
#include "RangeSensor.h"



using namespace cop;
// Local procedures
void detect_big_circle (Halcon::Hobject ImageFull, Halcon::Hobject *SelectedXLD, Halcon::HTuple CamParam,
    double min_length, double max_length, double min_circularity, double max_circularity, double radius,
    Halcon::HTuple *Pose1, Halcon::HTuple *Pose2)
{
  using namespace Halcon;
  Hobject Edges, UnionContours, SelectedContours2;
  edges_sub_pix(ImageFull, &(Edges), "canny", 1, 20, 40);
  union_adjacent_contours_xld((Edges), &(UnionContours), 10, 1, "attr_keep");
  segment_contours_xld (UnionContours,  &(UnionContours), "lines_circles", 0, 80, 80);
  select_contours_xld((UnionContours), &(SelectedContours2), "contour_length",
      min_length, max_length, 0.5, 2.0);

  select_contours_xld (SelectedContours2, &SelectedContours2, "curvature", 1,100, 1, 100);
  HTuple Circularity, Indices;

  circularity_xld (SelectedContours2, &Circularity);
  if(Circularity.Num() == 0)
  {  
    edges_sub_pix(ImageFull, &(Edges), "canny", 1, 5, 20);
    union_adjacent_contours_xld((Edges), &(UnionContours), 15, 4, "attr_keep");
    segment_contours_xld (UnionContours,  &(UnionContours), "lines_circles", 4, 10, 40);
    select_contours_xld((UnionContours), &(SelectedContours2), "contour_length",
                        min_length, max_length, 0.5, 2.0);

    select_contours_xld (SelectedContours2, &SelectedContours2, "curvature", 1,100, 1, 100); 
    HTuple Circularity, Indices;

    circularity_xld (SelectedContours2, &Circularity);
  }
  
  if(Circularity.Num() == 0)
    throw "no contours in the image (light?)";
  tuple_sort_index (Circularity, &Indices);
  select_obj (SelectedContours2, &(*SelectedXLD), Indices[Indices.Num() - 1].I() + 1);

    /*select_shape_xld((SelectedContours2), &(*SelectedXLD), "circularity", "and", 0.5,
        1.0);*/
  get_circle_pose((*SelectedXLD), CamParam, radius, "pose", &(*Pose1), &(*Pose2));
  return;
}

std::vector<RelPose*> DetectPlate::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  printf("Called DetectPlate");
  std::vector<RelPose*> result;
  Camera* cam = Camera::GetFirstCamera(sensors);
  if(cam != NULL)
  {
    Image* img = cam->GetImage(-1);
    RelPose* camPose = cam->m_relPose;
    if(img != NULL && camPose != NULL )
    {
      using namespace Halcon;
      HTuple camparam = cam->m_calibration.CamParam();

      if(img->GetType() == ReadingType_HalconImage)
      {
          CircleDescriptor* dsm = (CircleDescriptor*)(object.GetElement(0, DESCRIPTOR_CIRCLE));
          if(dsm != NULL)
          {
            //hommatfirst = *dsm->GetHomMatFirst();
            try
            {
              Hobject contour;
              Hobject imgs = *img->GetHImage();
              HTuple pose1, pose2, covariance;
              double min_length = 100, max_length = 10000, min_circularity = 0.5, max_circularity = 1.0;

              detect_big_circle (imgs, &contour, camparam,
                                min_length, max_length, min_circularity, max_circularity, dsm->GetRadius(),
                                 &pose1, &pose2);

              if(pose1.Num() > 0)
              {
                /* Given there is a 3D sensor we can cross check the extrakted pose in the depth data*/
                for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
                {
                  if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0 || (*it)->GetName().compare(XML_NODE_RANGESENSOR) == 0)
                  {
                    try
                    {
                        Halcon::HTuple HomMat3d, HomMat3Dinv;
                        SwissRangerReading* read = (SwissRangerReading*)(*it)->GetReading();
                        if(read != NULL)
                        {
                          sensor_msgs::PointCloud &pcd = read->m_image;
                          RelPoseHTuple::GetHommat((*it)->GetRelPose(), &HomMat3d, camPose->m_uniqueID);
                          printf("Sens to ref frame (%ld -> %ld) (initial num points = %ld)\n", (*it)->GetRelPose()->m_uniqueID, camPose->m_uniqueID, pcd.points.size());
                          hom_mat3d_invert(HomMat3d, &HomMat3Dinv);
                          Halcon::HTuple X,  Y,  Z, QX, QY, QZ, Row, Col;
                          int  counter = 0;
                          for(size_t i = 0; i < (pcd.points.size()); i++)
                          {
                            //printf("%f %f %f", pcd.points[i].x, pcd.points[i].y, pcd.points[i].z);
                            if( pcd.points[i].z !=  pcd.points[i].z || pcd.points[i].z == 0.0)
                            {
                              continue;
                            }
                            X[counter] = pcd.points[i].x;
                            Y[counter] = pcd.points[i].y;
                            Z[counter] = pcd.points[i].z;
                            counter++;
                          }
                          if(counter == 0)
                          {
                            printf("No points from Sensor: %s\n", (*it)->GetSensorID().c_str());
                            return result;
                            break;
                          }
                          Halcon::affine_trans_point_3d(HomMat3d, X, Y, Z, &QX, &QY, &QZ);
                          Halcon::project_3d_point(QX, QY, QZ, camparam, &Row, &Col);
                          Halcon::HTuple RowCenter, ColCenter, RowUp, ColUp, RowLeft, ColLeft, RowDelta, ColDelta, RowSgn, ColSgn, Eukl;
                          Halcon::HTuple SelectedRow, SelectedCol, SelectedFinal;
                          double max_row_diff, max_col_diff, allowed_dist;
                          Halcon::project_3d_point(pose1[0].D(), pose1[1], pose1[2], camparam, &RowCenter, &ColCenter);
                          Halcon::project_3d_point(pose1[0].D() - dsm->GetRadius(), pose1[1], pose1[2], camparam, &RowUp, &ColUp);
                          Halcon::project_3d_point(pose1[0], pose1[1].D() + dsm->GetRadius(), pose1[2], camparam, &RowLeft, &ColLeft);
                          max_row_diff = max(fabs(RowUp[0].D() - RowCenter[0].D()),fabs(RowLeft[0].D() - RowCenter[0].D()));
                          max_col_diff = max(fabs(ColUp[0].D() - ColCenter[0].D()),fabs(ColLeft[0].D() - ColCenter[0].D()));
                          allowed_dist = 0.2*sqrt(max_row_diff*max_row_diff + max_col_diff*max_col_diff);
                          Halcon::tuple_abs(Row - RowCenter, &RowDelta);
                          Halcon::tuple_abs(Col - ColCenter, &ColDelta);
                          Halcon::tuple_sgn(RowDelta - max_row_diff, &RowSgn);
                          Halcon::tuple_find(RowSgn, -1, &SelectedRow);
                          Halcon::tuple_select(RowDelta, SelectedRow, &RowDelta);
                          Halcon::tuple_select(ColDelta, SelectedRow, &ColDelta);
                          Halcon::tuple_select(QX, SelectedRow, &QX);
                          Halcon::tuple_select(QY, SelectedRow, &QY);
                          Halcon::tuple_select(QZ, SelectedRow, &QZ);
                          Halcon::tuple_sgn(ColDelta - max_col_diff, &ColSgn);
                          Halcon::tuple_find(ColSgn, -1, &SelectedCol);
                          Halcon::tuple_select(RowDelta, SelectedCol, &RowDelta);
                          Halcon::tuple_select(ColDelta, SelectedCol, &ColDelta);
                          Halcon::tuple_select(QX, SelectedCol, &QX);
                          Halcon::tuple_select(QY, SelectedCol, &QY);
                          Halcon::tuple_select(QZ, SelectedCol, &QZ);
                          Halcon::tuple_sqrt((ColDelta*ColDelta)+(RowDelta*RowDelta), &Eukl);
                          Halcon::tuple_sgn(Eukl - allowed_dist, &ColSgn);
                          Halcon::tuple_find(ColSgn, -1, &SelectedCol);
                          Halcon::tuple_select(QX, SelectedCol, &QX);
                          Halcon::tuple_select(QY, SelectedCol, &QY);
                          Halcon::tuple_select(QZ, SelectedCol, &QZ);
                          Halcon::HTuple MeanX, MeanY, MeanZ;
                          Halcon::tuple_mean(QX, &MeanX);
                          Halcon::tuple_mean(QY, &MeanY);
                          Halcon::tuple_mean(QZ, &MeanZ);
                          double lengthnew = sqrt(MeanX[0].D() * MeanX[0].D()+ MeanY[0].D() * MeanY[0].D() + MeanZ[0].D() * MeanZ[0].D());
                          double lengthold = sqrt( pose1[0].D()* pose1[0].D() + pose1[1].D()*pose1[1].D()+ pose1[2].D()*pose1[2].D());

                          printf("Pose before %f %f %f\n Mean in pcd: %f %f %f\n Pose after %f %f %f\n", pose1[0].D(), pose1[1].D(), pose1[2].D(), MeanX[0].D(),MeanY[0].D(), MeanZ[0].D()
                          , (pose1[0].D() / lengthold) * lengthnew, (pose1[1].D() / lengthold) * lengthnew, (pose1[2].D() / lengthold) * lengthnew);
                          pose1[0] = (pose1[0].D() / lengthold) * lengthnew;
                          pose1[1] = (pose1[1].D() / lengthold) * lengthnew;
                          pose1[2] = (pose1[2].D() / lengthold) * lengthnew;
                          dsm->m_x.clear();
                          dsm->m_y.clear();
                          dsm->m_z.clear();
                          for(int i = 0; i < QX.Num(); i++)
                          {
                            dsm->m_x.push_back(QX[i].D());
                            dsm->m_y.push_back(QY[i].D());
                            dsm->m_z.push_back(QZ[i].D());
                          }
                          //printf("Publish 3d data now of size %ld\n", x.size());
                          //cam->Publish3DData(x,y,z);


                        }

                    }
                    catch (Halcon::HException ex)
                    {
                      printf("Error: %s\n", ex.message);
                      return result;
                    }
                    catch (const char* text )
                    {
                       printf("Error in DetectPlate: %s\n", text);
                       return result;
                    }
                  }
                }


                RelPose* pose = RelPoseHTuple::FRelPose(pose1, covariance, img->GetPose(), - 1);

                result.push_back(pose);
                numOfObjects = 1;
                qualityMeasure = 1.0;

                pose->m_qualityMeasure = 1.0;
              }
              else
              {
                printf("No Pose for plate\n");
                numOfObjects = 0;
                qualityMeasure = 0.0;
              }
            }
            catch(Halcon::HException ex)
            {
                ROS_ERROR("Error finding circles: %s", ex.message);
                numOfObjects = 0;
                qualityMeasure = 0.0;
            }
          }
        }
        img->Free();
      }
  }
  return result;

}

double DetectPlate::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  if(object.GetElement(0,DESCRIPTOR_CIRCLE))
  {
    bool has3D = false;
    bool has2D = false;
    for(std::vector<Sensor*>::const_iterator it = sensors.begin(); it != sensors.end(); it++)
    {
      if((*it)->GetName().compare(XML_NODE_SWISSRANGER) == 0)
      {
        has3D = true;
      }
      if((*it)->IsCamera())
      {
        has2D = true;
      }
    }
    if(has3D && has2D)
      return 1.0;
    else if (has2D)
      return 0.5;
    else
      return 0.0;
  }
  else
    return 0.0;
}

