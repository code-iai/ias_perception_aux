#ifndef RELPOSEHTUPLE_H
#define RELPOSEHTUPLE_H

#include "RelPose.h"
#include "cpp/HalconCpp.h"

namespace cop
{
  class RelPoseHTuple
  {
  public:
    static void Update(RelPose* pose, Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation);
    static void Update(RelPose* pose, Halcon::HTuple& poseDesc, Matrix& covariance, RelPose* relation);

    static bool TupleToMat(Halcon::HTuple& poseDesc, Halcon::HTuple&  covariance, Matrix &m, Matrix &d);
    static bool TupleToMat(Halcon::HTuple& poseDesc, Matrix&  covariance, Matrix &m, Matrix &d);

    static void GetPose(RelPose* posein, Halcon::HTuple* pose, LocatedObjectID_t poseRel = 0);
    static void GetHommat(RelPose* pose, Halcon::HTuple* hommat, LocatedObjectID_t poseRel) ;
    static void Print(RelPose* pose);

    static RelPose* FRelPose(Halcon::HTuple& poseDesc, Halcon::HTuple& covariance, RelPose* relation, LocatedObjectID_t id = 0);
    static RelPose* FRelPose(Halcon::HTuple& poseDesc, Matrix& covariance, RelPose* relation, LocatedObjectID_t id = 0);

    static Halcon::HTuple m2HT(Matrix m);


  };
}
#endif /*RELPOSEHTUPLE_H*/
