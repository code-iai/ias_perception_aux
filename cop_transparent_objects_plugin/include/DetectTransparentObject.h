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


#ifndef DETECTTRANSPARENTOBJECT_H
#define DETECTTRANSPARENTOBJECT_H


#define XML_NODE_DETECTTRANSPARENTOBJECT "DetectTransparentObject"

#include "RefineAlgorithm.h"
#include "cpp/HalconCpp.h"


void Reconstruct_3D_Glasses (Halcon::Hobject Intensity1, Halcon::Hobject Distance1,
    Halcon::Hobject CandidateRegion1, Halcon::Hobject Intensity2, Halcon::Hobject Distance2,
    Halcon::Hobject CandidateRegion2, Halcon::Hobject ImageX1, Halcon::Hobject ImageY1,
    Halcon::Hobject ImageZ1, Halcon::Hobject ImageX2, Halcon::Hobject ImageY2,
    Halcon::Hobject ImageZ2, Halcon::HTuple *PCDX_Rec, Halcon::HTuple *PCDY_Rec, Halcon::HTuple *PCDZ_Rec,
    Halcon::HTuple SwissCamParam, Halcon::HTuple RelPoseReal, Halcon::HTuple *FoundGlassMeanDev,
    Halcon::HTuple HomMat3D_calib_inv, LocatedObjectID_t CandidatePosID, Halcon::HTuple RowCand, Halcon::HTuple ColCand,
    Halcon::HTuple Hommat_point_SR_to_base);

void ICP_screw_param (Halcon::HTuple X_2, Halcon::HTuple Y_2, Halcon::HTuple Z_2, 
    Halcon::HTuple X_1to2, Halcon::HTuple Y_1to2, Halcon::HTuple Z_1to2, Halcon::HTuple *RotX, 
    Halcon::HTuple *RotY, Halcon::HTuple *RotZ, Halcon::HTuple *HomMat3D_screw, Halcon::HTuple *RotY_plain,
    Halcon::HTuple *RotX_plain, Halcon::HTuple *RotZ_plain);

void Searchspace_Reduction (Halcon::HTuple RelPose, Halcon::HTuple CamParam,
	Halcon::Hobject ImageX, Halcon::Hobject ImageY, Halcon::Hobject ImageZ, Halcon::Hobject Candidate,
	Halcon::HTuple *SearchspaceRow, Halcon::HTuple *SearchspaceCol);



namespace cop
{


  class DetectTransparentObject :
    public RefineAlgorithm
  {
  public:
    DetectTransparentObject();

    void SetData(XMLTag* tag);

    ~DetectTransparentObject(void);

    virtual Descriptor* Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual XMLTag* Save();

    virtual std::string GetName(){return XML_NODE_DETECTTRANSPARENTOBJECT;}

  private:
    ros::Publisher Rec_Cloud_pub;

  };
}
#endif /*RFACOLORBYSHAPE_H*/
