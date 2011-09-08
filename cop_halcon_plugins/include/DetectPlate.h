#ifndef DETECTPLATE_H
#define  DETECTPLATE_H

#include <LocateAlgorithm.h>
#include <XMLTag.h>

#define XML_NODE_PLATEDETECTOR "DetectPlate"
namespace cop
{
class DetectPlate : public LocateAlgorithm
{
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual std::string GetName(){return XML_NODE_PLATEDETECTOR;}

    XMLTag* Save(){return new XMLTag(GetName());};



};

}

#endif /*DETECTPLATE_H*/

