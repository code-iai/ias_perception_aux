//Header for icpCpp.cpp file

#ifndef ICP_H
#define ICP_H

#include "cpp/HalconCpp.h"
namespace kdtree{
#include "icp/kdtree_common.h"
}
using namespace kdtree;

void icp(
double *trpr,
double *ttpr,
double *modelz,
unsigned int nmodelz,
double *dataz,
double *qltyz,
unsigned int ndataz,
//unsigned int *randvecz,
//unsigned int nrandvecz,
//unsigned int nrandz,
unsigned int iimax,
Tree *pointer_to_tree,
Halcon::HTuple *model_indices,
Halcon::HTuple *data_indices
);

#endif
