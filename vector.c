#include "vector.h"

vector_double_3d cross_product_double_3d(vector_double_3d v1, vector_double_3d v2)
{
  vector_double_3d ret = {v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
  return ret;
}
