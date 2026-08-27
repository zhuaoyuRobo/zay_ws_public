#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>

int main()
{
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setNumThreads(1);
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  return ndt.getResolution() > 0.0 ? 0 : 1;
}
