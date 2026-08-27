#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

TEST(PublicApi, ConstructsExportedPointXyzRegistrations)
{
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setNumThreads(2);
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt.setResolution(1.0);

  pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaximumIterations(5);

  EXPECT_DOUBLE_EQ(ndt.getResolution(), 1.0);
  EXPECT_EQ(gicp.getMaximumIterations(), 5);
}
