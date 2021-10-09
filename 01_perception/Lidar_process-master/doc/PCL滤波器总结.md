## PCL滤波器总结

1. 直通滤波器：对于在空间分布有一定空间特征的点云数据，比如使用线结构光扫描的方式采集点云，沿z向分布较广，但x,y向的分布处于有限范围内。此时可使用直通滤波器，确定点云在x或y方向上的范围，可较快剪除离群点，达到第一步粗处理的目的。
2. 体素滤波器：体素的概念类似于像素，使用AABB包围盒将点云数据体素化，一般体素越密集的地方信息越多，噪音点及离群点可通过体素网格去除。另一方面如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。过多的点云数量会对后续分割工作带来困难。体素滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。
3. 统计滤波器：考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。
4. 条件滤波：条件滤波器通过设定滤波条件进行滤波，有点分段函数的味道，当点云在一定范围则留下，不在则舍弃。
5. 半径滤波器：半径滤波器与统计滤波器相比更加简单粗暴。以某点为中心画一个圆计算落在该圆中点的数量，当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。此算法运行速度快，依序迭代留下的点一定是最密集的，但是圆的半径和圆内点的数目都需要人工指定。
   

```c++
#include<iostream>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
 
 
int main(int argc, char** argv)
{
  ///////****************************************************////////////////////
  /*创建点云数据集。*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 500;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  std::cout << "创建原始点云数据" << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    std::cerr << "   " << cloud->points[i].x << " "
      << cloud->points[i].y << " "
      << cloud->points[i].z << std::endl;
  }
  std::cout << "原始点云数据点数：" << cloud->points.size()<< std::endl << std::endl;
 
  ///////****************************************************////////////////////
 
  ///////****************************************************////////////////////
  /*方法一：直通滤波器对点云进行处理。*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);//
 
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(cloud);//输入点云
  passthrough.setFilterFieldName("z");//对z轴进行操作
  passthrough.setFilterLimits(0.0, 400.0);//设置直通滤波器操作范围
  //passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
  passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
 
  std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
 
  ///////****************************************************////////////////////
 
  ///////****************************************************////////////////////
  /*方法二：体素滤波器实现下采样*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
 
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid; 
  voxelgrid.setInputCloud(cloud);//输入点云数据
  voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);//AABB长宽高
  voxelgrid.filter(*cloud_after_voxelgrid);
 
  std::cout << "体素化网格方法后点云数据点数：" << cloud_after_voxelgrid->points.size() << std::endl;
  ///////****************************************************////////////////////
 
  ///////****************************************************////////////////////
  /*方法三：统计滤波器滤波*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);//
 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
  Statistical.setInputCloud(cloud);
  Statistical.setMeanK(20);//取平均值的临近点数
  Statistical.setStddevMulThresh(5);//临近点数数目少于多少时会被舍弃
  Statistical.filter(*cloud_after_StatisticalRemoval);
 
  std::cout << "统计分析滤波后点云数据点数：" << cloud_after_StatisticalRemoval->points.size() << std::endl;
  ///////****************************************************////////////////////
 
  ///////****************************************************////////////////////
  /*方法四：条件滤波器*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZ>);
 
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
  range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
  range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));  //LT表示小于等于
 
  pcl::ConditionalRemoval<pcl::PointXYZ> condition;
  condition.setCondition(range_condition);
  condition.setInputCloud(cloud);                   //输入点云
  condition.setKeepOrganized(true);
 
  condition.filter(*cloud_after_Condition);
  std::cout << "条件滤波后点云数据点数：" << cloud_after_Condition->points.size() << std::endl;
  ///////****************************************************////////////////////
 
  ///////****************************************************////////////////////
  /*方法五：半径滤波器*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
 
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器
 
  radiusoutlier.setInputCloud(cloud);    //设置输入点云
  radiusoutlier.setRadiusSearch(100);     //设置半径为100的范围内找临近点
  radiusoutlier.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除
                                 
  radiusoutlier.filter(*cloud_after_Radius);
  std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
 
  int a;
  std::cin >> a;
  return (0);
 
}
```

