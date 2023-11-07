#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>         //getMinMax3D函数需要用到
#include <pcl/kdtree/kdtree_flann.h> 	//kdtree类定义头文件
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;

void randomPoint(PointXYZ *pSearch, PointXYZ pMin, PointXYZ pMax) {
    pSearch->x = rand()/RAND_MAX * (pMax.x - pMin.x) + pMin.x;
    pSearch->y = rand()/ RAND_MAX * (pMax.x - pMin.x) + pMin.y;
    pSearch->z = rand()/ RAND_MAX * (pMax.x - pMin.x) + pMin.z;
    cout << "the point ("
        << pSearch->x << " " << pSearch->y << " " << pSearch->z
        << ") will be searched" <<endl;
}

int main()
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    if (io::loadPCDFile("bun02.pcd", *cloud) != 0)
    {
        return -1;
    }

    // 创建滤波对象
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // 设置体素栅格的大小为 1x1x1cm
    filter.setLeafSize(0.01f, 0.01f, 0.01f);
    filter.filter(*filteredCloud);

    KdTreeFLANN<PointXYZ> kdtree;       //建立kd tree
    kdtree.setInputCloud(filteredCloud);        //设置搜索空间
    
    PointXYZ pSearch, pMin, pMax;       //搜索点，三个轴的最大值和最小值
    getMinMax3D(*cloud, pMin, pMax);    //需要include<pcl/common/common.h>
    randomPoint(&pSearch, pMin, pMax);  //随机生成搜索点
    PointXYZ tmp;       //用于存储临时点

    int K = 2;
    std::vector<int> ptIdxByKNN(K);      //存储查询点近邻索引
    std::vector<float> ptKNN(K); 		//存储近邻点对应距离平方
    kdtree.nearestKSearch(pSearch, K, ptIdxByKNN, ptKNN); //执行K近邻搜索

    cout << "KNN search with K=" << K << endl; 
    for (size_t i = 0; i < ptIdxByKNN.size(); ++i) {
        tmp = cloud->points[ptIdxByKNN[i]];
        cout << tmp.x << " " << tmp.y << " " << tmp.z     //打印搜索到的点
            << " (squared distance: " << ptKNN[i] << ")" << endl;
    }

    // 半径 R内近邻搜索方法
    float r = 2.5;
    std::vector<int> ptIdxByRadius;   //存储近邻索引
    std::vector<float> ptRadius;      //存储近邻对应距离的平方
    kdtree.radiusSearch(pSearch, r, ptIdxByRadius, ptRadius);
    std::cout << "search with radius=" << r << endl;
    for (size_t i = 0; i < ptIdxByRadius.size(); ++i) {
        tmp = cloud->points[ptIdxByRadius[i]];
        cout << tmp.x << " " << tmp.y << " " << tmp.z
            << " (squared distance: " << ptRadius[i] << ")" << endl;
    }
    return 0;
}