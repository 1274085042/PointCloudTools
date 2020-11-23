#include <cmath>
#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>                      //pcd读写
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>             //法线特征
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>   //可视化点云类

pcl::PointCloud<pcl::Normal>::Ptr ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); //创建存储法线的对象

    //pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;   //创建法线估计的对象
    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne;  //多核、多线程法线估计对象

    ne.setInputCloud(cloud);
    ne.setRadiusSearch(radius);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());  //使用KdTree存储点云
    ne.setSearchMethod(tree);

    ne.compute(*cloud_normals);

    //pcl::removeNaNFromPointCloud(*cloud_normals,*cloud_normals);

    pcl::PointXYZ normal;
    pcl::PointXYZ sum(0,0,0);
    int size=0;

    for(size_t i=0; i<cloud_normals->size(); i++)
    {
        if(!(pcl_isfinite(cloud_normals->points[i].normal_x)) || !(pcl_isfinite(cloud_normals->points[i].normal_y))|| !(pcl_isfinite(cloud_normals->points[i].normal_z)) )
        {
            continue;
        }

        sum.x+=(cloud_normals->points[i].normal_x);
        sum.y+=(cloud_normals->points[i].normal_y);
        sum.z+=(cloud_normals->points[i].normal_z);
        size++;
    }

    normal.x=sum.x/size;
    normal.y=sum.y/size;
    normal.z=sum.z/size;
    
    double dot=normal.z;
    double dis1=1;
    double dis2=sqrt((normal.x*normal.x)+(normal.y*normal.y)+(normal.z*normal.z));
    double ra_angel=acos(dot/(dis2*dis1));
    double angel=ra_angel*180/M_PI;

    //std::cout<<sum<<std::endl;
    std::cout<<"--------------------------Compute Results--------------------------"<<std::endl;
    std::cout<<"radius: "<<radius<<std::endl;
    std::cout<<"points size: "<<size<<std::endl;
    std::cout<<"normal: "<<normal<<std::endl;
    std::cout<<"rad angle: "<<ra_angel<<std::endl;
    std::cout<<"angle: "<<angel<<std::endl;

    return cloud_normals;
}



int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    if(pcl::io::loadPCDFile("../ground_0.pcd",*cloud)==-1)
    {
        PCL_ERROR("Cont read file!");
        return -1;
    }

    pcl::PointXYZ maxpoint;
    pcl::PointXYZ minpoint;
    pcl::getMinMax3D(*cloud, minpoint, maxpoint);
    cout<<"min point: "<<minpoint<<endl;
    cout<<"max point: "<<maxpoint<<endl;
    
    double radius=2;
    normals=ComputeNormals(cloud , radius);

    std::stringstream ss;
    ss<<"../Normals_Out/noramls_"<<".pcd ";
    pcl::io::savePCDFileASCII(ss.str(),*normals);
    
    //pcl::visualization::PCLVisualizer viewer("PCL viewer");
    //viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer_ptr->addCoordinateSystem(4.0f,"global");         //坐标系
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255,0,0);
    viewer_ptr->addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, "original point cloud");
    viewer_ptr->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 5 , 2);
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original point cloud");

    while (!viewer_ptr->wasStopped())
    {
        viewer_ptr->spinOnce();
    }
    
    return 1;
}