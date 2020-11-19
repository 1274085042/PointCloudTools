#include <iostream>
#include <ctime>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char **argv)
{

    
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ c;
    //Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("../data/i2.pcd",*cloud_blob);

    auto start=std::chrono::system_clock::now();

    //pcl::PCDWriter writer;
    
    //Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_blob,*cloud_filtered);

    //pcl::PassThrough<pcl::PointXYZ> pass;
    //pass.setInputCloud(cloud_filtered);
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(-8.3,-4.3);
    //pass.filter(*pass_cloud);
    //pcl::io::savePCDFileASCII("i2_pass_83_43.pcd",*pass_cloud);

    pcl::ModelCoefficients::Ptr coefficiens(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.2);

    //Create the filtering object
    pcl:: ExtractIndices<pcl::PointXYZ> extract;

    int i=0;
    int nr_points=(int) cloud_filtered->size();

    pcl::PointXYZ cloud_max;
    pcl::PointXYZ cloud_min;
    pcl::getMinMax3D(*cloud_filtered,cloud_min,cloud_max);

    std::cout<<"cloud_max: "<<cloud_max<<std::endl;
    std::cout<<"cloud_min: "<<cloud_min<<std::endl;

    //while 30% of the origin cloud is still there
    while(cloud_filtered->size() > 0.3 * nr_points)
    {
        //Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers,*coefficiens);
        if(inliers->indices.size()==0)
        {
            std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
            break;
        }

        //Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        
        //std::cout<<cloud_p->size()<<std::endl;
        for(int i=0;i<cloud_p->size();i++)
        {
            centroid.add(cloud_p->points[i]);
        }
        
        pcl::PointXYZ max;
        pcl::PointXYZ min;

        pcl::getMinMax3D(*cloud_p,min,max);
        std::cout<<"Min: "<<min<<std::endl;
        std::cout<<"Max: "<<max<<std::endl;

        centroid.get(c);
        std::cout<<"Mean: "<<c<<std::endl;
        std::cout<<"PointCloud representing the planar component: "<<cloud_p->width*cloud_p->height<<" data points."<<std::endl;

        std::stringstream ss;
        ss<<"i2_"<<i<<".pcd";
        writer.write<pcl::PointXYZ> (ss.str(),*cloud_p,false);

        //Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
    std::chrono::duration<double,std::ratio<1,1000>>  diff=std::chrono::system_clock::now()-start;
    std::cout<<"Time: "<<diff.count()<<std::endl;
    writer.write<pcl::PointXYZ>("segmentation_result.pcd",*cloud_filtered,false);
    return 0;
}
