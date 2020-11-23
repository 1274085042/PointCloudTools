#include <ctime>
#include <chrono>
#include <iostream>
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
    pcl::PCDWriter writer;
    std::vector<boost::filesystem::path> filepaths(boost::filesystem::directory_iterator{"../../3d_url"}, boost::filesystem::directory_iterator{});
    for(auto fp=filepaths.begin();fp!=filepaths.end();fp++)
   {

       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr rem_cloud(new pcl::PointCloud<pcl::PointXYZ>);

       if(pcl::io::loadPCDFile<pcl::PointXYZ>(fp->string(),*cloud)==-1)
       {
           std::cout<<(*fp).string()<<std::endl;
           PCL_ERROR("Cont load pcd file");
       }
       else
       {
            int i=0;

            std::cout<<"<<<<<<<<<< "<<(*fp).string()<<" >>>>>>>>>>"<<std::endl;

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.2);

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            int nr_points=(int) cloud->size();

            while (cloud->size() > 0.3 *nr_points)
            {
                seg.setInputCloud(cloud);
                seg.segment(*inliers, *coefficients);
                if(inliers->indices.size()==0)
                {
                    std::cerr<<"Could not estimate a plane model for the given dataset"<<std::endl;
                    break;
                }

               //Extract the inliers
               extract.setInputCloud(cloud);
               extract.setIndices(inliers);
               extract.setNegative(false);  //保留分割出来的点
               extract.filter(*seg_cloud);

                if(i==0)
                {
               std::string folderpath="../../Ground_Out/";
               int end=(*fp).filename().string().length()-4;
               std::string filename=(*fp).filename().string().substr(0,end);

               std::stringstream ss;
               ss<<folderpath<<filename<<"_ground.pcd";
               writer.write<pcl::PointXYZ> (ss.str(),*seg_cloud,false);
                }

                extract.setNegative(true);  //将索引对应的的点丢掉，保留剩余的点
                extract.filter(*rem_cloud);
                cloud.swap(rem_cloud);

                i++;

            }

       }
       
   } 
  
}
