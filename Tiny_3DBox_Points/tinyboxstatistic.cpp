#include <vector>
#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int main()
{
    int in_voxel_points_max=0;
    int voxel_points_sum=0;
    int voxels=0;


    std::vector<boost::filesystem::path> folderpaths(boost::filesystem::directory_iterator{"../data"}, boost::filesystem::directory_iterator{});
    for(auto folp=folderpaths.begin();folp!=folderpaths.end();folp++)       //遍历第一级目录
    {
        std::cout<<"cd: "<<(*folp).string()<<std::endl;
        std::vector<boost::filesystem::path> filepaths(boost::filesystem::directory_iterator{(*folp).string()}, boost::filesystem::directory_iterator{});

        for(auto filep=filepaths.begin();filep!=filepaths.end();filep++)    //遍历第二级目录
        {   
            //std::cout<<rangex<<std::endl;
            
            
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcdCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredXCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredYCloud(new pcl::PointCloud<pcl::PointXYZ>);

            if(pcl::io::loadPCDFile<pcl::PointXYZ> ((*filep).string(),*pcdCloud)==-1)
            {
                PCL_ERROR("Cont read file\n");
            }
            else
            {   
                double rangex=-95.2;
                std::cout<<"statistic: "<<(*filep).string()<<std::endl;
                //std::cout<<rangex<<" "<<rangex+0.05<<std::endl;
                
                while((rangex+0.05)<=97)
                {
                    filteredXCloud->clear();
                    pass.setInputCloud(pcdCloud);
                    pass.setFilterFieldName("x");
                    pass.setFilterLimits(rangex,rangex+0.05);
                    pass.filter(*filteredXCloud);

                    //TODO:
                    /*
                    if(filteredCloud->size()>0):
                    {
                    继续
                    }
                    else
                    {
                    开始下一次循环
                    }
                    */
                    double rangey=0;
                    while ((rangey+0.05)<=168)
                    {
                        //std::cout<<rangey<<" "<<rangey+0.05<<std::endl;
                        filteredYCloud->clear();
                        pass.setInputCloud(filteredXCloud);
                        pass.setFilterFieldName("y");
                        pass.setFilterLimits(rangey,rangey+0.05);
                        pass.filter(*filteredYCloud);

                        rangey+=0.05;

                        //std::cout<<(*filteredYCloud).size()<<std::endl;
                        int iPoinsNum=(*filteredYCloud).size();    

                        if(iPoinsNum==0)
                        {
                            continue;
                        }
                        else
                        {
                            voxels+=1;
                            voxel_points_sum+=iPoinsNum;
                            if(iPoinsNum>in_voxel_points_max)
                            {
                                in_voxel_points_max=iPoinsNum;
                            }    
                        }
                        
                    }
                    
                    rangex+=0.05;
                   
                }
           }
        }
    }

    std::cout<<">>>>>>>>>>Statistic<<<<<<<<<<"<<std::endl;
    std::cout<<"voxels: "<<voxels<<std::endl;
    std::cout<<"in_voxel_points_max: "<<in_voxel_points_max<<std::endl;
    std::cout<<"voxel_points_sum: "<<voxel_points_sum<<std::endl;
    std::cout<<"in_voxel_points_mean: "<<double(voxel_points_sum)/double(voxels)<<std::endl;
}