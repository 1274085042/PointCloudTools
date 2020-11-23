# 地面与lidar坐标系xoy面的夹角

## Ground_Segmentation  
功能：  
分割出所有文件的地面点云  

## Estimate_Normals  
功能：  
* 统计地面点云每个点的法向量，这些法向量的均值作为地面的法向量
![][image1]
* 地面的法向量与lidar坐标系xoy面的法向量夹角作为两个平面的夹角


[//]:#(image)
[image1]:./picture/normals.png