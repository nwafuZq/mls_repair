/* \author Radu Bogdan Rusu
* adaptation Raphael Favier*/
//#define BOOST_USE_WINDOWS_H
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost\thread\thread_time.hpp>
#include<boost\thread.hpp>
#include <pcl\filters\conditional_removal.h>
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\segmentation\extract_clusters.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include<pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include<pcl/registration/correspondence_estimation.h> 
#include<pcl/common/common_headers.h>
#include<vector>
#include<pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include<pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include<pcl/surface/on_nurbs/triangulation.h>
#include<pcl/console/parse.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/keypoints/uniform_sampling.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h> //边界提取
#include<pcl/console/time.h>
#include<iomanip>
#include<pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
#include<pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
#include<pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include<pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include<pcl/surface/on_nurbs/fitting_curve_2d.h>
#include<pcl/surface/on_nurbs/triangulation.h>


#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include<io.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//这是一个辅助教程，因此我们可以负担全局变量
//创建可视化工具
pcl::visualization::PCLVisualizer* p=new pcl::visualization::PCLVisualizer();
pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
//定义左右视点
int vp_1, vp_2;
//处理点云的方便的结构定义
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;
    PCD() : cloud(new PointCloud) {};
};
struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};
//以< x, y, z, curvature >形式定义一个新的点
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation()
    {
        //定义尺寸值
        nr_dimensions_ = 4;
    }
    //覆盖copyToFloatArray方法来定义我们的特征矢量
    virtual void copyToFloatArray(const PointNormalT& p, float* out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

/** 在可视化窗口的第一视点显示源点云和目标点云
*
*/
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud("vp1_target");
    p->removePointCloud("vp1_source");
    PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
    p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);
    PCL_INFO("Press q to begin the registration.\n");
    p->spin();
}

/**在可视化窗口的第二视点显示源点云和目标点云
*
*/
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud("source");
    p->removePointCloud("target");
    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
    if (!tgt_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");
    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
    if (!src_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");
    p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);
    p->spinOnce();
    //boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(50000));
}

/**加载一组我们想要匹配在一起的PCD文件
* 参数argc是参数的数量 (pass from main ())
*参数 argv 实际的命令行参数 (pass from main ())
*参数models点云数据集的合成矢量
*/
void loadData(int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD> >& models)
{
    std::string extension(".pcd");
    //假定第一个参数是实际测试模型
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string(argv[i]);
        // 至少需要5个字符长（因为.plot就有 5个字符）
        if (fname.size() <= extension.size())
            continue;
        std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);
        //检查参数是一个pcd文件
        if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
            //加载点云并保存在总体的模型列表中
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile(argv[i], *m.cloud);
            //从点云中移除NAN点
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
            models.push_back(m);
        }
    }
}

/**匹配一对点云数据集并且返还结果
*参数 cloud_src 是源点云
*参数 cloud_src 是目标点云
*参数output输出的配准结果的源点云
*参数final_transform是在来源和目标之间的转换
*/
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false)
{
    //
    //为了一致性和高速的下采样
    //注意：为了大数据集需要允许这项
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);
        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }
    //计算曲面法线和曲率
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(40);
    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);
    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
    //
    //举例说明我们自定义点的表示（以上定义）
    MyPointRepresentation point_representation;
    //调整'curvature'尺寸权重以便使它和x, y, z平衡
    float alpha[4] = { 1, 1, 1, 1 };
    point_representation.setRescaleValues(alpha);
    //
    // 配准
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    //将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
    //注意：根据你的数据集大小来调整
    reg.setMaxCorrespondenceDistance(10);
    //设置点表示
    reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);
    //
    //在一个循环中运行相同的最优化并且使结果可视化
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);
    for (int i = 0; i < 40; ++i)
    {
        PCL_INFO("Iteration Nr. %d.\n", i);
        //为了可视化的目的保存点云
        points_with_normals_src = reg_result;
        //估计
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);
        //在每一个迭代之间累积转换
        Ti = reg.getFinalTransformation() * Ti;
        //如果这次转换和之前转换之间的差异小于阈值
        //则通过减小最大对应距离来改善程序
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
        prev = reg.getLastIncrementalTransformation();
        //可视化当前状态
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
        
        
    }
    //
    // 得到目标点云到源点云的变换
    targetToSource = Ti.inverse();
    //
    //把目标点云转换回源框架
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
    p->removePointCloud("source");
    p->removePointCloud("target");
    PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
    p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
    PCL_INFO("Press q to continue the registration.\n");
    p->spin();
    p->removePointCloud("source");
    p->removePointCloud("target");
    //添加源点云到转换目标
    *output += *cloud_src;
    final_transform = targetToSource;
}

//条件滤波
void Conditional_filte(std::string dir, std::string con_dir)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 填入点云数据
    pcl::PCDReader reader;
    reader.read(dir, *cloud);

    //创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    //为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 2)));   //添加在Z字段上大于1.0的比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 5)));  //添加在Z字段上小于1.0的比较算子,远离摄像机方向

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -2)));//添加在Y字段上大于1.0的比较算子，
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 2)));  //添加在Y字段上小于1.0的比较算子，靠近地面方向

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -3)));  //添加在X字段上大于1.0的比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
        pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 1)));  //7添加在X字段上小于1.0的比较算子
                                                                                 // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond); 
    condrem.setInputCloud(cloud);                   //输入点云

    condrem.setKeepOrganized(true);               //设置保持点云的结构												  
    condrem.filter(*cloud_filtered);  // 执行滤波

    pcl::io::savePCDFileASCII(con_dir, *cloud_filtered);
}

//点云显示（filename1为滤波前点云，filename2为滤波后点云）
int show_cloudd(char dir[], char dir_filter[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data  

    FILE* stream1, * stream2;
    stream1 = fopen(dir, "r");
    stream2 = fopen(dir_filter, "r");
    if (stream2 == NULL || stream2 == NULL)
        printf("The file was not opened\n");
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile(dir, *cloud))
        {
            std::cout << "error";
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile(dir_filter, *cloud_filtered))
        {
            std::cout << "error";
        }

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->initCameraParameters();
        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor(1, 1, 1, v1);//前三个参数为RGB值
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 100, 100, 100);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color,"befor filter", v1);
       // viewer->addPointCloud<pcl::PointXYZ>(cloud, "befor filter", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor(0, 0, 0, v2);//前三个参数为RGB值
        viewer->addText("Radius: 0.1", 20, 20, "v2 text", v2);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "after filter", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor filter");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after filter");
        //viewer->addCoordinateSystem(0.2);

        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    return 0;
}

//随机采样一致性分割
void RANSACc(char dir[], char ran_dir[])
{
    int o = 0;
    if(o==1)
    {  pcl::PCDReader reader;
    //pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(dir, *cloud);

    //创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);//SACMODEL_PARALLEL_PLANE
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    float angle = 5;
    float epsangle = pcl::deg2rad(angle);
    Eigen::Vector3f axis(0.0, 0.0, 1.0);
    seg.setAxis(axis);
    seg.setEpsAngle(epsangle);
    seg.setInputCloud(cloud);

    int i = 0, nr_points = (int)cloud->points.size();
    if (cloud->points.size() > 0.5 * nr_points)
    {

        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        extract.setNegative(true);
        extract.filter(*cloud_f);
        //cloud = cloud_f;
        std::cout << "PointCloud After RANSAC: " << cloud_f->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "Standard_" << dir << ".pcd";
        pcl::io::savePCDFileASCII(ran_dir, *cloud_f);
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_f, false);
    }
    }
    else {

       
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud) == -1)
            {
                PCL_ERROR("点云读取失败 \n");
               
            }

            //------------------------------------------RANSAC框架--------------------------------------------------------   
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);//定义RANSAC算法模型
            ransac.setDistanceThreshold(0.03);//设定距离阈值
            ransac.setMaxIterations(3300);     //设置最大迭代次数
            ransac.setProbability(0.99);      //设置从离群值中选择至少一个样本的期望概率
            ransac.computeModel();            //拟合平面
            std::vector<int> inliers;              //用于存放内点索引的vecto
            ransac.getInliers(inliers);       //获取内点索引

            Eigen::VectorXf coeff;
            ransac.getModelCoefficients(coeff);  //获取拟合平面参数，coeff分别按顺序保存a,b,c,d

            cout << "平面模型系数coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << endl;
            /*
             //-------------------平面法向量定向，与（1，1，1）同向，并输出平面与原点的距离D---------------------------
             double a, b, c, d, A, B, C, D;//a,b,c为拟合平面的单位法向量，A,B,C为重定向后的法向量
             a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];

             if (a + b + c > 0) {
                 A = a;
                 B = b;
                 C = c;
                 D = abs(d);
             }
             else {
                 A = -a;
                 B = -b;
                 C = -c;
                 D = abs(d);
             }
             cout << "" << A << ",\t" << "" << B << ",\t" << "" << C << ",\t" << "" << D << ",\t" << endl;
             */

             //--------------------------------根据内点索引提取拟合的平面点云-----------------------------------
            pcl::PointCloud<pcl::PointXYZ>::Ptr sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);
           // pcl::io::savePCDFileASCII(ran_dir, *sac_plane);
            std::shared_ptr<std::vector<int>> index_ptr = std::make_shared<std::vector<int>>(inliers);
            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            // Extract the inliers
            extract.setInputCloud(cloud);
            extract.setIndices(index_ptr);
            extract.setNegative(true);//如果设为true,可以提取指定index之外的点云
            extract.filter(*final);
           
            pcl::io::savePCDFileASCII(ran_dir, *final);
            
            //-------------------------------------------可视化-------------------------------------------------
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
            int v1 = 0;
            int v2 = 1;

            viewer->createViewPort(0, 0, 0.5, 1, v1);
            viewer->createViewPort(0.5, 0, 1, 1, v2);
            viewer->setBackgroundColor(0, 0, 0, v1);
            viewer->setBackgroundColor(0, 0, 0, v2);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(sac_plane, 0, 0, 255);

            viewer->addPointCloud(cloud, color, "cloud", v1);
            viewer->addPointCloud(sac_plane, after_sac, "plane cloud", v2);
            /*
            // 显示拟合出来的平面
            pcl::ModelCoefficients plane;
            plane.values.push_back(coeff[0]);
            plane.values.push_back(coeff[1]);
            plane.values.push_back(coeff[2]);
            plane.values.push_back(coeff[3]);

            viewer->addPlane(plane, "plane",v2);
            */

            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(10000));
            }
            pcl::io::savePCDFileASCII(ran_dir, *final);


    }
}

//欧式聚类
int EuclideanClusterr(char dir[],char dir1[],int flag)
{
    if(flag==0)
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        //reader.read(dir, *cloud);

        // 加载点云文件  C:\Users\lenovo\Desktop\date_mao\biaoqian
        pcl::io::loadPCDFile(dir, *cloud);


        //处理前点云数量
        std::cerr << "PointCloud before Euclidean clustering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

        //为提取点云时使用的搜索对象利用输入点云创建kd树对象tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05); // 2cm  上采样时0.025 分离时0.05
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;
        pcl::PCDWriter writer;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
            std::stringstream ss;
            //  ss << dir1<< j << ".pcd";
            if (j == 0) {
                pcl::io::savePCDFileASCII(dir1, *cloud_cluster);
            }
            j++;
        }
        return j;
    }
    else {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        //reader.read(dir, *cloud);

        // 加载点云文件  C:\Users\lenovo\Desktop\date_mao\biaoqian
        pcl::io::loadPCDFile(dir, *cloud);


        //处理前点云数量
        std::cerr << "PointCloud before Euclidean clustering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

        //为提取点云时使用的搜索对象利用输入点云创建kd树对象tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.025); // 2cm  上采样时0.025 分离时0.05
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;
        pcl::PCDWriter writer;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
            std::stringstream ss;
            //  ss << dir1<< j << ".pcd";
            if (j == 0) {
                pcl::io::savePCDFileASCII(dir1, *cloud_cluster);
            }
            j++;
        }
        return j;
    }
}

//体素滤波
void VG_filtere(char dir[], char vg_dir[],float leafx,float leafy,float leafz)//体素滤波
{
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr VG_cloud_filtered_blob(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr VG_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 填入点云数据
    pcl::PCDReader reader;
    reader.read(dir, *cloud_blob);
    std::cerr << "PointCloud before Voxel Grid Removal filtering: " << cloud_blob->width * cloud_blob->height
        << " data points (" << pcl::getFieldsList(*cloud_blob) << ")." << std::endl;
    // 创建滤波器对象
    pcl::VoxelGrid<pcl::PCLPointCloud2> VG_sor;
    VG_sor.setInputCloud(cloud_blob);
    VG_sor.setLeafSize(leafx,leafy,leafz);
    VG_sor.filter(*VG_cloud_filtered_blob);
    std::cerr << "PointCloud after filtering: " << VG_cloud_filtered_blob->width * VG_cloud_filtered_blob->height
        << " data points (" << pcl::getFieldsList(*VG_cloud_filtered_blob) << ")." << std::endl;

    // 转换为模板点云
    pcl::fromPCLPointCloud2(*VG_cloud_filtered_blob, *VG_cloud_filtered);
    pcl::io::savePCDFileASCII(vg_dir, *VG_cloud_filtered);
}

//贪心投影
void PCD_GP3D(char dir[], char dir1[]) {
    // Load input file into a PointCloud<T> with an appropriate type
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud) == -1)//打开点云文件
    {    
            PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //文件名要写对
    }

    //sensor_msgs::PointCloud2 cloud_blob;  
    //pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    //pcl::fromROSMsg (cloud_blob, *cloud);
    //* the data should be available in cloud
    
    time_t begin, end;
    
    // Normal estimation（法向量估计）
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);  //法线
    //* normals should not contain the point normals + surface curvatures（不能同时包含点的法向量和表面的曲率）


    // Concatenate the XYZ and normal fields （将点云和法线放在一起）
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);


    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // 曲面重建
    //pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
    //mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
    //mls.setInputCloud(cloud_with_normals);//设置参数
    //mls.setPolynomialFit(true);
    //mls.setSearchMethod(tree2);
    //mls.setSearchRadius(0.1);
   // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_msl(new pcl::PointCloud<pcl::PointNormal>);
    //mls.process(*cloud_with_normals_msl);
   // cloud_with_normals = cloud_with_normals_msl;
    //std::cerr << "曲面重建   完成" << std::endl;


    // Initialize objects （初始化对象）
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;  //创建多边形网格，用于存储结果

    //设置参数
    gp3.setSearchRadius(5);  //  设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径（默认为0）
    gp3.setMu(5);  //2.5  设置最近邻距离的乘子，已得到每个点的最终搜索半径（默认为0）
    gp3.setMaximumNearestNeighbors(1000);  //设置搜索的最近邻点的最大数量
    gp3.setMaximumSurfaceAngle(M_PI / 1); // 45 degrees 最大平面角
    gp3.setMinimumAngle(M_PI / 42); //18  10 degrees 每个三角的最小角度
    gp3.setMaximumAngle(5 * M_PI / 3); //3  120 degrees
    gp3.setNormalConsistency(false);  //若法向量一致，设为true

    // 设置搜索方法和输入点云
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);

    //执行重构，结果保存在triangles中
    begin = clock();
    gp3.reconstruct(triangles);
    end = clock();
    double Times = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "time:" << Times << "s" << endl;
    //保存网格图

    pcl::io::savePLYFile(dir1, triangles);   //这句可以没有

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    //显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  //设置背景
    viewer->addPolygonMesh(triangles, "my");  //设置显示的网格
    viewer->addCoordinateSystem(0.01);  //设置坐标系
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
            viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    // Finish
   
}

//移动立方体
void PCD_cube() {
    // 确定文件格式
    //根据文件格式选择输入方式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
   
    if (pcl::io::loadPCDFile("Upmls.pcd", *cloud) == -1) {
            PCL_ERROR("Could not read pcd file!\n");
           
        }
    

    // 估计法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals); //计算法线，结果存储在normals中
    //* normals 不能同时包含点的法向量和表面的曲率

    //将点云和法线放到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals


    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //初始化MarchingCubes对象，并设置参数
    pcl::MarchingCubes<pcl::PointNormal>* mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    /*
  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else
  {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }
    */

    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;

    //设置MarchingCubes对象的参数
    mc->setIsoLevel(0.0f);
    mc->setGridResolution(100, 100, 100);
    mc->setPercentageExtendGrid(0.0f);

    //设置搜索方法
    mc->setInputCloud(cloud_with_normals);

    //执行重构，结果保存在mesh中
    mc->reconstruct(mesh);

    //保存网格图
    pcl::io::savePLYFile("result.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); //设置背景
    viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
    viewer->addCoordinateSystem(0.01); //设置坐标系
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
   
}

//泊松
void PCD_possion() {
    char tmpStr[100]= "13.pcd";
   
    char* pext = strrchr(tmpStr, '.');
    std::string extply("ply");
    std::string extpcd("pcd");
    if (pext) {
        *pext = '\0';
        pext++;
    }


    //根据文件格式选择输入方式  "down.pcd"  "16.pcd"
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
 
    if (pcl::io::loadPCDFile("UPmls.pcd", *cloud) == -1) {
            PCL_ERROR("Could not read pcd file!\n");
          
    }
   

    // 计算法向量
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals); //计算法线，结果存储在normals中

    //将点云和法线放到一起
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    //创建Poisson对象，并设置参数
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    //pn.setIndices();

    //设置搜索方法和输入点云
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);

     
    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;
    //执行重构
    pn.performReconstruction(mesh);

    //保存网格图
    //pcl::io::savePLYFile("result.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}


//点云重合率计算  para1 = 0.5   para2 =  0.15  
void calaPointCloudCoincide(PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_target, float para1, float para2, float& coincide)

{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_target);

    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

    core.determineReciprocalCorrespondences(*cor, para1);   //点之间的最大距离,cor对应索引

    //构造重叠点云的PCD格式文件
    PointCloud overlapA;
    PointCloud overlapB;

    overlapA.width = cor->size();
    overlapA.height = 1;
    overlapA.is_dense = false;
    overlapA.points.resize(overlapA.width * overlapA.height);

    overlapB.width = cor->size();
    overlapB.height = 1;
    overlapB.is_dense = false;
    overlapB.points.resize(overlapB.width * overlapB.height);
    cout << "点云原来的数量：" << cloud_target->size() << endl;
    cout << "重合的点云数： " << cor->size() << endl;
    double total = cor->size();
    cout << para1<<"搜索距离下)重合率： " << float( total / cloud_target->size()) * 100 << "%" << endl;
    double num = 0;
    for (size_t i = 0; i < cor->size(); i++)
    {
        //overlapA写入pcd文件
        overlapA.points[i].x = cloud_src->points[cor->at(i).index_query].x;
        overlapA.points[i].y = cloud_src->points[cor->at(i).index_query].y;
        overlapA.points[i].z = cloud_src->points[cor->at(i).index_query].z;

        //overlapB写入pcd文件
        overlapB.points[i].x = cloud_target->points[cor->at(i).index_match].x;
        overlapB.points[i].y = cloud_target->points[cor->at(i).index_match].y;
        overlapB.points[i].z = cloud_target->points[cor->at(i).index_match].z;

        double dis = sqrt(pow(overlapA.points[i].x - overlapB.points[i].x, 2) +
            pow(overlapA.points[i].y - overlapB.points[i].y, 2) +
            pow(overlapA.points[i].z - overlapB.points[i].z, 2));
        if (dis < para2)
            num++;
    }
    pcl::PCDWriter writer1;
    writer1.write<pcl::PointXYZ>("10_A.pcd", overlapA, false);
    pcl::PCDWriter writer2;
    writer2.write<pcl::PointXYZ>("10_B.pcd", overlapB, false);

    //cout << "精配重叠区域的点云数：" << num << endl;
    //cout << "重合率： " << float(num / cor->size()) * 100 << "%" << endl;
    //coincide = float(num / cor->size());

}

//mls进行下采样去除重复点
void mls_donwPcd(char dir[],char dir1[])
{
        // 将一个适当类型的输入文件加载到对象PointCloud中
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // 加载bun0.pcd文件，加载的文件在 PCL的测试数据中是存在的 
        pcl::io::loadPCDFile(dir, *cloud);
        cout << "原始点云个数：" << cloud->points.size() << endl;
        // 创建一个KD树
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
        pcl::PointCloud<pcl::PointNormal> mls_points;
        // 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        mls.setComputeNormals(true);
        //设置参数
        mls.setInputCloud(cloud);
        mls.setPolynomialOrder(true); //mls.setPolynomialFit(true); debug版
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.1);
        // 曲面重建
        mls.process(mls_points);
        cout << "下采样后点云的个数：" << mls_points.size() << endl;
        // 保存结果
        //pcl::PCDWriter writer2;
        //writer2.write<pcl::PointXYZ>("_B.pcd", mls_points, false);
        pcl::io::savePCDFile(dir1, mls_points);

         //---------显示点云-----------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(dir1, *cloud1);
      
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer(new pcl::visualization::PCLVisualizer("显示点云"));
        int v1(0), v2(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText("point clouds", 10, 10, "v1_text", v1);
        viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
        viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
        viewer->addText("Downsampled point clouds", 10, 10, "v2_text", v2);

        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud1, "cloud_up", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "sample cloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_up", v2);
        //viewer->addCoordinateSystem(1.0);
        //viewer->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
}

//利用mls进行上采样增加点数量  1代表均匀修复，0代表局部修复
void mls_upPcd(char dir[],char dir1[],int flag)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
        cout << "原始点云个数：" << cloud->points.size() << endl;
        // ---------------创建上采样对象-----------------
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> up;
        up.setInputCloud(cloud);
        //----------------建立搜索对象-----------------
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        if (flag == 1) {
            up.setSearchMethod(tree);
            //--------设置搜索邻域的半径-------------------
            up.setSearchRadius(0.2);
            up.setSqrGaussParam(0.01);

            up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
             up.setPointDensity(20000); //背部使用20000较好 底部使用3000足以
              // ----------设置采样的半径----------------
           // up.setUpsamplingRadius(0.05);//背部使用0.5
            // -------采样步长的大小-------------
           // up.setUpsamplingStepSize(0.02); //背部使用0.02

            up.process(*cloud_up);
            pcl::io::savePCDFileASCII(dir1, *cloud_up);

            cout << "上采样后点云的个数：" << cloud_up->points.size() << endl;
            //---------显示点云-----------------------
            boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("显示点云"));

            int v1(0), v2(0);
            viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
            viewer->setBackgroundColor(0, 0, 0, v1);
            viewer->addText("point clouds", 10, 10, "v1_text", v1);
            viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
            viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
            viewer->addText("Upsampled point clouds", 10, 10, "v2_text", v2);

            viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_up, "cloud_up", v2);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "sample cloud", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_up", v2);
            //viewer->addCoordinateSystem(1.0);
            //viewer->initCameraParameters();
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }
        else {
            up.setSearchMethod(tree);
            //--------设置搜索邻域的半径-------------------
            up.setSearchRadius(0.2);
            up.setSqrGaussParam(0.01);

            up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
            // up.setPointDensity(3000); //背部使用20000较好 底部使用3000足以
              // ----------设置采样的半径----------------
            up.setUpsamplingRadius(0.05);//背部使用0.5
            // -------采样步长的大小-------------
            up.setUpsamplingStepSize(0.02); //背部使用0.02

            up.process(*cloud_up);
            pcl::io::savePCDFileASCII(dir1, *cloud_up);

            cout << "上采样后点云的个数：" << cloud_up->points.size() << endl;
            //---------显示点云-----------------------
            boost::shared_ptr<pcl::visualization::PCLVisualizer>
                viewer(new pcl::visualization::PCLVisualizer("显示点云"));

            int v1(0), v2(0);
            viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
            viewer->setBackgroundColor(0, 0, 0, v1);
            viewer->addText("point clouds", 10, 10, "v1_text", v1);
            viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
            viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
            viewer->addText("Upsampled point clouds", 10, 10, "v2_text", v2);

            viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_up, "cloud_up", v2);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "sample cloud", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_up", v2);
            //viewer->addCoordinateSystem(1.0);
            //viewer->initCameraParameters();
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }
}

//均匀采样，降低样本数量
void avg_mls(char dir[],char dir1[]) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);

    std::cout << "原始点云个数：" << cloud->points.size() << endl;
    // ----------------创建均匀采样对象-------------------------
    pcl::UniformSampling<pcl::PointXYZ> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.005f);// 设置滤波时创建球体的半径
    US.filter(*cloud_filtered);
    cout << "均匀采样之后点云的个数：" << cloud_filtered->points.size() << endl;
    //---------------------显示点云-----------------------
    pcl::io::savePCDFileASCII(dir1, *cloud_filtered);
}


void PointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d& data)
{
        for (unsigned i = 0; i < cloud->size(); i++)
        {
            pcl::PointXYZ& p = cloud->at(i);
            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
       }
}


void
PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data)
{
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZ& p = cloud->at(i);
        if (!std::isnan(p.x) && !std::isnan(p.y))
            data.push_back(Eigen::Vector2d(p.x, p.y));
    }
}


void visualizeCurve(ON_NurbsCurve& curve, ON_NurbsSurface&surface, pcl::visualization::PCLVisualizer& viewer)
{
   
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
        for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
        {
            pcl::PointXYZRGB& p1 = curve_cloud->at(i);
            pcl::PointXYZRGB& p2 = curve_cloud->at(i + 1);
            std::ostringstream os;
            os << "line" << i;
            viewer.removeShape(os.str());
            viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int i = 0; i < curve.CVCount(); i++)
        {
            ON_3dPoint p1;
            curve.GetCV(i, p1);

            double pnt[3];
            surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
            pcl::PointXYZRGB p2;
            p2.x = float(pnt[0]);
            p2.y = float(pnt[1]);
            p2.z = float(pnt[2]);

            p2.r = 255;
            p2.g = 0;
            p2.b = 0;

            curve_cps->push_back(p2);
        }
        viewer.removePointCloud("cloud_cps");
        viewer.addPointCloud(curve_cps, "cloud_cps");

}

void pcd_Nurbs()
{

    pcl::visualization::PCLVisualizer viewer("PCL");
    viewer.setBackgroundColor(255, 255, 255);

    //--------------加载点云-----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud2;
    pcl::on_nurbs::NurbsDataSurface data;

    if (pcl::io::loadPCDFile("UPmls.pcd", cloud2) == -1)
        throw std::runtime_error("  PCD file not found.");

    fromPCLPointCloud2(cloud2, *cloud);
    PointCloud2Vector3d(cloud, data.interior);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");
    printf("  %lu points in data set\n", cloud->size());

    //-----------B样条曲面重建------------------------

    // -----B样条曲面拟合的参数-----------------------
    unsigned order(3);//B样条曲面的模型多项式的阶数
    unsigned refinement(5);//拟合优化的迭代次数
    unsigned iterations(50);//完成拟合优化后的迭代次数
    unsigned mesh_resolution(128);//每个参数方向上的采样点个数，用于对拟合得到的B样条曲面进行三角化
    bool two_dim = true;

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;//描述曲面本身的平滑性
    params.interior_weight = 1.0;//拟合优化时用到的权重
    params.boundary_smoothness = 0.2;//曲面边界（非裁剪边界）的平滑性
    params.boundary_weight = 1.0;//优化时的边界权重

    // --------初始化B样条曲面----------------------
    printf("  surface fitting ...\n");
    //构造局部坐标系
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);
    fit.setQuiet(false); //设置是否打印调试信息

  // ----------可视化曲面模型---------------------
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);//可视化初始化的B样条曲面
    std::cout << "Before refine" << endl;
    viewer.spinOnce(3000);
    //----------- 表面精细化处理---------------------
    for (unsigned i = 0; i < refinement; i++)//每次迭代都会添加控制点数目
    {
        fit.refine(0);           //设置在参数方向0上添加控制点优化
        if (two_dim)fit.refine(1);// 设置在参数方向1上添加控制点优化
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        std::cout << "refine: " << i << endl;
    }

    //----------以最终优化确定的控制点数量来进行多次迭代拟合求解-----------
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        std::cout << "iterations: " << i << endl;
    }

    // ----------------------拟合B样条曲线-------------------------

    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;//距离阈值，如果曲线的支撑域到最近数据点的距离大于该阈值，则添加一控制点
    curve_params.addCPsIteration = 10;  //不进行控制点插入时的内部迭代优化次数
    curve_params.maxCPs = 2000;         //允许的最大控制点个数
    curve_params.accuracy = 3;      //曲线的平均拟合精度
    curve_params.iterations = 20;     //最大迭代次数

    curve_params.param.closest_point_resolution = 0;//每一个支撑域内控制点的个数
    curve_params.param.closest_point_weight = 1.0;//最近点对应的权重
    curve_params.param.closest_point_sigma2 = 0.1;//外点的最近点阈值，拟合时不考虑远离于曲线的外点的距离值大于该点的值
    curve_params.param.interior_sigma2 = 0.00001; //内点的最近点阈值，拟合时不考虑远离于曲线的内点的距离值大于该点的值
    curve_params.param.smooth_concavity = 1.0;    //平滑凹凸性，该值使曲线向内或外凹（=0没用，<0向内凹，>0向外凹）
    curve_params.param.smoothness = 1.0;          //平滑项的权重

    // 用最小的控制点个数表示的一个圆来初始化该拟合曲线
    printf("  curve fitting ...\n");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back(true);//设置进行带权重的B样条曲线拟合
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

    //进行曲线拟合并可视化
    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // 设置是否打印调试信息
    curve_fit.fitting(curve_params);
    visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

    //-------------------裁剪B样条曲面-----------------------
    printf("  triangulate trimmed surface ...\n");
    viewer.removePolygonMesh(mesh_id);
    //对B样条曲面进行三角化，并根据B样条曲线对属于外部的三角形进行裁剪，
    //对于裁剪掉的三角形与B样条曲线相交处用曲线表示
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
        mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);


    // ------------保存已修剪的B样条曲面--------------------------
    if (fit.m_nurbs.IsValid())
    {
        ONX_Model model;
        ONX_Model_Object& surf = model.m_object_table.AppendNew();
        surf.m_object = new ON_NurbsSurface(fit.m_nurbs);
        surf.m_bDeleteObject = true;
        surf.m_attributes.m_layer_index = 1;
        surf.m_attributes.m_name = "surface";

        ONX_Model_Object& curv = model.m_object_table.AppendNew();
        curv.m_object = new ON_NurbsCurve(curve_fit.m_nurbs);
        curv.m_bDeleteObject = true;
        curv.m_attributes.m_layer_index = 2;
        curv.m_attributes.m_name = "trimming curve";

       // model.Write("136.ply");

    }

    printf("  ... done.\n");

    viewer.spin();
 
}

void pcd_ashape(char dir[],char dir1[])
{
        
    time_t begin, end;
    
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::io::loadPCDFile<pcl::PointNormal>(dir, *cloud);
        pcl::PointCloud<pcl::PointNormal>::Ptr surface_hull(new pcl::PointCloud<pcl::PointNormal>);
        pcl::ConcaveHull<pcl::PointNormal> cavehull;
       
        cavehull.setInputCloud(cloud);
        cavehull.setAlpha(0.03);
        std::vector<pcl::Vertices> polygons;
        cavehull.reconstruct(*surface_hull, polygons);// 重建面要素到点云

        pcl::PolygonMesh mesh;
        begin = clock();
        cavehull.reconstruct(mesh);// 重建面要素到mesh
        end = clock();
        pcl::io::savePLYFile(dir1, mesh);
        
        cerr << "Concave hull has: " << surface_hull->points.size()
            << " data points." << endl;
        double Times = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "time:" << Times << "s" << endl;
        pcl::PCDWriter writer;
       // writer.write("hull.pcd", *surface_hull, false);
        // 可视化
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("hull"));
        viewer->setWindowName("alshape曲面重构");
        viewer->addPolygonMesh<pcl::PointNormal>(surface_hull, polygons, "polyline");
        viewer->spin();

    


}

void readply(char dir[],char dir1[])
{

        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(dir, mesh);
        pcl::PolygonMesh mesh1;
        pcl::io::loadPLYFile(dir1, mesh1);

        //-----------------点云模型可视化----------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

        int v1(0), v2(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText("ply", 10, 10, "v1_text", v1);
        viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
        viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
        viewer->addText("ply_mls", 10, 10, "v2_text", v2);

        viewer->addPolygonMesh(mesh, "cloud", v1);
        viewer->addPolygonMesh(mesh1, "cloud_mls", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_mls", v2);
        //viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

       // viewer->setBackgroundColor(0, 0, 0);
       // viewer->setWindowName(dir);
       // viewer->addPolygonMesh(mesh, "my");
        //设置网格模型显示模式
       //viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
       //viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
       //viewer->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示

       // viewer->addCoordinateSystem(0.2);
        //viewer->initCameraParameters ();
       // while (!viewer->wasStopped())
       // {
       //     viewer->spinOnce(100);
       //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
       // }

}

void div_pcd(char dir[],char dir1[],char dir2[])
{
        //初始化点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
   
        // 加载点云文件  C:\Users\lenovo\Desktop\date_mao\biaoqian
        pcl::io::loadPCDFile(dir, *source_cloud);
        //设置点云为500个，也可以自行修改，或者直接读取PCD文件    
        //cloud->width = 500;
        //cloud->height = 1;
        //cloud->points.resize(cloud->width * cloud->height);

       // for (size_t i = 0; i < cloud->points.size(); ++i)
        //{
        //    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        //    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        //    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        //}
       // writer.write<pcl::PointXYZ>(dir, *cloud, false);

        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(new pcl::ConditionAnd<pcl::PointXYZ>());
        //设置作用域为z，取大于0且小于0.8的位置，保留在点云中，其余进行移除
        range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -1)));
        range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -0.28)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condream;
        condream.setCondition(range_cloud);
        condream.setInputCloud(source_cloud);
        condream.setKeepOrganized(true);
        condream.filter(*cloud_filtered);
        pcl::io::savePCDFileASCII(dir1, *cloud_filtered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud1(new pcl::PointCloud<pcl::PointXYZ>());

        // 加载点云文件  C:\Users\lenovo\Desktop\date_mao\biaoqian
        pcl::io::loadPCDFile(dir, *source_cloud1);
     
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud1(new pcl::ConditionAnd<pcl::PointXYZ>());
        //设置作用域为z，取大于0且小于0.8的位置，保留在点云中，其余进行移除
        range_cloud1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.25)));
        range_cloud1->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 3)));
        pcl::ConditionalRemoval<pcl::PointXYZ> condream1;
        condream1.setCondition(range_cloud1);
        condream1.setInputCloud(source_cloud1);
        condream1.setKeepOrganized(true);
        condream1.filter(*cloud_filtered1);
        pcl::io::savePCDFileASCII(dir2, *cloud_filtered1);

}

void PlyToPcd(char dir[],char dir1[])
{
    pcl::PCLPointCloud2 point_cloud2;
    pcl::PLYReader reader;
    reader.read(dir, point_cloud2);
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromPCLPointCloud2(point_cloud2, point_cloud);
    pcl::PCDWriter writer;
    writer.writeASCII(dir1, point_cloud);
    cout << "Done!" << endl;
   
}

void PcdToPly(char dir[])
{
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(dir, cloud) < 0)
    {
        cout << "Error: cannot load the PCD file!!!" << endl;
     
    }
    pcl::PLYWriter writer;
    writer.writeASCII("1.ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
    cout << "Done!" << endl;
}

//合并两个ply文件
int contenate_ply(char dir[],char dir1[],char dir2[],char dir3[],char dir4[])
{
    pcl::PolygonMesh mesh1;
    pcl::io::loadPLYFile(dir, mesh1);
    pcl::PolygonMesh mesh2;
    pcl::io::loadPLYFile(dir2, mesh2);
    pcl::PolygonMesh  mesh3;
    mesh3.concatenate(mesh1, mesh2, mesh3);
    pcl::io::savePLYFile(dir4, mesh3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(dir, *cloud1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(dir2, *cloud2);
    int sum = cloud1->points.size();
    cout << "point_1_number:" << sum << endl;
    rename(dir, dir1);

    fstream fin(dir1, ios::in);
    if (!fin)
    {
        cerr << "can not open file" << endl;
        return -1;
    }

    char c1;
    int lineCnt = 0;
    while (fin.get(c1))
    {
        if (c1 == '\n')
            lineCnt++;
    }
    cout << lineCnt + 1 + cloud2->points.size() << endl;
    int total = lineCnt + 1 + cloud2->points.size();
    fin.close();
    rename(dir1, dir);
    rename(dir4, dir3);
    fstream fin1(dir3, ios::in);
    if (!fin1)
    {
        cerr << "can not open file" << endl;
        return -1;
    }
    char cc;
    lineCnt = 0;
    while (fin1.get(cc))
    {
        if (cc == '\n')
            lineCnt++;
    }
    cout << lineCnt + 1 << endl;
    int txttotal = lineCnt + 1;
    fin1.close();
    std::queue<int> a;
    std::queue<int> b;
    std::queue<int> c;
    std::queue<int> d;

    ifstream f;//读权限变量 f
    std::queue<std::string> r;//用于存储
    f.open(dir3);//文件text需要存放在当前cpp文件的根目录中
    std::string linee, s;
    int txtLine = 1;
    while (getline(f, linee))        //从文件中读取一行存放在line中
    {
        int  aaa, bbb, ccc, ddd;
        r.push(linee);
        if (txtLine >= total && txtLine < txttotal)
        {
            std::istringstream is(linee);
            is >> aaa >> bbb >> ccc >> ddd;
           // cout << aaa << " " << bbb << " " << ccc << " " << ddd << endl;
            a.push(aaa);
            b.push(bbb);
            c.push(ccc);
            d.push(ddd);
        }
        txtLine++;
    }
    ifstream in;
    in.open(dir3);
    std::string strFileData = "";
    int line = 1;
    char tmpLineData[1024] = { 0 };
    while (in.getline(tmpLineData, sizeof(tmpLineData)))
    {
        if (line >= total && line < txttotal)
        {
            strFileData += std::string("3 " + std::to_string(b.front() - cloud2->points.size()) + " " + std::to_string(c.front() - cloud2->points.size()) + " " + std::to_string(d.front() - cloud2->points.size()));
            strFileData += "\n";
            b.pop();
            c.pop();
            d.pop();
        }
        else
        {
            strFileData += std::string(tmpLineData);
            strFileData += "\n";
        }
        line++;
    }
    in.close();
    //写入文件
    ofstream out;
    out.open(dir3);
    out.flush();
    out << strFileData;
    out.close();
    rename(dir3, dir4);
    cout << "Done!" << endl;
}

void Getboundary(char dir[], char dir1[])
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(dir, *cloud);
        cout << "加载点云" << cloud->points.size() << "个" << endl;

        //------------------------计算法向量---------------------------
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setRadiusSearch(30);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.compute(*normals);
        //-----------------------边界特征估计--------------------------
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
        boundEst.setInputCloud(cloud);
        boundEst.setInputNormals(normals);
        boundEst.setRadiusSearch(50);
        boundEst.setAngleThreshold(M_PI/4 );//边界判断时的角度阈值

        boundEst.setSearchMethod(tree);
        pcl::PointCloud<pcl::Boundary> boundaries;
        boundEst.compute(boundaries);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->points.size(); i++)
        {

            if (boundaries[i].boundary_point > 0)
            {
                cloud_boundary->push_back(cloud->points[i]);
            }
        }
        cout << "边界点个数:" << cloud_boundary->points.size() << endl;
        pcl::io::savePCDFileASCII(dir1, *cloud_boundary);

        //-------------------------可视化-----------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
        int v1(0);
        viewer->setWindowName("边界提取");
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
        viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
        viewer->addText("Boudary point clouds", 10, 10, "v2_text", v2);

        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
        //view->addCoordinateSystem(1.0);
        //view->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

      
}

static bool compare_y(pcl::PointXYZ a, pcl::PointXYZ b)
{
	return (a.y < b.y);
}

float getMiddleZ(char dir[]) {


	// ---------------------------加载点云数据--------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    float middleZ=0;
    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
        middleZ += cloud->points[nIndex].z;
        
    }
    middleZ = middleZ / cloud->points.size();
    cout << setiosflags(ios::fixed) << setprecision(2)  << "Z中值点的坐标为: " << middleZ << endl;
    return middleZ;
}

float getMiddleY(char dir[]) {


    // ---------------------------加载点云数据--------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    float middleZ = 0;
    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
        middleZ += cloud->points[nIndex].y;

    }
    middleZ = middleZ / cloud->points.size();
    cout << setiosflags(ios::fixed) << setprecision(2) << "y中值点的坐标为: " << middleZ << endl;
    return middleZ;
}

float round(float number, unsigned int bits) {
    std::stringstream ss;
    ss << setiosflags(ios::fixed) <<setprecision(bits) << number;
    ss >> number;
    return number;
}


void convertPcd(char dir[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    for(int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {

        cloud->points[nIndex].x=round(cloud->points[nIndex].x, 2);
        cloud->points[nIndex].y = round(cloud->points[nIndex].y, 2);
        cloud->points[nIndex].z = round(cloud->points[nIndex].z, 2);
        cout << cloud->points[nIndex].x << " " << cloud->points[nIndex].y << " " << cloud->points[nIndex].z << endl;
    }
    pcl::io::savePCDFileASCII(dir, *cloud);
}

void equalToZ(char dir[],char dir1[],float middleZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    int indexZ = 0;
    cloud_Z->width = cloud->width;
    cloud_Z->height = 1;
    cloud_Z->points.resize(cloud_Z->width * cloud_Z->height);
    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
        if ((std::abs(cloud->points[nIndex].z - middleZ) <= 0.001) == 1)
        {
            cloud_Z->points[indexZ].x = cloud->points[nIndex].x;
            cloud_Z->points[indexZ].y = cloud->points[nIndex].y;
            cloud_Z->points[indexZ].z = cloud->points[nIndex].z;
            indexZ++;
            }
    }
    cloud_Z->width = indexZ;
    cloud_Z->height = 1;
    cloud_Z->points.resize(cloud_Z->width * cloud_Z->height);
    pcl::io::savePCDFileASCII(dir1, *cloud_Z);
   
}

void equalToY(char dir[], char dir1[], float middleZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    int indexZ = 0;
    cloud_Z->width = cloud->width;
    cloud_Z->height = 1;
    cloud_Z->points.resize(cloud_Z->width * cloud_Z->height);
    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
        if ((std::abs(cloud->points[nIndex].y - middleZ) <= 0.0005) == 1)
        {
            cloud_Z->points[indexZ].x = cloud->points[nIndex].x;
            cloud_Z->points[indexZ].y = cloud->points[nIndex].y;
            cloud_Z->points[indexZ].z = cloud->points[nIndex].z;
            indexZ++;
        }
    }
    cloud_Z->width = indexZ;
    cloud_Z->height = 1;
    cloud_Z->points.resize(cloud_Z->width * cloud_Z->height);
    pcl::io::savePCDFileASCII(dir1, *cloud_Z);

}


void
VisualizeCurveNURBS(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 30);
    
    for (std::size_t i = 0; i < cloud->size() - 1; i++)
    {
        pcl::PointXYZRGB& p1 = cloud->at(i);
        pcl::PointXYZRGB& p2 = cloud->at(i + 1);
        std::ostringstream os;
        os << "line_" << r << "_" << g << "_" << b << "_" << i;
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());

    }

    if (show_cps)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < curve.CVCount(); i++)
        {
            ON_3dPoint cp;
            curve.GetCV(i, cp);

            pcl::PointXYZ p;
            p.x = float(cp.x);
            p.y = float(cp.y);
            p.z = float(cp.z);
            cps->push_back(p);
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
        pcl::io::savePCDFileASCII("NURBS.pcd", *cps);
        
      

    }
}

void NURBS_curve(char dir[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(dir, *cloud);
 

    // 变换为xoy平面上的2D原始点云（PCL中定义的2D曲线拟合方法默认在xoy平面方向曲线拟合）
    pcl::PointCloud<pcl::PointXYZ>::iterator it_1;
    for (it_1 = cloud->begin(); it_1 != cloud->end();)
    {
        float x = it_1->x;
        float y = it_1->y;
        float z = it_1->z;

        it_1->x = z;
        it_1->y = y;
        it_1->z = x;

        ++it_1;
    }

    //利用PointCloud2Vector2d 函数将空间变换后的点云数据转换为vector_vec2d格式
    pcl::on_nurbs::NurbsDataCurve2d data;
    PointCloud2Vector2d(cloud, data.interior);

    //利用FittingCurve2dSDM类下的initNurbsCurve2D函数构造初始曲线用于后续曲线拟合
    unsigned order(3);            //确定曲线阶数
    unsigned n_control_points(30);//确定曲线控制点个数
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dSDM::initNurbsCurve2D(order,
        data.interior, n_control_points); //初始化曲线

    //---------初始化曲线拟合对象-----------------
    pcl::on_nurbs::FittingCurve2dPDM::Parameter curve_params;
    curve_params.smoothness = 0.000001;
    curve_params.rScale = 1;

     pcl::on_nurbs::FittingCurve2dPDM fit(&data, curve);
     fit.assemble(curve_params);
     fit.solve();
     VisualizeCurveNURBS(fit.m_nurbs, 1.0, 0.0, 0.0, true);
     
  
    
    //------------可视化曲线拟合的结果-------------------------
    viewer.setSize(800, 600);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");//腿部切割点云
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

 
}

void get_BirthDistance(char dir[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    
    double distance = 0;  
    for (int nIndex = 1; nIndex < cloud->points.size()-1; nIndex++)
    {
        int indexZ = nIndex-1;
          distance += sqrt(abs(cloud->points[nIndex].x - cloud->points[indexZ].x) * abs(cloud->points[nIndex].x - cloud->points[indexZ].x) + abs(cloud->points[nIndex].y - cloud->points[indexZ].y) * abs(cloud->points[nIndex].y - cloud->points[indexZ].y));    
            indexZ++;
       
    }
    cout << "胸围值为（m）：" << distance << endl;
}

void equalToX(char dir[], char dir1[], float middleX)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_X(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
    int indexX = 0;
    cloud_X->width = cloud->width;
    cloud_X->height = 1;
    cloud_X->points.resize(cloud_X->width * cloud_X->height);
    for (int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    {
        if ((std::abs(cloud->points[nIndex].x - middleX) <= 0.0004) == 1)
        {
            cloud_X->points[indexX].x = cloud->points[nIndex].x;
            cloud_X->points[indexX].y = cloud->points[nIndex].y;
            cloud_X->points[indexX].z = cloud->points[nIndex].z;
            indexX++;
        }
    }
    cloud_X->width = indexX;
    cloud_X->height = 1;
    cloud_X->points.resize(cloud_X->width * cloud_X->height);
    pcl::io::savePCDFileASCII(dir1, *cloud_X);

}

