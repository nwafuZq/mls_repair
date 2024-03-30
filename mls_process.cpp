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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

pcl::visualization::PCLVisualizer* p=new pcl::visualization::PCLVisualizer();
pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");

int vp_1, vp_2;

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

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation()
    {
     
        nr_dimensions_ = 4;
    }
    
    virtual void copyToFloatArray(const PointNormalT& p, float* out) const
    {
      
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


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
   
}


void loadData(int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD> >& models)
{
    std::string extension(".pcd");
  
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string(argv[i]);
      ）
        if (fname.size() <= extension.size())
            continue;
        std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);
      
        if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile(argv[i], *m.cloud);
           
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
            models.push_back(m);
        }
    }
}


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
        viewer->setBackgroundColor(1, 1, 1, v1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 100, 100, 100);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color,"befor filter", v1);
  

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor(0, 0, 0, v2);
        viewer->addText("Radius: 0.1", 20, 20, "v2 text", v2);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "after filter", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor filter");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after filter");
      

        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }
    return 0;
}


void RANSACc(char dir[], char ran_dir[])
{
    int o = 0;
    if(o==1)
    {  pcl::PCDReader reader;
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read(dir, *cloud);

 
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);

    float angle = 5;
    float epsangle = pcl::deg2rad(angle);
    Eigen::Vector3f axis(0.0, 0.0, 1.0);


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
     
        std::cout << "PointCloud After RANSAC: " << cloud_f->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "Standard_" << dir << ".pcd";
        pcl::io::savePCDFileASCII(ran_dir, *cloud_f);

    }
    }
    else {

       
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud) == -1)
            {
                PCL_ERROR("点云读取失败 \n");
               
            }

           pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
            ransac.setDistanceThreshold(0.03);
        

            Eigen::VectorXf coeff;
            ransac.getModelCoefficients(coeff);  

            cout << "平面模型系数coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << endl;
  
            extract.filter(*final);
           
            pcl::io::savePCDFileASCII(ran_dir, *final);
  

            viewer->createViewPort(0, 0, 0.5, 1, v1);
            viewer->createViewPort(0.5, 0, 1, 1, v2);
            viewer->setBackgroundColor(0, 0, 0, v1);
            viewer->setBackgroundColor(0, 0, 0, v2);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(sac_plane, 0, 0, 255);

            viewer->addPointCloud(cloud, color, "cloud", v1);
      
            pcl::ModelCoefficients plane;
            plane.values.push_back(coeff[0]);
            plane.values.push_back(coeff[1]);
            plane.values.push_back(coeff[2]);
            plane.values.push_back(coeff[3]);

            viewer->addPlane(plane, "plane",v2);
        
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(10000));
            }
            pcl::io::savePCDFileASCII(ran_dir, *final);


    }
}


int EuclideanClusterr(char dir[],char dir1[],int flag)
{
    if(flag==0)
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile(dir, *cloud);

        std::cerr << "PointCloud before Euclidean clustering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05); 
       
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
    
        pcl::io::loadPCDFile(dir, *cloud);


        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
       
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
        
            if (j == 0) {
                pcl::io::savePCDFileASCII(dir1, *cloud_cluster);
            }
            j++;
        }
        return j;
    }
}

void VG_filtere(char dir[], char vg_dir[],float leafx,float leafy,float leafz)
{
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr VG_cloud_filtered_blob(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr VG_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read(dir, *cloud_blob);
    std::cerr << "PointCloud before Voxel Grid Removal filtering: " << cloud_blob->width * cloud_blob->height
        << " data points (" << pcl::getFieldsList(*cloud_blob) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> VG_sor;
    VG_sor.setInputCloud(cloud_blob);
    VG_sor.setLeafSize(leafx,leafy,leafz);
    VG_sor.filter(*VG_cloud_filtered_blob);
    std::cerr << "PointCloud after filtering: " << VG_cloud_filtered_blob->width * VG_cloud_filtered_blob->height
        << " data points (" << pcl::getFieldsList(*VG_cloud_filtered_blob) << ")." << std::endl;

    pcl::fromPCLPointCloud2(*VG_cloud_filtered_blob, *VG_cloud_filtered);
    pcl::io::savePCDFileASCII(vg_dir, *VG_cloud_filtered);
}


void PCD_GP3D(char dir[], char dir1[]) {

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>)

    time_t begin, end;

    n.setSearchMethod(tree);
    n.setKSearch(20);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    gp3.setMaximumAngle(5 * M_PI / 3); 

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);

    begin = clock();
    gp3.reconstruct(triangles);
    end = clock();
    double Times = double(end - begin) / CLOCKS_PER_SEC;


    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
            viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

   
}


void PCD_cube() {
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    if (pcl::io::loadPCDFile("Upmls.pcd", *cloud) == -1) {
            PCL_ERROR("Could not read pcd file!\n");
           
        }
    

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);



    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals


    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::MarchingCubes<pcl::PointNormal>* mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();

  if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
  else
  {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }

    mc->setIsoLevel(0.0f);
    mc->setGridResolution(100, 100, 100);
    mc->setPercentageExtendGrid(0.0f);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
   
}


void calaPointCloudCoincide(PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_target, float para1, float para2, float& coincide)

{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_target);


    core.determineReciprocalCorrespondences(*cor, para1);   

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

    double num = 0;
    for (size_t i = 0; i < cor->size(); i++)
    {
 
        overlapA.points[i].x = cloud_src->points[cor->at(i).index_query].x;
        overlapA.points[i].y = cloud_src->points[cor->at(i).index_query].y;
        overlapA.points[i].z = cloud_src->points[cor->at(i).index_query].z;


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


}


void mls_donwPcd(char dir[],char dir1[])
{
      
      
        mls.process(mls_points);
        cout << "下采样后点云的个数：" << mls_points.size() << endl;


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
      
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
}

void mls_upPcd(char dir[],char dir1[],int flag)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);
   
        if (flag == 1) {
            up.setSearchMethod(tree);

            up.process(*cloud_up);
            pcl::io::savePCDFileASCII(dir1, *cloud_up);

            cout << "上采样后点云的个数：" << cloud_up->points.size() << endl;
        
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

            pcl::io::savePCDFileASCII(dir1, *cloud_up);

            cout << "上采样后点云的个数：" << cloud_up->points.size() << endl;
          
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


void avg_mls(char dir[],char dir1[]) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(dir, *cloud);

    std::cout << "原始点云个数：" << cloud->points.size() << endl;

    pcl::UniformSampling<pcl::PointXYZ> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.005f);
    US.filter(*cloud_filtered);
    cout << "均匀采样之后点云的个数：" << cloud_filtered->points.size() << endl;

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


    pcl::on_nurbs::FittingSurface::Parameter params;

 
    printf("  surface fitting ...\n");

    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);



    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);
    std::cout << "Before refine" << endl;
    viewer.spinOnce(3000);

    for (unsigned i = 0; i < refinement; i++)
    {

        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        std::cout << "refine: " << i << endl;
    }

  
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce(3000);
        std::cout << "iterations: " << i << endl;
    }

 

    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;



    printf("  curve fitting ...\n");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);


    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); 
    curve_fit.fitting(curve_params);
    visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);


    printf("  triangulate trimmed surface ...\n");
    viewer.removePolygonMesh(mesh_id);

    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
        mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);


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
        cavehull.reconstruct(*surface_hull, polygons);

        pcl::PolygonMesh mesh;
        begin = clock();
        cavehull.reconstruct(mesh);
        end = clock();
        pcl::io::savePLYFile(dir1, mesh);
        
        cerr << "Concave hull has: " << surface_hull->points.size()
            << " data points." << endl;
        double Times = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "time:" << Times << "s" << endl;
        pcl::PCDWriter writer;
    
        viewer->addPolygonMesh<pcl::PointNormal>(surface_hull, polygons, "polyline");
        viewer->spin();

    


}

void readply(char dir[],char dir1[])
{

        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(dir, mesh);
        pcl::PolygonMesh mesh1;
        pcl::io::loadPLYFile(dir1, mesh1);

    
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

    

}

void div_pcd(char dir[],char dir1[],char dir2[])
{
      
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
   
   
        pcl::io::loadPCDFile(dir, *source_cloud);
 
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

      
        pcl::io::loadPCDFile(dir, *source_cloud1);
     
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud1(new pcl::ConditionAnd<pcl::PointXYZ>());
    
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

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setRadiusSearch(30);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.compute(*normals);
    
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
        boundEst.setInputCloud(cloud);
        boundEst.setInputNormals(normals);
        boundEst.setRadiusSearch(50);
        boundEst.setAngleThreshold(M_PI/4 );

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


    pcl::on_nurbs::NurbsDataCurve2d data;
    PointCloud2Vector2d(cloud, data.interior);

  
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dSDM::initNurbsCurve2D(order,
        data.interior, n_control_points); 

 
    pcl::on_nurbs::FittingCurve2dPDM::Parameter curve_params;
    curve_params.smoothness = 0.000001;
    curve_params.rScale = 1;

     pcl::on_nurbs::FittingCurve2dPDM fit(&data, curve);
     fit.assemble(curve_params);
     fit.solve();
     VisualizeCurveNURBS(fit.m_nurbs, 1.0, 0.0, 0.0, true);
     

    viewer.setSize(800, 600);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
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

