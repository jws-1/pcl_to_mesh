#include <pcl/point_types.h>
#include <shape_msgs/Mesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename PointT>
class PclToMesh
{
public:
    static shape_msgs::Mesh pcl_to_mesh(const pcl::PointCloud<PointT>::Ptr &pclROS)
    {

        // Convert to shape_msgs::Mesh
        shape_msgs::Mesh meshROS;

        // Convert to pcl::PointCloud
        pcl::PointCloud<PointT>::Ptr pcl(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(pclROS, *pcl);

        pcl::PointCloud<PointT>::Ptr voxelised(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxelGrid;
        voxelGrid.setInputCloud(pcl);
        voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelGrid.filter(*voxelised);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
        mls.setInputCloud(voxelised);
        mls.setSearchRadius(0.05);
        mls.setPolynomiaOrder(1);
        mls.setComputeNormals(true);
        mls.process(*cloudWithNormals);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormalsVoxelised(new pcl::PointCloud<pcl::PointNormal>);
        pcl::VoxelGrid<pcl::PointNormal> voxelGridNormals;
        voxelGridNormals.setInputCloud(cloudWithNormals);
        voxelGridNormals.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelGridNormals.filter(*cloudWithNormalsVoxelised);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(cloudWithNormalsVoxelised);

        pcl::PolygonMesh cloudToMesh;
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedy;
        greedy.setSearchRadius(0.1);
        greedy.setMu(2.5);
        greedy.setMaximumNearestNeighbors(100);
        greedy.setMinimumAngle(M_PI / 18);
        greedy.setMaximumAngle(2 * M_PI / 3);
        greedy.setNormalConsistency(true);
        greedy.setConsistentVertexOrdering(true);
        greedy.setSearchMethod(tree);
        greedy.setInputCloud(cloudWithNormalsVoxelised);
        greedy.reconstruct(cloudToMesh);

        pcl_msgs::PolygonMesh polygonMesh;
        pcl_conversions::fromPCL(mesh, polygonMesh);

        sensor_msgs::PointCloud2Modifier modifier(polygonMesh.cloud);

        sensor_msgs::PointCloud2ConstIterator<float> pointIterator(polygonMesh.cloud, "x");
        for (int i = 0; i < modifier->size(); ++i, ++pointIterator)
        {
            meshROS.vertices[i].x = pointIterator[0];
            meshROS.vertices[i].y = pointIterator[1];
            meshROS.vertices[i].z = pointIterator[2];
        }
        meshROS.triangles.resize(polygonMesh.polygons.size());

        for (int i = 0; i < polygonMesh.polygons.size(); ++i)
        {
            if (polygonMesh.polygons[i].vertices.size() != 3)
                continue;

            meshROS.triangles[i].vertex_indices[0] = polygonMesh.polygons[i].vertices[0];
            meshROS.triangles[i].vertex_indices[1] = polygonMesh.polygons[i].vertices[1];
            meshROS.triangles[i].vertex_indices[2] = polygonMesh.polygons[i].vertices[2];
        }

        return meshROS;
    }
};