#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_to_mesh/pcl_to_mesh.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_to_mesh_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("pcl_to_mesh",
                                                     [](pcl_to_mesh::PclToMesh::Request &req, pcl_to_mesh::PclToMesh::Response &res)
                                                     {
                                                         res.mesh = pcl_to_mesh::PclToMesh<pcl::PointXYZRGB>::pcl_to_mesh(req.pcl);
                                                         return true;
                                                     });

    ros::spin();
}