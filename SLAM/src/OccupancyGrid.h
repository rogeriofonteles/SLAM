#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_
#define OCCUPANCY_GRID_OCCUPANCY_GRID_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
 
class OccupancyGrid
{
    public:
         OccupancyGrid(float center_x, float center_y, float center_z,
                      float size_x, float size_y, float size_z,
                       float res_x, float res_y, float res_z);

         ~OccupancyGrid();

         void fillOccupancyGrid(const sensor_msgs::PointCloud cloud);

         sensor_msgs::PointCloud gridToPoints();

         unsigned int nX();
         unsigned int nY();
         unsigned int nZ();

         int8_t* getData();
         int occupied;

     private:
         unsigned int nx_, ny_, nz_;
         float size_x_, size_y_, size_z_;
         float center_x_, center_y_, center_z_;
         float res_x_, res_y_, res_z_;
         int8_t *data_;
         
};
 
 
 #endif
