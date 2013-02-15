#include "OccupancyGridBuilder.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_builder");
    ros::NodeHandle n; //Listener
  
    OccupancyGridBuilder occMapBuilder(n);
    occMapBuilder.initNode();
  
    ros::Rate r(0.5);
  
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
    
}
