#include <ros/init.h>
#include <ros/node_handle.h>
#include <instinct_store/instinct_store_service.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "instinct_store");
    ros::NodeHandle n;
    ros::NodeHandle privateN("~");
    int defaultStoreValue;
    privateN.param<int>("store_default", defaultStoreValue, 0);
    InstinctStoreService storeService(n, defaultStoreValue);
    ros::spin();
}