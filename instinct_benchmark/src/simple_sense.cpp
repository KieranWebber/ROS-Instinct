#include "../include/instinct_benchmark/simple_sense.h"
int SimpleSense::readSense(int senseId)
{
    // Simple random + modulus simulated sense operation
    return rand() % 100;
}
SimpleSense::SimpleSense(const ros::NodeHandle& handle) : SenseService(handle)
{
}
