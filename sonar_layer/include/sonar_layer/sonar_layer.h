#ifndef SONAR_LAYER_H_
#define SONAR_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <sensor_msgs/Range.h>
//#include <sonar_layer/SonarLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace sonar_layer
{

class SonarLayer : public costmap_2d::CostmapLayer
{
public:
  SonarLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void incomingRange(const sensor_msgs::RangeConstPtr& range);
  
  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);
  
  void get_deltas(double angle, double *dx, double *dy);
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny);
  
  double to_prob(unsigned char c){ return double(c)/costmap_2d::LETHAL_OBSTACLE; }
  unsigned char to_cost(double p){ return (unsigned char)(p*costmap_2d::LETHAL_OBSTACLE); }
    
  double max_angle_, phi_v_;
  std::string global_frame_;

  ros::Subscriber range_sub_;
  double min_x_, min_y_, max_x_, max_y_;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
