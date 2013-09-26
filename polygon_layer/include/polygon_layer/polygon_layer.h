#ifndef POLYGON_LAYER_H_
#define POLYGON_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <polygon_layer/PolygonList.h>
#include <polygon_layer/PolygonLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace polygon_layer
{

class PolygonLayer : public costmap_2d::CostmapLayer
{
public:
  PolygonLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(PolygonLayerConfig &config, uint32_t level);
  void incomingPolygons(const polygon_layer::PolygonListConstPtr& polygons);

  unsigned char cost_;
  ros::Subscriber poly_sub_;
  double min_x_, min_y_, max_x_, max_y_;
  
  dynamic_reconfigure::Server<PolygonLayerConfig> *dsrv_;
};
}
#endif
