//
//    Human Layer Costmap Plugin Class
//     by Travis Riggs
//
//  This class implements a costmap 2D plugin that represents humans. It
//  is primarily intended as a layer for the global costmap.
//
#ifndef SOCIAL_NAVIGATION_LAYERS_BADGER_HUMAN_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_BADGER_HUMAN_LAYER_H
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.h>
#include <social_navigation_layers/social_layer.h>
#include <social_navigation_layers/HumanLayerConfig.h>


namespace social_navigation_layers
{

class HumanLayer : public SocialLayer
{
public:
  HumanLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);

protected:
  static constexpr double LETHAL_COST = static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
  static constexpr double INSCRIBED_COST = static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  void configure(HumanLayerConfig& config, uint32_t level);
  double cutoff_, lethal_radius_, inscribed_radius_, variance_;
  double calculate_cost(double x, double y, double x0, double y0, double variance,
                        double lethal_radius, double inscribed_radius);
  double get_radius(double inscribed_radius, double cutoff, double variance);
  dynamic_reconfigure::Server<HumanLayerConfig>* server_;
  dynamic_reconfigure::Server<HumanLayerConfig>::CallbackType f_;
};

}  // namespace social_navigation_layers

#endif  // SOCAL_NAVIGATION_LAYERS_BADGER_HUMAN_LAYER_H
