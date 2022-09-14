// Copyright 2018 David V. Lu!!
#ifndef SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
#define SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <people_msgs/People.h>
#include <boost/thread.hpp>
#include <list>

namespace social_navigation_layers
{
class SocialLayer : public costmap_2d::Layer
{
public:
  SocialLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized()
  {
    return false;
  }

protected:
  void peopleCallback(const people_msgs::People& people);
  ros::Subscriber people_sub_;
  people_msgs::People people_list_;
  std::list<people_msgs::Person> transformed_people_;
  ros::Duration people_keep_time_;
  boost::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};
}  // namespace social_navigation_layers

#endif  // SOCIAL_NAVIGATION_LAYERS_SOCIAL_LAYER_H
