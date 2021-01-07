// Copyright 2018 David V. Lu!!
#ifndef RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
#define RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/Range.h>
#include <range_sensor_layer/RangeSensorLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace range_sensor_layer
{

class RangeSensorLayer : public costmap_2d::CostmapLayer
{
public:
  enum InputSensorType
  {
    VARIABLE,
    FIXED,
    ALL
  };

  RangeSensorLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual void deactivate();
  virtual void activate();

private:
  void reconfigureCB(range_sensor_layer::RangeSensorLayerConfig &config, uint32_t level);

  void bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message);
  void processRangeMsg(sensor_msgs::Range& range_message);
  void processFixedRangeMsg(sensor_msgs::Range& range_message);
  void processVariableRangeMsg(sensor_msgs::Range& range_message);

  void resetRange();
  void updateCostmap();
  void updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone);
  void removeOutdatedReadings();

  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);

  void get_deltas(double angle, double *dx, double *dy);
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear);

  double to_prob(unsigned char c)
  {
    return static_cast<double>(c) / costmap_2d::LETHAL_OBSTACLE;
  }
  unsigned char to_cost(double p)
  {
    return static_cast<unsigned char>(p * costmap_2d::LETHAL_OBSTACLE);
  }

  boost::function<void(sensor_msgs::Range& range_message)> processRangeMessageFunc_;
  boost::mutex range_message_mutex_;
  std::list<sensor_msgs::Range> range_msgs_buffer_;
  std::map<std::pair<unsigned int, unsigned int>, double> marked_point_history_;

  double max_angle_, phi_v_;
  double inflate_cone_;
  std::string global_frame_;

  double clear_threshold_, mark_threshold_;
  bool clear_on_max_reading_;

  double no_readings_timeout_;
  ros::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<ros::Subscriber> range_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  bool use_decay_;
  double pixel_decay_;
  double transform_tolerance_;

  dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig> *dsrv_;


  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  };

  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  };
};
}  // namespace range_sensor_layer
#endif  // RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H
