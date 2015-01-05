#ifndef RANGE_SENSOR_LAYER_H_
#define RANGE_SENSOR_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <sensor_msgs/Range.h>
//#include <range_sensor_layer/RangeSensorLayerConfig.h>
#include <dynamic_reconfigure/server.h>

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
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  void bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message);
  void processRangeMsg(sensor_msgs::Range& range_message);
  void processFixedRangeMsg(sensor_msgs::Range& range_message);
  void processVariableRangeMsg(sensor_msgs::Range& range_message);

  void updateCostmap();
  void updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone);

  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);

  void get_deltas(double angle, double *dx, double *dy);
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear);

  double to_prob(unsigned char c){ return double(c)/costmap_2d::LETHAL_OBSTACLE; }
  unsigned char to_cost(double p){ return (unsigned char)(p*costmap_2d::LETHAL_OBSTACLE); }

  boost::function<void (sensor_msgs::Range& range_message)> processRangeMessageFunc_;
  boost::mutex range_message_mutex_;
  std::list<sensor_msgs::Range> range_msgs_buffer_;

  double max_angle_, phi_v_;
  std::string global_frame_;

  double clear_threshold_, mark_threshold_;
  bool clear_on_max_reading_;

  double no_readings_timeout_;
  ros::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<ros::Subscriber> range_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
