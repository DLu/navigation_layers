#include<range_sensor_layer/range_sensor_layer.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::RangeSensorLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace range_sensor_layer
{

RangeSensorLayer::RangeSensorLayer() {}

void RangeSensorLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);
  phi_v_ = 1.2;
  max_angle_ = 12.5*M_PI/180;

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();

  // Default topic names list contains a single topic: /sonar
  // We use the XmlRpcValue constructor that takes a XML string and reading start offset
  const char* xml = "<value><array><data><value>/sonar</value></data></array></value>";
  int zero_offset = 0;
  std::string topics_ns;
  XmlRpc::XmlRpcValue topic_names(xml, &zero_offset);

  nh.param("ns", topics_ns, std::string());
  nh.param("topics", topic_names, topic_names);

  nh.param("no_readings_timeout", no_readings_timeout_, .0);

  nh.param("clear_threshold", clear_threshold_, .2);
  nh.param("mark_threshold", mark_threshold_, .8);

  nh.param("clear_on_max_reading", clear_on_max_reading_, false);

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if ((topic_names.valid() == false) || (topic_names.getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("Invalid topic names list: it must be a non-empty list of strings");
    return;
  }

  if (topic_names.size() < 1)
  {
    // This could be an error, but I keep it as it can be useful for debug
    ROS_WARN("Empty topic names list: range sensor layer will have no effect on costmap");
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  for (unsigned int i = 0; i < topic_names.size(); i++)
  {
    if (topic_names[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN("Invalid topic names list: element %d is not a string, so it will be ignored", i);
    }
    else
    {
      std::string topic_name(topics_ns);
      if ((topic_name.size() > 0) && (topic_name.at(topic_name.size() - 1) != '/'))
        topic_name += "/";
      topic_name += static_cast<std::string>(topic_names[i]);
      range_subs_.push_back(nh.subscribe(topic_name, 100, &RangeSensorLayer::incomingRange, this));
      ROS_INFO("RangeSensorLayer: subscribed to topic %s", range_subs_.back().getTopic().c_str());
    }
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &RangeSensorLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


double RangeSensorLayer::gamma(double theta)
{
    if(fabs(theta)>max_angle_)
        return 0.0;
    else
        return 1 - pow(theta/max_angle_, 2);
}

double RangeSensorLayer::delta(double phi)
{
    return 1 - (1+tanh(2*(phi-phi_v_)))/2;
}

void RangeSensorLayer::get_deltas(double angle, double *dx, double *dy)
{
    double ta = tan(angle);
    if(ta==0)
        *dx = 0;
    else
        *dx = resolution_ / ta;

    *dx = copysign(*dx, cos(angle));
    *dy = copysign(resolution_, sin(angle));
}

double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
    double lbda = delta(phi)*gamma(theta);

    double delta = resolution_;

    if(phi >= 0.0 and phi < r - 2 * delta * r)
        return (1- lbda) * (0.5);
    else if(phi < r - delta * r)
        return lbda* 0.5 * pow((phi - (r - 2*delta*r))/(delta*r), 2)+(1-lbda)*.5;
    else if(phi < r + delta * r){
        double J = (r-phi)/(delta*r);
        return lbda * ((1-(0.5)*pow(J,2)) -0.5) + 0.5;
    }
    else
        return 0.5;
}


void RangeSensorLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void RangeSensorLayer::incomingRange(const sensor_msgs::RangeConstPtr& range)
{
  double r = range->range;
  bool clear_sensor_cone = false;
  if (r < range->min_range)
    return;
  else if (r >= range->max_range && clear_on_max_reading_)
  {
    clear_sensor_cone = true;
    r = range->max_range;
  }

  max_angle_ = range->field_of_view/2;

  geometry_msgs::PointStamped in, out;
  in.header.stamp = range->header.stamp;
  in.header.frame_id = range->header.frame_id;

  if(!tf_->waitForTransform(global_frame_, in.header.frame_id,
        in.header.stamp, ros::Duration(0.1)) ) {
     ROS_ERROR("Range sensor layer can't transform from %s to %s at %f",
        global_frame_.c_str(), in.header.frame_id.c_str(),
        in.header.stamp.toSec());
     return;
  }

  tf_->transformPoint (global_frame_, in, out);

  double ox = out.point.x, oy = out.point.y;

  in.point.x = r;

  tf_->transformPoint(global_frame_, in, out);

  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  double dx = tx-ox, dy = ty-oy,
        theta = atan2(dy,dx), d = sqrt(dx*dx+dy*dy);

  // Integer Bounds of Update
  int bx0, by0, bx1, by1;

  // Bounds includes the origin
  worldToMapNoBounds(ox, oy, bx0, by0);
  bx1 = bx0;
  by1 = by0;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  unsigned int aa, ab;
  if(worldToMap(tx, ty, aa, ab)){
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;
  int a, b;

  // Update left side of sonar cone
  mx = ox + cos(theta-max_angle_) * d * 1.2;
  my = oy + sin(theta-max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update right side of sonar cone
  mx = ox + cos(theta+max_angle_) * d * 1.2;
  my = oy + sin(theta+max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, a, b);
  bx0 = std::min(bx0, a);
  bx1 = std::max(bx1, a);
  by0 = std::min(by0, b);
  by1 = std::max(by1, b);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min((int)size_x_, bx1);
  by1 = std::min((int)size_y_, by1);

  for(unsigned int x=bx0; x<=(unsigned int)bx1; x++){
    for(unsigned int y=by0; y<=(unsigned int)by1; y++){
      double wx, wy;
      mapToWorld(x,y,wx,wy);
      update_cell(ox, oy, theta, r, wx, wy, clear_sensor_cone);
    }
  }

  buffered_readings_++;
  last_reading_time_ = ros::Time::now();
}

void RangeSensorLayer::update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear)
{
  unsigned int x, y;
  if(worldToMap(nx, ny, x, y)){
    double dx = nx-ox, dy = ny-oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx*dx+dy*dy);
    double sensor = 0.0;
    if(!clear)
        sensor = sensor_model(r,phi,theta);
    double prior = to_prob(getCost(x,y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ/(prob_occ+prob_not);

    //ROS_INFO("%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    //ROS_INFO("%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
      unsigned char c = to_cost(new_prob);
      setCost(x,y,c);
  }
}

void RangeSensorLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();
}

void RangeSensorLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  if (buffered_readings_ == 0)
  {
    if (no_readings_timeout_ > 0.0 &&
        (ros::Time::now() - last_reading_time_).toSec() > no_readings_timeout_)
    {
      ROS_WARN_THROTTLE(2.0, "No range readings received for %.2f seconds, " \
                             "while expected at least every %.2f seconds.",
               (ros::Time::now() - last_reading_time_).toSec(), no_readings_timeout_);
      current_ = false;
    }

    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if(prob>mark)
        current = costmap_2d::LETHAL_OBSTACLE;
      else if(prob<clear)
        current = costmap_2d::FREE_SPACE;
      else{
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current)
        master_array[it] = current;
      it++;
    }
  }

  buffered_readings_ = 0;
  current_ = true;
}

} // end namespace
