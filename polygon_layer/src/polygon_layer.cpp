#include<polygon_layer/polygon_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(polygon_layer::PolygonLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace polygon_layer
{

PolygonLayer::PolygonLayer() {}

void PolygonLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  poly_sub_ = nh.subscribe("polygons", 1, &PolygonLayer::incomingPolygons, this);

  dsrv_ = new dynamic_reconfigure::Server<PolygonLayerConfig>(nh);
  dynamic_reconfigure::Server<PolygonLayerConfig>::CallbackType cb = boost::bind(
      &PolygonLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void PolygonLayer::reconfigureCB(PolygonLayerConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled || cost_ != config.polygon_cost)
  {
    enabled_ = config.enabled;
    cost_ = config.polygon_cost;
    current_ = false;
  }
}

void PolygonLayer::incomingPolygons(const polygon_layer::PolygonListConstPtr& polygons)
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();

  geometry_msgs::PointStamped p, tp;
  p.header = polygons->header;

  for(unsigned int i=0;i<polygons->polygons.size();i++)
  {
    std::vector<geometry_msgs::Point> polygon;

    for(unsigned int j=0;j<polygons->polygons[i].points.size(); j++)
    {
        p.point.x = polygons->polygons[i].points[j].x;
        p.point.y = polygons->polygons[i].points[j].y;
        p.point.z = polygons->polygons[i].points[j].z;
        tf_->transformPoint(layered_costmap_->getGlobalFrameID(), p, tp);
        polygon.push_back(tp.point);
        touch(tp.point.x, tp.point.y, &min_x_, &min_y_, &max_x_, &max_y_);
    }

    setConvexPolygonCost(polygon, cost_);
  }

  current_ = false;
}

void PolygonLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (current_)
    return;

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
}

void PolygonLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

} // end namespace
