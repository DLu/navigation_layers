#include <sonar_layer/sonar_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(sonar_layer::SonarLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace sonar_layer
{

SonarLayer::SonarLayer() {}

void SonarLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = to_cost(0.5);
  phi_v_ = 1.2;
  max_angle_ = 12.5*M_PI/180;

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();

  range_sub_ = nh.subscribe("/sonar", 10, &SonarLayer::incomingRange, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SonarLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


double SonarLayer::gamma(double theta)
{
    if(fabs(theta)>max_angle_)
        return 0.0;
    else
        return 1 - pow(theta/max_angle_, 2);
}

double SonarLayer::delta(double phi)
{
    return 1 - (1+tanh(2*(phi-phi_v_)))/2;
}

void SonarLayer::get_deltas(double angle, double *dx, double *dy)
{
    double ta = tan(angle);
    if(ta==0)
        *dx = 0;
    else
        *dx = resolution_ / ta;
    
    *dx = copysign(*dx, cos(angle));
    *dy = copysign(resolution_, sin(angle));
}

double SonarLayer::sensor_model(double r, double phi, double theta)
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


void SonarLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void SonarLayer::incomingRange(const sensor_msgs::RangeConstPtr& range)
{
  double r = range->range;
  if(r<range->min_range || r>range->max_range)
    return;
  max_angle_ = range->field_of_view/2;
  
  geometry_msgs::PointStamped in, out;
  in.header.stamp = range->header.stamp;
  in.header.frame_id = range->header.frame_id;  

  // TransformListener *tf_;
  if( tf_->waitForTransform(global_frame_, in.header.frame_id,
        in.header.stamp, ros::Duration(0.1)) ) {
    tf_->transformPoint (global_frame_, in, out);
    
    double ox = out.point.x, oy = out.point.y;
    
    in.point.x = r;
  
    tf_->transformPoint(global_frame_, in, out);
    
    double tx = out.point.x, ty = out.point.y;
    ROS_INFO("%s", range->header.frame_id.c_str());
    ROS_INFO("%f %f --> %f %f", ox, oy, tx, ty);
    
    // calculate target props
    double dx = tx-ox, dy = ty-oy,
          theta = atan2(dy,dx), d = sqrt(dx*dx+dy*dy);
  
    /*
    double dx1, dx2, dy1, dy2;
    get_deltas(theta+max_angle_, &dx1, &dy1);
    get_deltas(theta-max_angle_, &dx2, &dy2);
  
    if(dx1 > dx2){
      double a = dx1, b = dy1;
      dx1 = dx2;
      dy1 = dy2;
      dx2 = a;
      dy2 = b;
    }
  
    ROS_INFO("%f %f | %f %f", dx1, dy1, dx2, dy2);
  
    double x1 = ox, y1 = oy, x2 = ox, y2 = oy;
    for(int i=0;i<int(d/dx1);i++){
      double x = x1, y = y1;
      ROS_INFO("==%f==", y);
      while(x <= x2){
        update_cell(ox, oy, r, x, y);
        ROS_INFO("   %f", x);
        x += resolution_;
      }
     
      x1 += dx1;
      x2 += dx2;
      y1 += dy1;
      y2 += dy2;
    }
    */
    
    double bx0, by0, bx1, by1;
    bx0 = bx1 = ox;
    by0 = by1 = oy;
    
    double mx, my;
    mx = ox + cos(theta-max_angle_) * d * 1.2;
    my = oy + sin(theta-max_angle_) * d * 1.2;  
    touch(mx, my, &bx0, &by0, &bx1, &by1);  
    mx = ox + cos(theta+max_angle_) * d * 1.2;
    my = oy + sin(theta+max_angle_) * d * 1.2;
    touch(mx, my, &bx0, &by0, &bx1, &by1);
    
    int sx, sy, ex, ey;
    worldToMapNoBounds(bx0, by0, sx, sy);
    sx = std::max(0, sx);
    sy = std::max(0, sy);
    worldToMapNoBounds(bx1, by1, ex, ey);
    ex = std::min((int)size_x_, ex);
    ey = std::min((int)size_y_, ey);
    
    ROS_INFO("\t%f %f %f %f", bx0, by0, bx1, by1);
    ROS_INFO("\t%d %d %d %d", sx, sy, ex, ey);

    
    for(unsigned int x=sx; x<ex; x++){
      for(unsigned int y=sy; y<ey; y++){
        double wx, wy;
        mapToWorld(x,y,wx,wy);
        //ROS_INFO("\t\t%f %f", wx, wy);
        update_cell(ox, oy, r, wx, wy);
      }
    } 
    
  //  update_cell(ox, oy, r, tx, ty);
    
    touch(bx0, by0, &min_x_, &min_y_, &max_x_, &max_y_);
    touch(bx1, by1, &min_x_, &min_y_, &max_x_, &max_y_);
  
    current_ = false;
  } else {
    // can't transform
    ROS_ERROR("Sonar layer can't transform from %s to %s at %f",
        global_frame_.c_str(), in.header.frame_id.c_str(),
        in.header.stamp.toSec());
  }
}

void SonarLayer::update_cell(double ox, double oy, double r, double nx, double ny)
{
  unsigned int x, y;
  if(worldToMap(nx, ny, x, y)){
    double dx = nx-ox, dy = ny-oy;
    double theta = atan2(dy, dx), phi = sqrt(dx*dx+dy*dy);
    double sensor = sensor_model(r,phi,theta);
    double prior = to_prob(getCost(x,y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ/(prob_occ+prob_not);
    //new_prob = phi>r?1.0:0.0;
    
    //ROS_INFO("%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    //ROS_INFO("%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
    unsigned char c = to_cost(new_prob);
    setCost(x,y,c);
  }
}

void SonarLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  if (current_)
    return;

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
  
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();
  current_ = true;
}

void SonarLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

} // end namespace
