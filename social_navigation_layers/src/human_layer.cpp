//
//      Human Layer Costmap Plugin Class
//       by Travis Riggs
//
//  This class implements a costmap 2D plugin that represents humans. It
//  is primarily intended as a layer for the global costmap.
//
#include <cmath>
#include <social_navigation_layers/human_layer.h>


namespace social_navigation_layers
{

// Initializes parameters for the Human Layer
//
// The costmap calls this method when it loads this class as a plugin.
void HumanLayer::onInitialize()
{
  SocialLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<HumanLayerConfig>(nh);
  f_ = boost::bind(&HumanLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

// Calculates the obstacle cost of a point in 2D space
//
// This cost function is circular. From the center, to the lethal radius
// the cost is "lethal." From there to the inscribed radius, the cost is
// the "inscribed" cost. Beyond the inscribed radius, the cost gradually
// decays exponentially.
//
// Args:
//  (x, y): Coordinates of cost to be calculated
//  (x0, y0): Coordinates of center of obstacle
//  variance: Exponential variance for costs oustide of inscribed radius
//            Increasing variance causes a slower decay of cost and a
//            wider cirle of cost around a person.
//  lethal_radius: Radius of lethal cost region (meters)
//  inscribed_radius: Radius of inscribed cost region (meters)
//
// Returns:
//  costmap cost, ranging from 0 to 254
double HumanLayer::calculate_cost(double x, double y, double x0, double y0, double variance,
                                  double lethal_radius, double inscribed_radius)
{
  double cost;
  const double dx = x - x0;
  const double dy = y - y0;
  const double radius = std::sqrt(dx * dx + dy * dy);
  if (radius < lethal_radius)
    cost = LETHAL_COST;
  else if (radius < inscribed_radius)
    cost = INSCRIBED_COST;
  else {
    cost = INSCRIBED_COST * std::exp(-(radius - inscribed_radius) / variance);
    if (cost > INSCRIBED_COST)
      cost = INSCRIBED_COST;
  }

  return cost;
}

// Calculates the maximum radius of costs for a human
//
// The cost function is exponential, which means it technically extends
// to inifinite, but practically, any changes lower than the cutoff are
// simply ignored. Calculate that maximum radius of pixels that need to
// change surrounding an obstacle for a given cutoff and variance.
//
// Args:
//  inscribed_radius: Radius of inscribed cost region (meters)
//  cutoff: Cost threshold below which changes are not made to costmap
//  variance: Exponential variance of cost function
//
// Returns:
//  maximum radius of changes to costmap surrounding a human in meters
double HumanLayer::get_radius(double inscribed_radius, double cutoff, double variance)
{
  return inscribed_radius - variance * std::log(cutoff / INSCRIBED_COST);
}

// Calculates the min/max bounding box of costmap changes for all people
//
// This is called by the costmap via the updateBounds method in the
// parent SocialLayer class. Instead of updating all of the cells for
// the entire costmap, only update a region of interest. This iterates
// through all of the people detected and finds the minimum bounding box
// to contain all of the changes for this update cycle.
//
// Output:
//  (min_x, min_y, max_x, max_y) - Pointers to bounding box of changes
void HumanLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  const double point = get_radius(inscribed_radius_, cutoff_, variance_);
  std::list<people_msgs::Person>::iterator p_it;
  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    const people_msgs::Person& person = *p_it;

    *min_x = std::min(*min_x, person.position.x - point);
    *min_y = std::min(*min_y, person.position.y - point);
    *max_x = std::max(*max_x, person.position.x + point);
    *max_y = std::max(*max_y, person.position.y + point);
  }
}

// Updates the costmap cells within the bounded window
//
// This method is called by the costmap on every update cycle. It
// calculates the cost value for each costmap cell surrounding each
// human reported and applies them to the costmap.
//
// Inputs:
//  master_grid - Reference to 2D costmap
//  (min_i, min_j, max_i, max_j) - Bounding box of changes to costmap
void HumanLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (people_list_.people.size() == 0)
    return;

  if (!enabled_)
    return;

  std::list<people_msgs::Person>::iterator p_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  const double res = costmap->getResolution();
  const double base = get_radius(inscribed_radius_, cutoff_, variance_);
  // Width of square bounding box for a person in units of costmap cells
  const unsigned int width = std::max(1, static_cast<int>((2.0 * base) / res));

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
  {
    const people_msgs::Person& person = *p_it;

    // Global frame coordinates of person (meters)
    const double cx = person.position.x;
    const double cy = person.position.y;

    // Offset coordinates of person (meters). For a typical map, this is
    // the lower-left corner of the bounding box for the costmap cells
    // we are going to modify.
    const double ox = cx - base;
    const double oy = cy - base;

    // Convert offset coords to discretized coordinates in units of
    // costmap cells
    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    // Calculate size of bounding box in units of costmap cells
    int start_x = 0, start_y = 0, end_x = width, end_y = width;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + width > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    // Shift bounding box start by half of a costmap cell to represent
    // the center of the cell, instead of the lower-left corner, to
    // calculate the cost. (bx, by) are in units of meters.
    const double bx = ox + res / 2;
    const double by = oy + res / 2;
    // Calculate the cost of each cell in the bounding box around person
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          // Do not change costmap if there is no information
          // previously. Most commonly applies past the edge of the map.
          continue;

        double x = bx + i * res;
        double y = by + j * res;
        double a = calculate_cost(x, y, cx, cy, variance_, lethal_radius_, inscribed_radius_);

        if (a < cutoff_)
          // Don't bother updating the costmap for costs that are small
          continue;

        unsigned char cvalue = static_cast<unsigned char>(a);
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

// Dynamically reconfigures the parameters of the human layer
void HumanLayer::configure(HumanLayerConfig& config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  enabled_ = config.enable;
  cutoff_ = config.cutoff;
  variance_ = config.variance;
  lethal_radius_ = config.lethal_radius;
  inscribed_radius_ = config.inscribed_radius;
}

};  // namespace social_navigation_layers

PLUGINLIB_EXPORT_CLASS(social_navigation_layers::HumanLayer, costmap_2d::Layer)
