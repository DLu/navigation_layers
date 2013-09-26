#include<ros/ros.h>
#include<polygon_layer/PolygonList.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "examplepolygon");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<polygon_layer::PolygonList>("polygons", 1000);

  ros::Rate loop_rate(.1);

  polygon_layer::PolygonList pl;
  pl.header.frame_id = "/map";
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 pt;
  polygon.points.push_back(pt);
  polygon.points.push_back(pt);
  polygon.points.push_back(pt);
  pl.polygons.push_back(polygon);

  int count = 0;
  while (ros::ok())
  {
pl.polygons.clear();
    polygon.points[1].x = count + 1;
    polygon.points[2].y = count + 1;
  pl.polygons.push_back(polygon);
    pub.publish(pl);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    if(count>5){
        count = 0;
    }
  }


  return 0;
    

}
