# Route Planner

Route planning module in CMPF finds the high-level route (the global path) from a starting point to a goal in the given map. Specifically, it finds the optimal path in structured environments (i.e., roads, intersections, traffic signs, etc. are clearly defined) by following the traffic rules such as driving in the center of the lane or changing lanes only when it is allowed. In these scenarios high-definition maps (HD maps) are widely used for self-driving vehicle navigation since these maps can provide geometric, routing and semanctic information of the road networks, traffic regulations, sidewalks and parking spaces. In CMPF, we rely on [Lanelet2 HD map framework](https://ieeexplore.ieee.org/document/8569929) and use the [Lanelet2 C++ library](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) for handling the HD map data. 

## Action API

The route planning module in CMPF provides the implementation of the SimpleActionServer (see [actionlib documentation](http://wiki.ros.org/actionlib)) as `cmpf_route_planner/RoutePlannerNodelet` ROS Nodelet that takes in the HD map information and generate the optimal route in the road network by using the [routing module from lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_routing). It uses the `cmpf_msgs/ComputeRouteToPose` custom action message type (See [cmpf_msgs](https://github.com/mlsdpk/cmpf/tree/master/cmpf_msgs)).

Action Subscribed Topics

- ~\<name>\/plan_route/goal (geometry_msgs/PoseStamped)  
    A goal for finding the optimal route.

Action Published Topics

- ~\<name>\/plan_route/result (cmpf_msgs/Route)  
    The route from a starting position to a goal.

## Service Requests

- /lanelet2_map_server/get_lanelet2_map (cmpf_msgs::Lanelet2Map)  
    Get HD map via service call.

## Published Topics

- ~\<name>\/route_marker (visualization_msgs/Marker)  
    The global route that the ego vehicle is currently attempting to follow. Used primarily for visualization purposes.

## Parameters

- ~\<name>\/route_resolution (double, default: 2.0)  
    The resolution of the generated route (i.e., the distance between route waypoints) in meters.
%
- ~\<name>\/publish_route_markers (bool, default: false)  
    Whether or not publish the route markers.