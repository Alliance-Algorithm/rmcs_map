# RMCS Map

### Interface Table
#### Publish

| Detail | Topic Name | Type |
| :--- | :---- | :---- |
| `grid map` | /rmcs_map/map/grid | nav_msgs::msg::OccupancyGrid |
| `cost map` | /rmcs_map/map/cost | nav_msgs::msg::OccupancyGrid |
| `status` | /rmcs_map/status | rmcs_map::msg::GameStatus |

#### Subscribe

| Detail | Topic Name | Type |
| :--- | :---- | :---- |
| `lidar` | /livox/lidar | livox_ros_driver2::msg::CustomMsg |