# Decision Interface

### Interface Table
#### Publish

| Detail | Topic Name | Type |
| :--- | :---- | :---- |
| `grid map` | /decision_interface/map/grid | nav_msgs::msg::OccupancyGrid |
| `cost map` | /decision_interface/map/cost | nav_msgs::msg::OccupancyGrid |
| `status` | /decision_interface/status | decision_interface::msg::GameStatus |

#### Subscribe

| Detail | Topic Name | Type |
| :--- | :---- | :---- |
| `velocity` | /decision_interface/control/velocity | geometry_msgs::msg::Pose2D |
| `rotation` | /decision_interface/control/rotation | std_msgs::msg::Int32 |
| `gimbal` | /decision_interface/control/gimbal | geometry_msgs::msg::Vector3 |