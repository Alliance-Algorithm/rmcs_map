#include <rmcs_executor/component.hpp>
#include <rmcs_map/msg/game_status.hpp>

#include "ros2/node.hpp"

namespace rmcs_map {

class Component
    : public MapNode
    , public rmcs_executor::Component {
public:
    Component()
        : MapNode()
    {
    }

    void update() override
    {
        static auto update_count = 0;
        static rmcs_map::msg::GameStatus status;

        if (update_count % 100 == 0) {
            collect_referee_message(status);
            collect_auto_aim_message(status);
            publish_status(status);
        }

        update_count++;
    }

private:
    InputInterface<std::array<uint8_t, 7>> friends_hp_;
    InputInterface<std::array<uint8_t, 7>> enemies_hp_;

    InputInterface<std::array<Eigen::Vector2d, 7>> friends_pose_;
    InputInterface<std::array<Eigen::Vector2d, 7>> enemies_pose_;

    InputInterface<uint16_t> base_friend_hp_;
    InputInterface<uint16_t> outpost_friend_hp_;

    InputInterface<uint16_t> base_enemy_hp_;
    InputInterface<uint16_t> outpost_enemy_hp_;

    InputInterface<uint16_t> bullet_;

    void collect_referee_message(rmcs_map::msg::GameStatus& status)
    {
        // the hp of friends
        for (int index = 0; index < friends_hp_->size(); index++) {
            status.friends[index].hp = (*friends_hp_)[index];
        }
        // the hp of enemies
        for (int index = 0; index < enemies_hp_->size(); index++) {
            status.enemies[index].hp = (*enemies_hp_)[index];
        }

        // the hp of base and outpost
        status.base_friend_hp    = *base_friend_hp_;
        status.base_enemy_hp     = *base_enemy_hp_;
        status.outpost_friend_hp = *outpost_friend_hp_;
        status.outpost_enemy_hp  = *outpost_enemy_hp_;

        // the pose of friends
        for (int index = 0; index < friends_pose_->size(); index++) {
            status.friends[0].position.x = (*friends_pose_)[index].x();
            status.friends[0].position.y = (*friends_pose_)[index].y();
        }

        // the bullet of self
        status.bullet = *bullet_;
    }

    void collect_auto_aim_message(rmcs_map::msg::GameStatus& status)
    {
        // the pose of enemies
        for (int index = 0; index < enemies_pose_->size(); index++) {
            status.enemies[0].position.x = (*enemies_pose_)[index].x();
            status.enemies[0].position.y = (*enemies_pose_)[index].y();
        }
    }
};

} // namespace rmcs_map

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_map::Component, rmcs_executor::Component)