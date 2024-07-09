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
        // input from referee system
        register_input("/referee/friends/hero/hp", friends_hero_hp_);
        register_input("/referee/friends/engineer/hp", friends_engineer_hp_);
        register_input("/referee/friends/infantry_iii/hp", friends_infantry_iii_hp_);
        register_input("/referee/friends/infantry_iv/hp", friends_infantry_iv_hp_);
        register_input("/referee/friends/infantry_v/hp", friends_infantry_v_hp_);
        register_input("/referee/friends/sentry/hp", friends_sentry_hp_);
        register_input("/referee/friends/outpost/hp", friends_outpost_hp_);
        register_input("/referee/friends/base/hp", friends_base_hp_);

        register_input("/referee/friends/hero/position", friends_hero_pose_);
        register_input("/referee/friends/engineer/position", friends_engineer_pose_);
        register_input("/referee/friends/infantry_iii/position", friends_infantry_iii_pose_);
        register_input("/referee/friends/infantry_iv/position", friends_infantry_iv_pose_);
        register_input("/referee/friends/infantry_v/position", friends_infantry_v_pose_);
        register_input("/referee/friends/sentry/position", friends_sentry_pose_);

        register_input("/referee/enemies/hero/hp", enemies_hero_hp_);
        register_input("/referee/enemies/engineer/hp", enemies_engineer_hp_);
        register_input("/referee/enemies/infantry_iii/hp", enemies_infantry_iii_hp_);
        register_input("/referee/enemies/infantry_iv/hp", enemies_infantry_iv_hp_);
        register_input("/referee/enemies/infantry_v/hp", enemies_infantry_v_hp_);
        register_input("/referee/enemies/sentry/hp", enemies_sentry_hp_);
        register_input("/referee/enemies/outpost/hp", enemies_outpost_hp_);
        register_input("/referee/enemies/base/hp", enemies_base_hp_);

        register_input("/referee/shooter/bullet_allowance", bullet_);

        // input from auto aim system
        register_input("/referee/enemies/hero/position", enemies_hero_pose_);
        register_input("/referee/enemies/engineer/position", enemies_engineer_pose_);
        register_input("/referee/enemies/infantry_iii/position", enemies_infantry_iii_pose_);
        register_input("/referee/enemies/infantry_iv/position", enemies_infantry_iv_pose_);
        register_input("/referee/enemies/infantry_v/position", enemies_infantry_v_pose_);
        register_input("/referee/enemies/sentry/position", enemies_sentry_pose_);
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
    InputInterface<uint16_t> friends_hero_hp_;
    InputInterface<uint16_t> friends_engineer_hp_;
    InputInterface<uint16_t> friends_infantry_iii_hp_;
    InputInterface<uint16_t> friends_infantry_iv_hp_;
    InputInterface<uint16_t> friends_infantry_v_hp_;
    InputInterface<uint16_t> friends_sentry_hp_;
    InputInterface<uint16_t> friends_outpost_hp_;
    InputInterface<uint16_t> friends_base_hp_;

    InputInterface<Eigen::Vector2d> friends_hero_pose_;
    InputInterface<Eigen::Vector2d> friends_engineer_pose_;
    InputInterface<Eigen::Vector2d> friends_infantry_iii_pose_;
    InputInterface<Eigen::Vector2d> friends_infantry_iv_pose_;
    InputInterface<Eigen::Vector2d> friends_infantry_v_pose_;
    InputInterface<Eigen::Vector2d> friends_sentry_pose_;

    InputInterface<uint16_t> enemies_hero_hp_;
    InputInterface<uint16_t> enemies_engineer_hp_;
    InputInterface<uint16_t> enemies_infantry_iii_hp_;
    InputInterface<uint16_t> enemies_infantry_iv_hp_;
    InputInterface<uint16_t> enemies_infantry_v_hp_;
    InputInterface<uint16_t> enemies_sentry_hp_;
    InputInterface<uint16_t> enemies_outpost_hp_;
    InputInterface<uint16_t> enemies_base_hp_;

    InputInterface<Eigen::Vector2d> enemies_hero_pose_;
    InputInterface<Eigen::Vector2d> enemies_engineer_pose_;
    InputInterface<Eigen::Vector2d> enemies_infantry_iii_pose_;
    InputInterface<Eigen::Vector2d> enemies_infantry_iv_pose_;
    InputInterface<Eigen::Vector2d> enemies_infantry_v_pose_;
    InputInterface<Eigen::Vector2d> enemies_sentry_pose_;

    InputInterface<uint16_t> bullet_;

    void collect_referee_message(rmcs_map::msg::GameStatus& status)
    {
        // the hp of friends
        status.friends_hero.hp         = *friends_hero_hp_;
        status.friends_engineer.hp     = *friends_engineer_hp_;
        status.friends_infantry_iii.hp = *friends_infantry_iii_hp_;
        status.friends_infantry_iv.hp  = *friends_infantry_iv_hp_;
        status.friends_infantry_v.hp   = *friends_infantry_v_hp_;
        status.friends_sentry.hp       = *friends_sentry_hp_;
        status.friends_outpost_hp      = *friends_outpost_hp_;
        status.friends_base_hp         = *friends_base_hp_;

        // the hp of enemies
        status.enemies_hero.hp         = *enemies_hero_hp_;
        status.enemies_engineer.hp     = *enemies_engineer_hp_;
        status.enemies_infantry_iii.hp = *enemies_infantry_iii_hp_;
        status.enemies_infantry_iv.hp  = *enemies_infantry_iv_hp_;
        status.enemies_infantry_v.hp   = *enemies_infantry_v_hp_;
        status.enemies_sentry.hp       = *enemies_sentry_hp_;
        status.enemies_outpost_hp      = *enemies_outpost_hp_;
        status.enemies_base_hp         = *enemies_base_hp_;

        // the hp of base and outpost
        status.friends_base_hp    = *friends_base_hp_;
        status.friends_outpost_hp = *friends_outpost_hp_;
        status.enemies_base_hp    = *enemies_base_hp_;
        status.enemies_outpost_hp = *enemies_outpost_hp_;

        // the pose of friends
        status.friends_hero.pose.x         = (*friends_hero_pose_).x();
        status.friends_hero.pose.y         = (*friends_hero_pose_).y();
        status.friends_engineer.pose.x     = (*friends_engineer_pose_).x();
        status.friends_engineer.pose.y     = (*friends_engineer_pose_).y();
        status.friends_infantry_iii.pose.x = (*friends_infantry_iii_pose_).x();
        status.friends_infantry_iii.pose.y = (*friends_infantry_iii_pose_).y();
        status.friends_infantry_iv.pose.x  = (*friends_infantry_iv_pose_).x();
        status.friends_infantry_iv.pose.y  = (*friends_infantry_iv_pose_).y();
        status.friends_infantry_v.pose.x   = (*friends_infantry_v_pose_).x();
        status.friends_infantry_v.pose.y   = (*friends_infantry_v_pose_).y();
        status.friends_sentry.pose.x       = (*friends_sentry_pose_).x();
        status.friends_sentry.pose.y       = (*friends_sentry_pose_).y();

        // the bullet of self
        status.bullet = *bullet_;
    }

    void collect_auto_aim_message(rmcs_map::msg::GameStatus& status)
    {
        // the pose of enemies
        status.enemies_hero.pose.x         = (*enemies_hero_pose_).x();
        status.enemies_hero.pose.y         = (*enemies_hero_pose_).y();
        status.enemies_engineer.pose.x     = (*enemies_engineer_pose_).x();
        status.enemies_engineer.pose.y     = (*enemies_engineer_pose_).y();
        status.enemies_infantry_iii.pose.x = (*enemies_infantry_iii_pose_).x();
        status.enemies_infantry_iii.pose.y = (*enemies_infantry_iii_pose_).y();
        status.enemies_infantry_iv.pose.x  = (*enemies_infantry_iv_pose_).x();
        status.enemies_infantry_iv.pose.y  = (*enemies_infantry_iv_pose_).y();
        status.enemies_infantry_v.pose.x   = (*enemies_infantry_v_pose_).x();
        status.enemies_infantry_v.pose.y   = (*enemies_infantry_v_pose_).y();
        status.enemies_sentry.pose.x       = (*enemies_sentry_pose_).x();
        status.enemies_sentry.pose.y       = (*enemies_sentry_pose_).y();
    }
};

} // namespace rmcs_map

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_map::Component, rmcs_executor::Component)