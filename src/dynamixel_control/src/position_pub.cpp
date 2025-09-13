#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <chrono>
#include <vector>
#include <cmath>
#include <memory>

#define j2l 80
#define j3l 62

const float PI = M_PI;
const float DEG_TO_RAD = PI / 180.0f;
const float RAD_TO_DEG = 180.0f / PI;

typedef struct 
{
    uint8_t id[3];
    int32_t position[3];
} Dynamixel_Servo_t;

Dynamixel_Servo_t legs[6];
rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher;
rclcpp::TimerBase::SharedPtr timer;

int32_t step_height = 50;
int32_t step_length = 40;
int32_t body_height = 30;
int gait_speed_ms = 50;

int gait_phase = 0;
const int total_phases = 24;
int cycle_counter = 0;

// Group 1: Legs 0 (front right), 4 (middle left), 2 (back right)
// Group 2: Legs 1 (middle right), 3 (front left), 5 (back left)
const int tripod1_leg_indices[3] = {0, 4, 2};
const int tripod2_leg_indices[3] = {1, 3, 5};

const int32_t y_rest = j2l;
const int32_t z_rest = -j3l;
const float j2l_sq = j2l * j2l;
const float j3l_sq = j3l * j3l;

std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> message_batch;

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t deg_to_pos(int32_t degree)
{
    return map(degree, 0, 359, 0, 4095);
}

void inverse_kinematics(Dynamixel_Servo_t *joint_input, int32_t x_input, int32_t y_input, int32_t z_input)
{
    float x_sq = x_input * x_input;
    float y_sq = y_input * y_input;
    float z_sq = z_input * z_input;
    
    float h = std::sqrt(y_sq + x_sq);
    float h_sq = h * h;
    float l = std::sqrt(h_sq + z_sq);
    float l_sq = l * l;
    
    float a = std::atan2(z_input, h);
    float b = std::acos((j2l_sq - j3l_sq + l_sq) / (2 * j2l * l));
    
    float j1 = std::atan2(x_input, y_input) + PI; // 180 deg offset
    float j2 = a + b + PI; // 180 deg offset
    float j3 = std::acos((j2l_sq + j3l_sq - l_sq) / (2 * j2l * j3l)) + (PI / 2); // 90 deg offset
    
    joint_input->position[0] = deg_to_pos(j1 * RAD_TO_DEG);
    joint_input->position[1] = deg_to_pos(j2 * RAD_TO_DEG);
    joint_input->position[2] = deg_to_pos(j3 * RAD_TO_DEG);
}

void set_leg_position(Dynamixel_Servo_t *joint_input, int32_t x_goal, int32_t y_goal, int32_t z_goal)
{
    int32_t y_goal_total = y_rest - y_goal;

    int32_t z_goal_total = z_rest - z_goal;

    inverse_kinematics(joint_input, x_goal, y_goal_total, z_goal_total);
}

float smooth_step(float t)
{
    //--- Cubic smootherstep function
    return t * t * t * (t * (t * 6 - 15) + 10);
}

float lift_profile(float t, float max_height)
{
    /*
     * Smoother sinusoidal lift profile with easing
     * Lifting phase with ease-out and
     * lowering phase with ease-in
     */
    if (t < 0.5f)
    {
        return max_height * std::sin(PI * t * 0.5f);
    }
    else
    {
        return max_height * std::sin(PI * (0.5f + (t - 0.5f) * 0.5f));
    }
}

void timer_callback()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    float phase_progress = static_cast<float>(gait_phase) / total_phases;
    
    message_batch.clear();
    message_batch.reserve(18);
    
    for (int i = 0; i < 3; i++)
    {
        /* Tripod 1 */
        int leg_id1 = tripod1_leg_indices[i];

        bool is_left_leg1 = (leg_id1 == 4);
        bool y_adjust_mirror_1 = (leg_id1 == 2);

        float leg_time1 = phase_progress;
        
        leg_time1 = std::max(0.0f, std::min(1.0f, leg_time1));
        
        float x_pos1 = -step_length/2 + smooth_step(leg_time1) * step_length;
        float y_pos1 = 20;
        float z_pos1 = body_height - lift_profile(leg_time1, step_height);
        
        
        if (is_left_leg1)
        {
            x_pos1 = -x_pos1;
            z_pos1 = -z_pos1;
        }
        if(y_adjust_mirror_1)
        {
            y_pos1 = -y_pos1;
        }

        set_leg_position(&legs[leg_id1], static_cast<int32_t>(x_pos1), y_pos1, static_cast<int32_t>(z_pos1));
        
        for(int joint_id = 0; joint_id < 3; joint_id++)
        {
            dynamixel_sdk_custom_interfaces::msg::SetPosition message;
            message.id = legs[leg_id1].id[joint_id];
            message.position = legs[leg_id1].position[joint_id];
            message_batch.push_back(message);
        }

        /* Tripod 2 */
        int leg_id2 = tripod2_leg_indices[i];

        bool is_left_leg2 = (leg_id2 == 3 || leg_id2 == 5);
        bool y_adjust_mirror_2 = (leg_id2 == 1);
        
        float leg_time2 = std::fmod(phase_progress + 0.5f, 1.0f); //--- 180 degree phase offset
        
        leg_time2 = std::max(0.0f, std::min(1.0f, leg_time2));
        
        float x_pos2 = -step_length/2 + smooth_step(leg_time2) * step_length;
        float y_pos2 = 20;
        float z_pos2 = body_height - lift_profile(leg_time2, step_height);

        if (is_left_leg2)
        {
            x_pos2 = -x_pos2;
            z_pos2 = -z_pos2;
        }
        if(y_adjust_mirror_2)
        {
            y_pos2 = -y_pos2;
        }

        set_leg_position(&legs[leg_id2], static_cast<int32_t>(x_pos2), y_pos2, static_cast<int32_t>(z_pos2));
        
        for(int joint_id = 0; joint_id < 3; joint_id++)
        {
            dynamixel_sdk_custom_interfaces::msg::SetPosition message;
            message.id = legs[leg_id2].id[joint_id];
            message.position = legs[leg_id2].position[joint_id];
            message_batch.push_back(message);
        }
    }
    
    for (const auto& message : message_batch)
    {
        publisher->publish(message);
    }
    
    gait_phase = (gait_phase + 1) % total_phases;
    
    cycle_counter++;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (cycle_counter % 10 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("hexapod_gait_publisher"), 
                "Cycle: %d, Phase: %d/%d, Processing time: %ld ms", 
                cycle_counter, gait_phase, total_phases, duration.count());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hexapod_gait_publisher");
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher = node->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", qos);
    
    timer = node->create_wall_timer(std::chrono::milliseconds(gait_speed_ms), timer_callback);
    
    RCLCPP_INFO(node->get_logger(), "Hexapod Gait Publisher started for 18 servos");
    RCLCPP_INFO(node->get_logger(), "Gait speed set to %d ms per phase", gait_speed_ms);
    RCLCPP_INFO(node->get_logger(), "Step height: %d (taller gait)", step_height);
    RCLCPP_INFO(node->get_logger(), "Tripod 1: Legs 0, 4, 2");
    RCLCPP_INFO(node->get_logger(), "Tripod 2: Legs 1, 3, 5");

    //--- right legs
    legs[0].id[0] = 1; legs[0].id[1] = 2; legs[0].id[2] = 3;    // Front right
    legs[1].id[0] = 4; legs[1].id[1] = 5; legs[1].id[2] = 6;    // Middle right
    legs[2].id[0] = 7; legs[2].id[1] = 8; legs[2].id[2] = 9;    // Back right
    
    //--- left legs
    legs[3].id[0] = 10; legs[3].id[1] = 11; legs[3].id[2] = 12; // Front left
    legs[4].id[0] = 13; legs[4].id[1] = 14; legs[4].id[2] = 15; // Middle left
    legs[5].id[0] = 16; legs[5].id[1] = 17; legs[5].id[2] = 18; // Back left
    
    message_batch.reserve(18);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}