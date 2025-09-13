#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <vector>
#include <cmath>
#include <memory>
#include <mutex>

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
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ps4_subscriber;

int32_t step_height = 70;
int32_t step_length = 60;
int32_t body_height = 30;
int gait_speed_ms = 50;

int32_t rotation_step_length = 70;
int32_t rotation_step_height = 70;

float forward_stride_amplification = 1.25f;

int gait_phase = 0;
const int total_phases = 24;
int cycle_counter = 0;

float move_direction_x = 0.0f;
float move_direction_y = 0.0f;
float rotate_direction = 0.0f;
std::mutex control_mutex;

// Neutral position tracking
bool in_neutral_position = true;
const int32_t NEUTRAL_POSITION = 2048; //--- 180 deg

// Group 1: Legs 0 (front right), 4 (middle left), 2 (back right)
// Group 2: Legs 1 (middle right), 3 (front left), 5 (back left)
const int tripod1_leg_indices[3] = {0, 4, 2};
const int tripod2_leg_indices[3] = {1, 3, 5};

const int32_t y_rest = j2l;
const int32_t z_rest = -j3l;
const float j2l_sq = j2l * j2l;
const float j3l_sq = j3l * j3l;

std::vector<dynamixel_sdk_custom_interfaces::msg::SetPosition> message_batch;

void initialize_legs()
{
    //--- Right legs
    legs[0].id[0] = 1; legs[0].id[1] = 2; legs[0].id[2] = 3;    // Front right
    legs[1].id[0] = 4; legs[1].id[1] = 5; legs[1].id[2] = 6;    // Middle right
    legs[2].id[0] = 7; legs[2].id[1] = 8; legs[2].id[2] = 9;    // Back right
    
    //--- Left legs
    legs[3].id[0] = 10; legs[3].id[1] = 11; legs[3].id[2] = 12; // Front left
    legs[4].id[0] = 13; legs[4].id[1] = 14; legs[4].id[2] = 15; // Middle left
    legs[5].id[0] = 16; legs[5].id[1] = 17; legs[5].id[2] = 18; // Back left
}

void ps4_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 8)
    { 
        float ly = msg->data[1];
        float rx = msg->data[3];
        
        const float deadzone = 0.25f;
        if (fabs(ly) < deadzone) ly = 0.0f;
        if (fabs(rx) < deadzone) rx = 0.0f;
        
        std::lock_guard<std::mutex> lock(control_mutex);
        move_direction_x = -ly;
        move_direction_y = 0.0f;
        rotate_direction = rx;
        
        RCLCPP_DEBUG(rclcpp::get_logger("hexapod_gait_publisher"), 
                    "PS4 Control: LY=%.2f, RX=%.2f -> X=%.2f, R=%.2f", 
                    ly, rx, move_direction_x, rotate_direction);
    }
}

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

void get_movement_parameters(float& effective_step_len, float& effective_step_hgt)
{
    effective_step_len = step_length;
    effective_step_hgt = step_height;
    
    if (fabs(rotate_direction) > 0.5f && fabs(move_direction_x) < 0.3f)
    {
        effective_step_len = rotation_step_length;
        effective_step_hgt = rotation_step_height;
    }
}

void calculate_forward_trajectory(int leg_id, float phase, float& x_pos, float& y_pos, float& z_pos, 
                                 float effective_step_len, float effective_step_hgt)
{
    bool is_left_leg = (leg_id == 3 || leg_id == 4 || leg_id == 5);
    bool y_adjust_mirror = (leg_id == 1 || leg_id == 2);
    
    float leg_time = std::max(0.0f, std::min(1.0f, phase));
    
    float amplified_step_len = effective_step_len * forward_stride_amplification;
    
    x_pos = -amplified_step_len/2 + smooth_step(leg_time) * amplified_step_len;
    y_pos = 20;
    z_pos = body_height - lift_profile(leg_time, effective_step_hgt);
    
    x_pos *= move_direction_x;
    
    if (is_left_leg)
    {
        x_pos = -x_pos;
        z_pos = -z_pos;
    }
    if(y_adjust_mirror)
    {
        y_pos = -y_pos;
    }
}

void calculate_rotation_trajectory(int leg_id, float phase, float& x_pos, float& y_pos, float& z_pos,
                                  float effective_step_len, float effective_step_hgt)
{
    bool is_left_leg = (leg_id == 3 || leg_id == 4 || leg_id == 5);
    
    float leg_time = std::max(0.0f, std::min(1.0f, phase));
    
    if (is_left_leg)
    {
        if (phase < 0.5f)
        {
            x_pos = -effective_step_len/2 + smooth_step(leg_time * 2.0f) * effective_step_len;
        }
        else
        {
            x_pos = effective_step_len/2 - smooth_step((leg_time - 0.5f) * 2.0f) * effective_step_len;
        }
        x_pos *= -rotate_direction;
    }
    else
    {
        if (phase < 0.5f)
        {
            x_pos = effective_step_len/2 - smooth_step(leg_time * 2.0f) * effective_step_len;
        }
        else
        {
            x_pos = -effective_step_len/2 + smooth_step((leg_time - 0.5f) * 2.0f) * effective_step_len;
        }
        x_pos *= rotate_direction;
    }
    
    y_pos = 20;
    
    if (phase < 0.5f)
    {
        z_pos = body_height - lift_profile(leg_time * 2.0f, effective_step_hgt);
    }
    else
    {
        z_pos = body_height; 
    }
    
    if (is_left_leg)
    {
        z_pos = -z_pos;
    }
}

void set_all_legs_to_neutral()
{
    for (int leg_id = 0; leg_id < 6; leg_id++)
    {
        for(int joint_id = 0; joint_id < 3; joint_id++)
        {
            legs[leg_id].position[joint_id] = NEUTRAL_POSITION;
            
            dynamixel_sdk_custom_interfaces::msg::SetPosition message;
            message.id = legs[leg_id].id[joint_id];
            message.position = legs[leg_id].position[joint_id];
            message_batch.push_back(message);
        }
    }
}

void timer_callback()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    float current_move_x, current_move_y, current_rotate;
    {
        std::lock_guard<std::mutex> lock(control_mutex);
        current_move_x = move_direction_x;
        current_move_y = move_direction_y;
        current_rotate = rotate_direction;
    }
    
    if (fabs(current_move_x) < 0.01f && fabs(current_rotate) < 0.01f)
    {
        if (!in_neutral_position)
        {
            message_batch.clear();
            message_batch.reserve(18);
            
            set_all_legs_to_neutral();
            
            for (const auto& message : message_batch)
            {
                publisher->publish(message);
            }
            
            in_neutral_position = true;
            RCLCPP_INFO(rclcpp::get_logger("hexapod_gait_publisher"), 
                       "No input detected. Returning to neutral position (180 degrees).");
        }
        return;
    }
    else
    {
        if (in_neutral_position)
        {
            in_neutral_position = false;
            RCLCPP_INFO(rclcpp::get_logger("hexapod_gait_publisher"), 
                       "Input detected. Exiting neutral position.");
        }
    }
    
    float phase_progress = static_cast<float>(gait_phase) / total_phases;
    
    message_batch.clear();
    message_batch.reserve(18);
    
    float effective_step_len, effective_step_hgt;
    get_movement_parameters(effective_step_len, effective_step_hgt);
    
    for (int i = 0; i < 3; i++)
    {
        /* Tripod 1 */
        int leg_id1 = tripod1_leg_indices[i];
        float x_pos1, y_pos1, z_pos1;
        
        if (fabs(current_rotate) > 0.5f && fabs(current_move_x) < 0.3f)
        {
            calculate_rotation_trajectory(leg_id1, phase_progress, x_pos1, y_pos1, z_pos1, 
                                         effective_step_len, effective_step_hgt);
        }
        else
        {
            calculate_forward_trajectory(leg_id1, phase_progress, x_pos1, y_pos1, z_pos1,
                                        effective_step_len, effective_step_hgt);
        }
        
        set_leg_position(&legs[leg_id1], static_cast<int32_t>(x_pos1), static_cast<int32_t>(y_pos1), static_cast<int32_t>(z_pos1));
        
        for(int joint_id = 0; joint_id < 3; joint_id++)
        {
            dynamixel_sdk_custom_interfaces::msg::SetPosition message;
            message.id = legs[leg_id1].id[joint_id];
            message.position = legs[leg_id1].position[joint_id];
            message_batch.push_back(message);
        }

        /* Tripod 2 */
        int leg_id2 = tripod2_leg_indices[i];
        float x_pos2, y_pos2, z_pos2;
        
        float phase_offset = std::fmod(phase_progress + 0.5f, 1.0f);
        
        if (fabs(current_rotate) > 0.5f && fabs(current_move_x) < 0.3f)
        {
            calculate_rotation_trajectory(leg_id2, phase_offset, x_pos2, y_pos2, z_pos2,
                                         effective_step_len, effective_step_hgt);
        }
        else
        {
            calculate_forward_trajectory(leg_id2, phase_offset, x_pos2, y_pos2, z_pos2,
                                        effective_step_len, effective_step_hgt);
        }
        
        set_leg_position(&legs[leg_id2], static_cast<int32_t>(x_pos2), static_cast<int32_t>(y_pos2), static_cast<int32_t>(z_pos2));
        
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
                "Cycle: %d, Phase: %d/%d, Direction: (%.1f, %.1f, %.1f), Processing time: %ld ms", 
                cycle_counter, gait_phase, total_phases, 
                current_move_x, current_move_y, current_rotate,
                duration.count());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hexapod_gait_publisher");
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher = node->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", qos);
    
    ps4_subscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "ps4_controller", 10, ps4_callback);
    
    timer = node->create_wall_timer(std::chrono::milliseconds(gait_speed_ms), timer_callback);
    
    initialize_legs();
    
    RCLCPP_INFO(node->get_logger(), "Hexapod Gait Publisher started for 18 servos");
    RCLCPP_INFO(node->get_logger(), "Gait speed set to %d ms per phase", gait_speed_ms);
    RCLCPP_INFO(node->get_logger(), "Step height: %d, Step length: %d", step_height, step_length);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}