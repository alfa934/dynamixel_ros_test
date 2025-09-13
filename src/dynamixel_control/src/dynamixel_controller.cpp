#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"

#define PROTOCOL_VERSION 1.0
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36
#define ADDR_MX_MOVING_SPEED 32

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncWrite *groupSyncWrite;

std::vector<uint8_t> dynamixel_ids = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
std::vector<uint16_t> goal_positions(18, 2048); //--- default to center position for 18 servos
std::mutex goal_mutex; //--- mutex to protect goal_positions

rclcpp::Node::SharedPtr node;
rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_subscriber;
rclcpp::Service<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr get_position_server;
rclcpp::TimerBase::SharedPtr sync_write_timer;

void set_position_callback(const dynamixel_sdk_custom_interfaces::msg::SetPosition::SharedPtr msg)
{
    if (msg->id < 1 || msg->id > 18)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid Dynamixel ID: %d. Must be between 1 and 18", msg->id);
        return;
    }
    
    if (msg->position < 0 || msg->position > 4095)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid position: %d. Must be between 0-4095", msg->position);
        return;
    }
    
    std::lock_guard<std::mutex> lock(goal_mutex);
    goal_positions[msg->id - 1] = static_cast<uint16_t>(msg->position);
    
    static int log_counter = 0;
    if (++log_counter % 50 == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Set goal position for ID %d: %d", msg->id, msg->position);
    }
}

void sync_write_callback()
{
    //--- Use local copy to minimize mutex locking time
    std::vector<uint16_t> local_goal_positions;
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        local_goal_positions = goal_positions;
    }
    
    bool dxl_addparam_result = false;
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t param_goal_position[2];
    
    groupSyncWrite->clearParam();
    
    for (size_t i = 0; i < dynamixel_ids.size(); i++)
    {
        param_goal_position[0] = DXL_LOBYTE(local_goal_positions[i]);
        param_goal_position[1] = DXL_HIBYTE(local_goal_positions[i]);
        
        dxl_addparam_result = groupSyncWrite->addParam(dynamixel_ids[i], param_goal_position);
        
        if (!dxl_addparam_result)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to add param to syncwrite for ID %d", dynamixel_ids[i]);
        }
    }
    
    dxl_comm_result = groupSyncWrite->txPacket();

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Syncwrite failed: %s", packetHandler->getTxRxResult(dxl_comm_result));
    }
}

void get_position_callback(
    const std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request> request,
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> response)
{
    if (request->id < 1 || request->id > 18)
    {
        RCLCPP_ERROR(node->get_logger(), "Invalid Dynamixel ID: %d. Must be between 1 and 18", request->id);
        response->position = -1;
        return;
    }
    
    uint8_t dxl_error = 0;
    uint16_t present_position = 0;

    int dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        request->id,
        ADDR_MX_PRESENT_POSITION,
        &present_position,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to read position: %s", packetHandler->getTxRxResult(dxl_comm_result));
        response->position = -1;
    }
    else if (dxl_error != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Dynamixel error: %s", packetHandler->getRxPacketError(dxl_error));
        response->position = -1;
    }
    else
    {
        response->position = static_cast<int32_t>(present_position);
        RCLCPP_INFO(node->get_logger(), "Get position [ID: %d]: %d", request->id, present_position);
    }
}

bool setup_dynamixel(uint8_t dxl_id)
{
    //--- enable torque
    int dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_MX_TORQUE_ENABLE,
        1,
        nullptr
    );

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to enable torque on ID %d", dxl_id);
        return false;
    }

    //--- set a speed or something
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_MX_MOVING_SPEED,
        100,
        nullptr
    );
    
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_WARN(node->get_logger(), "Failed to set moving speed on ID %d, but torque enabled", dxl_id);
    }
    
    RCLCPP_INFO(node->get_logger(), "Successfully enabled torque on ID %d", dxl_id);
    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("dynamixel_controller");

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, 2);

    if (!portHandler->openPort())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open port!");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Successfully opened the port");

    if (!portHandler->setBaudRate(BAUDRATE))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to set the baudrate!");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Successfully set the baudrate to %d", BAUDRATE);

    for (auto id : dynamixel_ids)
    {
        if (!setup_dynamixel(id))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to setup Dynamixel ID %d", id);
            return -1;
        }
    }

    set_position_subscriber = node->create_subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
        "set_position", 100, set_position_callback);

    get_position_server = node->create_service<dynamixel_sdk_custom_interfaces::srv::GetPosition>(
        "get_position", get_position_callback);
    
    sync_write_timer = node->create_wall_timer(std::chrono::milliseconds(20), sync_write_callback);

    RCLCPP_INFO(node->get_logger(), "Dynamixel controller started for 18 MX-28 servos");

    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "Disabling torque on all servos");
    for (auto id : dynamixel_ids)
    {
        packetHandler->write1ByteTxRx(
            portHandler,
            id,
            ADDR_MX_TORQUE_ENABLE,
            0,
            nullptr
        );
    }

    delete groupSyncWrite;
    portHandler->closePort();

    return 0;
}