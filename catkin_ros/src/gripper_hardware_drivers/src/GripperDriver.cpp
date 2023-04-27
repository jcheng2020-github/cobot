//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>

#include "gripper_hardware_drivers/GripperDriver.h"

#include <thread>
#include <chrono>

#define FW_VERSION "0.0.1"

namespace gripper_hardware_drivers {

void GripperDriver::init(std::string port, int baudrate, int num_joints, std::vector<double>& enc_steps_per_deg)
{
    // @TODO read version from config
    version_ = FW_VERSION;

    // establish connection with teensy board
    boost::system::error_code ec;
    serial_port_.open(port, ec);

    if (ec)
    {
        ROS_WARN("Failed to connect to serial port %s", port.c_str());
        return;
    }
    else
    {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        ROS_INFO("Successfully connected to serial port %s", port.c_str());
    }
    
    initialised_ = false;
    std::string msg = "STA" + version_ + "\n";

    while (!initialised_)
    {
        ROS_INFO("Waiting for response from Gripper on port %s", port.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        exchange(msg);
    }
    ROS_INFO("Successfully initialised driver on port %s", port.c_str());

    // initialise joint and encoder calibration
    num_joints_ = num_joints;
    enc_commands_.resize(num_joints_);
    enc_steps_.resize(num_joints_);
    enc_steps_per_deg_.resize(num_joints_);
    enc_calibrations_.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        enc_steps_per_deg_[i] = enc_steps_per_deg[i];
    }

    // get current joint positions
    msg = "JP\n";
    exchange(msg);
    for (int i = 0; i < num_joints_; ++i)
    {
        enc_commands_[i] = enc_steps_[i];
    }
}

GripperDriver::GripperDriver()
: serial_port_(io_service_)
{
}
/*
void GripperDriver::setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel)
{
    std::string outMsg = "SS";
    for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2)
    {
        outMsg += 'A' + charIdx;
        outMsg += std::to_string(max_speed[i]);
        outMsg += 'A' + charIdx + 1;
        outMsg += std::to_string(max_accel[i]);
    }
    outMsg += "\n";
    exchange(outMsg);
}
*/

// Update between hardware interface and hardware driver
void GripperDriver::update(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
{
    // get updated position commands
    jointPosToEncSteps(pos_commands, enc_commands_);

    // construct update message
    std::string outMsg = "MT";
    for (int i = 0; i < num_joints_; ++i)
    {
        outMsg += 'A' + i;
        outMsg += std::to_string(enc_commands_[i]);
    }
    outMsg += "\n";
    
    //jcheng2020 revised
    std::string report = "Submitted Message to Gripper: " + outMsg;
    ROS_INFO(report.c_str());
    //jcheng2020 revised end

    // run the communication with board
    exchange(outMsg);
	
    /* jcehng2020 revised
    // return updated joint_positions
    encStepsToJointPos(enc_steps_ , joint_positions);
    */
    //jcheng2020 revised
    for(int i = 0; i < joint_positions.size(); ++i)
    {
    	joint_positions[i] = pos_commands[i];
    }
    //jcheng2020 revised end
}

void GripperDriver::calibrateJoints()
{
    std::string outMsg = "JC\n";
    sendCommand(outMsg);
}

void GripperDriver::getJointPositions(std::vector<double>& joint_positions)
{
    // get current joint positions
    std::string msg = "JP\n";
    exchange(msg);
    encStepsToJointPos(enc_steps_, joint_positions);
}

// Send specific commands
void GripperDriver::sendCommand(std::string outMsg)
{
    exchange(outMsg);
}

// Send msg to board and collect data
void GripperDriver::exchange(std::string outMsg)
{
    std::string inMsg;
    std::string errTransmit = "";
    std::string errReceive = "";
    
    if (!transmit(outMsg, errTransmit))
    {
    	ROS_ERROR("transmit(outMsg, errTransmit) error appears\n");
        // print err
    }

    if (!receive(inMsg, errReceive))
    {
    	ROS_ERROR("receive(inMsg, errReceive) error appears\n");
        // print err
    }
    // parse msg
    std::string header = inMsg.substr(0, 2);
    if (header == "ST")
    {
        // init acknowledgement
        checkInit(inMsg);   
    }
    else if (header == "JC")
    {
        // encoder calibration values
        updateEncoderCalibrations(inMsg);
    }
    else if (header == "JP")
    {
    	//jcheng2020 revised
    	std::string report = "Received EncSteps Message from Gripper: " + inMsg;
    	ROS_INFO(report.c_str());
    	//jcheng2020 revised end
        // encoder steps
        updateEncoderSteps(inMsg);
    }
    else
    {
        // unknown header
        ROS_WARN("Unknown header %s", header);
    } 
}

bool GripperDriver::transmit(std::string msg, std::string& err)
{
    boost::system::error_code ec;
    const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

    boost::asio::write(serial_port_, sendBuffer, ec);

    if(!ec)
    {
        return true;
    }
    else
    {
        err = "Error in transmit";
        //jcheng2020 revised
        ROS_WARN("GripperDriver::transmit ERROR in transmit");
        //jcheng2020 revised end
        return false;
    }    
}

bool GripperDriver::receive(std::string& inMsg, std::string& err)
{
    char c;
    std::string msg = "";
    bool eol = false;
    while (!eol)
    {
        boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                eol = true;
            default:
                msg += c;
        }
    }
    inMsg = msg;
    return true;
}

void GripperDriver::checkInit(std::string msg)
{
    std::size_t ack_idx = msg.find("A", 2) + 1;
    std::size_t version_idx = msg.find("B", 2) + 1;
    /* jcheng2020 revised
    int ack = std::stoi(msg.substr(ack_idx, version_idx));
    */
    //jcheng2020 revised
    int ack = std::stoi(msg.substr(ack_idx, version_idx - ack_idx - 1));
    //jcheng2020 revised end
    if (ack)
    {
        initialised_ = true;
    }
    else
    {
        std::string version = msg.substr(version_idx);
        ROS_ERROR("Firmware version mismatch %s", version);
    }  
}

void GripperDriver::updateEncoderCalibrations(std::string msg)
{
    //jcheng2020 revised
    enc_calibrations_[0] = 0;
    enc_calibrations_[1] = 0;
    //jcheng2020 revised end

    // @TODO update config file
    ROS_INFO("Successfully updated encoder calibrations");
}

void GripperDriver::updateEncoderSteps(std::string msg)
{
    /*
    size_t idx1 = msg.find("A", 2) + 1;
    size_t idx2 = msg.find("B", 2) + 1;
    size_t idx3 = msg.find("C", 2) + 1;
    size_t idx4 = msg.find("D", 2) + 1;
    size_t idx5 = msg.find("E", 2) + 1;
    size_t idx6 = msg.find("F", 2) + 1;
    enc_steps_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
    enc_steps_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
    enc_steps_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
    enc_steps_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
    enc_steps_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
    enc_steps_[5] = std::stoi(msg.substr(idx6));
    */
    
    //jcheng2020 revised
    const size_t stateBeginIndex = 2, stateIndexLength = 5;
    std::string gripperState = msg.substr( stateBeginIndex, stateIndexLength);
    if(gripperState == "close")
    {
    	enc_steps_[0] = 0;
    	enc_steps_[1] = 0;
    }
    else if(gripperState == "open_")
    {
    	enc_steps_[0] = 90;
    	enc_steps_[1] = 90;
    }
    else
    {
    	ROS_WARN( "void GripperDriver::updateEncoderSteps Error\n");
    }
    //jcheng2020 revised end
}

void GripperDriver::encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions)
{
    for (int i = 0; i < enc_steps.size(); ++i)
    {
    	//for gripper enc_steps[i]: [0, 90], if enc_steps_per_deg_[i] == 1, then joint_positions[i]: [ 0, 90]
        // convert enc steps to joint deg
        joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
        //jcheng2020 revised angle to distance 90/0.032 = 2812.5
        joint_positions[i] = joint_positions[i] / 2812.5;
        //jcheng2020 revised end
    }
}

void GripperDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
    	/*
        // convert joint deg to enc steps
        enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
        */
        //jcheng2020 revised angle to distance 90/0.032 = 2812.5 , distance 0.012 m is the cirtial point hence 34 is the critical angle
        //jcheng2020 revised angle to distance 90/0.032 = 2812.5 , distance 0.020 m is the cirtial point hence 56.25 is the critical angle
        enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i] * 2812.5);
        //jcheng2020 revised end
    }
}

} // namespace gripper_hardware_drivers
