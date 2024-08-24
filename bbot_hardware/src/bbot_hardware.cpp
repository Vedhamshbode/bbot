#include <chrono> //write(), read() , close()
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <fcntl.h> //contains files controls like O_RDWR ( read/write access )
#include <errno.h> //error integer and strerror() function
#include <unistd.h>


#include "bbot_hardware/bbot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bbot_hardware
{
    hardware_interface::CallbackReturn BbotHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        
        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "PID values not supplied, using defaults.");
        }

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        
        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BbotHardware"),"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BbotHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BbotHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BbotHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BbotHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    int BbotHardware::WriteToSerial(unsigned char* buf, int nBytes){
        return ::write(SerialPort, buf, nBytes);
    }

    int BbotHardware::ReadSerial(unsigned char* buf, int nBytes){
        auto t_start = std::chrono::high_resolution_clock::now();
        int n=0;
        while(n<nBytes){
            int ret = ::read(SerialPort, &(buf[n]), 1);
            if (ret < 0){
                return ret;
            }
            n+=ret;
            auto t_end = std::chrono::high_resolution_clock::now();
            double elasped_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
            if (elasped_time_ms>1000){
                break;
            }
        }
        return n;
    }


//////////////////////////////////////////////////On Configure////////////////////////////////////////////////

    hardware_interface::CallbackReturn BbotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Configuring ...please wait...");
    for (uint i = 0; i < hw_states_position_.size(); i++){
        hw_states_position_[i]=0;
        hw_states_velocity_[i]=0;
        hw_commands_[i]=0;
    }
    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
    }

///////////////////////////////////////////////////export_state_interfaces////////////////////////////////////////

    std::vector<hardware_interface::StateInterface> BbotHardware::export_state_interfaces()
    {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i =0 ; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }
    return state_interfaces;
    }

///////////////////////////////////////////////////export_command_interfaces////////////////////////////////////////

    std::vector<hardware_interface::CommandInterface> BbotHardware::export_command_interfaces()
    {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i =0 ; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
    }


///////////////////////////////////////////////////On Activate////////////////////////////////////////


    hardware_interface::CallbackReturn BbotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Activating ...please wait...");
    std::string port = "/dev/ttyUSB0";
    SerialPort = open(port.c_str(), O_RDWR);
    if (SerialPort <0){
        RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Error %i from open: %s", errno, strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (tcgetattr(SerialPort, &tty) !=0){
        RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return hardware_interface::CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB;        // Disable parity bit
    tty.c_cflag &= ~CSTOPB;        // Use one stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver and set local mode

    tty.c_lflag &= ~ICANON;        // Disable canonical mode
    tty.c_lflag &= ~ECHO;          // Disable echo
    tty.c_lflag &= ~ECHOE;         // Disable erasure echo
    tty.c_lflag &= ~ECHONL;        // Disable new-line echo
    tty.c_lflag &= ~ISIG;          // Disable signal interpretation
    tty.c_lflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable miscellaneous input flags

    tty.c_oflag &= ~OPOST;         // Disable output processing
    tty.c_oflag &= ~ONLCR;         // Disable mapping of NL to CR-NL on output

    tty.c_cc[VTIME] = 1;           // Timeout in deciseconds
    tty.c_cc[VMIN] = 0;            // Minimum number of characters for non-canonical read

    speed_t speed = B115200;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tcflush(SerialPort, TCIOFLUSH);

    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0 ){
        RCLCPP_ERROR(rclcpp::get_logger("hardwareInterface"), "Error %i from tcsetattr %s", errno, strerror(errno));
        close(SerialPort);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);
    auto t_start = std::chrono::high_resolution_clock::now();
    while(true){
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms>3000){
            break;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
    }

///////////////////////////////////////////////////On Deactivate////////////////////////////////////////

    hardware_interface::CallbackReturn BbotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Deactivating ...please wait...");

    tcflush(SerialPort, TCIOFLUSH);
    close(SerialPort);

    RCLCPP_INFO(rclcpp::get_logger("BbotHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
    }

///////////////////////////////////////////////////   Read   //////////////////////////////////////////


    hardware_interface::return_type BbotHardware::read(const rclcpp::Time &, const rclcpp::Duration &){
        unsigned char r[1]={'r'};
        WriteToSerial(r, 1);
        float ret[]= {0,0,0,0,0,0,0,0};
        uint8_t* v = (uint8_t*)ret;
        ReadSerial(v, sizeof(ret));
        RCLCPP_INFO(rclcpp::get_logger("Hardwareinterface"), "Received : %f, %f, %f, %f, %f, %f, %f, %f", ret[0], ret[1], ret[2], ret[3], ret[4], ret[5], ret[6], ret[7]);
        return hardware_interface::return_type::OK;
    }

///////////////////////////////////////////////////   Write   /////////////////////////////////////////


    hardware_interface::return_type BbotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
    return hardware_interface::return_type::OK;
    }

}  // namespace bbot_hardware

//This is for making this bbothardware into a ros2 plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bbot_hardware::BbotHardware, hardware_interface::SystemInterface)
