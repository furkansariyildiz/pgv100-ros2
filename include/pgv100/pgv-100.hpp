#ifndef PGV_100_HPP
#define PGV_100_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pgv100/msg/pgv_scan.hpp>
#include <pgv100/msg/pgv_command.hpp>

#include <chrono>
#include <future> 
#include <iostream>
#include <string.h>
#include <cstring>
#include <cstdlib>
#include <stdexcept>

#include <regex>

#include <libusb-1.0/libusb.h>
#include <cassert>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <features.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

using namespace std;


/*
 * Developed for the PGV-100 ROS2 Driver.
 * This package was inspired by <https://github.com/Ermanas/pf_pgv100/tree/master>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */



class PGV100: public rclcpp::Node
{
    public:
        PGV100();
        ~PGV100();

        rclcpp::Subscription<pgv100::msg::PGVCommand>::SharedPtr _pgv_direction_subscriber;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _pgv_calibration_subscriber;
        
        rclcpp::Publisher<pgv100::msg::PGVScan>::SharedPtr _pgv_scan_publisher;

        rclcpp::TimerBase::SharedPtr _timer;

        pgv100::msg::PGVCommand _pgv_command_message;

        std_msgs::msg::String _calibration_message;

        pgv100::msg::PGVScan _pgv_scan_message;

        void pgvDirectionCallback(const pgv100::msg::PGVCommand::SharedPtr message);

        void pgvCalibrationCallback(const std_msgs::msg::String::SharedPtr message);

        int settingBaudrate(int baudrate);

        void prepareTermios(termios* tty, int serial_port, int baudrate);

        unsigned long int stringToDecimal(string input);

        void mainLoop(void);

        struct termios _pgv_tty;

        int _pgv_serial_port;

        string _pgv_serial_port_name;

        int _pgv_baudrate = B19200;

        string _pgv_serial_port_group;

        string _pgv_id_vendor, _pgv_id_vendor_id, _pgv_id_model_id, _pgv_, _pgv_id_path;

        int _timeout;

        char _pgv_serial_buffer[21];

        int _pgv_readed_byte = 0;

        string::const_iterator _pgv_regex_iterator_start, _pgv_regex_iterator_end;

        string _pgv_readed_data = "";

        /**
         * @brief Straight ahead parameter 
        */
        unsigned char _straight_direction[ 2 ] = {0xEC, 0x13}; 

        /**
         * @brief Follow left parameter 
        */
        unsigned char _left_direction[ 2 ] = { 0xE8, 0x17}; 

        /**
         * @brief Follow right parameter 
        */
        unsigned char _right_direction[ 2 ] = { 0xE4, 0x1B}; 

        /**
         * @brief No lane parameter
        */
        unsigned char _no_lane_direction[ 2 ] = { 0xE0, 0x1F}; 

        /**
         * @brief Position request parameter
        */
        unsigned char _position_request[ 2 ] = { 0xC8, 0x37}; // Position Request

        /**
         * @brief Angle of robot 
        */
        double _robot_angle = 0.0;

        /**
         * @brief Position x of robot
        */
        double _robot_position_x = 0.0;

        /**
         * Position y of robot;
        */
        double _robot_position_y = 0.0;

        /**
         * @brief Selected direction 
        */
        string _selected_direction;

        /**
         * @brief 
        */
        int _robot_c_lane_count = 0;

        /**
         * @brief
        */
        int _robot_no_color_lane = 1;

        /**
         * @brief
        */
        int _robot_no_position = 1;

        /**
         * @brief
        */
        int _tag_detected = 0;

        /**
         * @brief
        */
        double _position_error_x = 0.0;
        
        /**
         * @brief
        */
        double _position_error_y = 0.0;

        /**
         * @brief
        */
        double _orientation_error_angle = 0.0;

        double _robot_x_position_decimal, _robot_y_position_decimal, _robot_angle_decimal = 0.0;

        double _calibration_error_x, _calibration_error_y, _calibration_error_angle = 0.0;

        string selected_command;

        int _robot_colour_lane_count_decimal = 0;

        int _robot_no_colour_lane_decimal = 1;

        int _robot_no_pos_decimal = 1;

        int _tag_detected_decimal = 0;
};

#endif