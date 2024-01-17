#include <pgv100/pgv-100.hpp>



PGV100::PGV100():
Node("pgv100_node")
{
    _pgv_direction_subscriber = this->create_subscription<pgv100::msg::PGVCommand>("/pgv_dir", 100, bind(&PGV100::pgvDirectionCallback, this, placeholders::_1));

    _pgv_calibration_subscriber = this->create_subscription<std_msgs::msg::String>("/pgv_calibrate", 100, bind(&PGV100::pgvCalibrationCallback, this, placeholders::_1));

    _pgv_scan_publisher = this->create_publisher<pgv100::msg::PGVScan>("/pgv_scan", 1);

    _timer = this->create_wall_timer(1ms, bind(&PGV100::mainLoop, this));

    declare_parameter("pgv.port_name", "/dev/ttyUSB0");
    declare_parameter("pgv.baudrate", 115200);
    declare_parameter("pgv.serial_port_group", "/dev/tty");
    declare_parameter("pgv.id_vendor", "-");
    declare_parameter("pgv.id_vendor_id", "-");
    declare_parameter("pgv.id_model_id", "-");
    declare_parameter("pgv.id_path", "-");
    declare_parameter("pgv.timeout", 1000);

    _pgv_serial_port_name = this->get_parameter("pgv.port_name").as_string();
    _pgv_baudrate = this->get_parameter("pgv.baudrate").as_int();
    _pgv_serial_port_group = this->get_parameter("pgv.serial_port_group").as_string();
    _pgv_id_vendor = this->get_parameter("pgv.id_vendor").as_string();
    _pgv_id_vendor_id = this->get_parameter("pgv.id_vendor_id").as_string();
    _pgv_id_model_id = this->get_parameter("pgv.id_model_id").as_string();
    _pgv_id_path = this->get_parameter("pgv.id_path").as_string();
    _timeout = this->get_parameter("pgv.timeout").as_int();

    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 Port Name: " << _pgv_serial_port_name);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 Baudrate: " << _pgv_baudrate);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 Serial Port Group: " << _pgv_serial_port_group);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 ID Vendor: " << _pgv_id_vendor);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 ID Vendor ID: " << _pgv_id_vendor_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 ID Model ID: " << _pgv_id_model_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 ID Path: " << _pgv_id_path);
    RCLCPP_INFO_STREAM(this->get_logger(), "PGV-100 Timeout: " << _timeout);

    _pgv_baudrate = this->settingBaudrate(_pgv_baudrate);

    _pgv_serial_port = open(_pgv_serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);    

    rclcpp::sleep_for(chrono::milliseconds(_timeout));

    this->prepareTermios(&_pgv_tty, _pgv_serial_port, _pgv_baudrate);
}



PGV100::~PGV100()
{
    
}



void PGV100::pgvDirectionCallback(const pgv100::msg::PGVCommand::SharedPtr message)
{
    _pgv_command_message.command = message->command;
    
    if(_pgv_command_message.command == 0)
    {
        _selected_direction = "No lane is selected";
        write(_pgv_serial_port, _no_lane_direction, sizeof(_no_lane_direction));
    }
    else if(_pgv_command_message.command == 1)
    {
        _selected_direction = "Right lane is selected";
        write(_pgv_serial_port, _right_direction, sizeof(_right_direction));
    }
    else if(_pgv_command_message.command == 2)
    {
        _selected_direction = "Left lane is selected";
        write(_pgv_serial_port, _left_direction, sizeof(_left_direction));
    }
    else if(_pgv_command_message.command == 3)
    {
        _selected_direction = "Straight Ahead";
        write(_pgv_serial_port, _straight_direction, sizeof(_straight_direction));
    }
}



void PGV100::pgvCalibrationCallback(const std_msgs::msg::String::SharedPtr message)
{
    _calibration_message.data = "reset";

    if(_tag_detected_decimal != 0)
    {
        _calibration_error_x = _robot_x_position_decimal;
        _calibration_error_y = _robot_y_position_decimal;
        _calibration_error_angle = _robot_angle_decimal;
    }
}



int PGV100::settingBaudrate(int baudrate)
{
    switch (baudrate)
    {
        case 9600:
            std::cout << "Baudrate of Serial Port 9600: " << std::endl;
            baudrate = B9600;
            break;

        case 19200:
            std::cout << "Baudrate of Serial Port 19200: " << std::endl;
            baudrate = B19200;
            break;

        case 38400:
            std::cout << "Baudrate of Serial Port 38400: " << std::endl;
            baudrate = B38400; 
            break;

        case 57600:
            std::cout << "Baudrate of Serial Port 57600: " << std::endl;
            baudrate = B57600;
            break;

        case 115200:
            std::cout << "Baudrate of Serial Port 115200: " << std::endl;
            baudrate = B115200;
            break;
        case 230400:
            std::cout << "Baudrate of Serial Port 230400: " << std::endl;
            baudrate = B230400;
            break;
        default:
            std::cout << "Baudrate of Serial Port 9600: " << std::endl;
            baudrate = B9600;
            break;
    }
    return baudrate;
}



void PGV100::prepareTermios(termios* tty, int serial_port, int baudrate)
{
    if(tcgetattr(serial_port, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not open PGV-100 serial port!");
        rclcpp::shutdown();
    }

    cfsetospeed(tty, (speed_t)baudrate);
    cfsetispeed(tty, (speed_t)baudrate);

    tty->c_cflag     &=  ~PARENB;            
    tty->c_cflag     &=  ~CSTOPB;
    tty->c_cflag     &=  ~CSIZE;
    tty->c_cflag     |=  CS8;

    tty->c_cflag     &=  ~CRTSCTS;           
    tty->c_cc[VMIN]   =  1;                 
    tty->c_cc[VTIME]  =  5;                  
    tty->c_cflag     |=  CREAD | CLOCAL;     

    cfmakeraw(tty);
    tcflush(serial_port, TCIFLUSH);

    if(tcsetattr(serial_port, TCSANOW, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not configure PGV-100 serial port!");
        rclcpp::shutdown();
    }

    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Serial ports configuration is completed.");
    }
}



unsigned long int PGV100::stringToDecimal(string input)
{
    unsigned long int decimal_value = stoull(input, nullptr, 2);
    return decimal_value;
}



void PGV100::mainLoop(void)
{
    write(_pgv_serial_port, _position_request, 2);
    RCLCPP_INFO_STREAM(this->get_logger(), "First test for PGV100 sensor...");
    memset(&_pgv_serial_buffer, '\0', sizeof(_pgv_serial_buffer));
    
    _pgv_readed_byte = read(_pgv_serial_port, &_pgv_serial_buffer, sizeof(_pgv_serial_buffer));

    // Get Lane-Detection from the byte array [Bytes 1-2]
    bitset<7> lane_detect_second_byte(_pgv_serial_buffer[0]);
    bitset<7> lane_detect_first_byte(_pgv_serial_buffer[1]);

    string robot_lane_detect = lane_detect_second_byte.to_string() + lane_detect_first_byte.to_string();

    string robot_colour_lane_count = robot_lane_detect.substr(8, 2);
    string robot_colour_lane_detect = robot_lane_detect.substr(11, 1);
    string robot_no_pos = robot_lane_detect.substr(5, 1);
    string tag_detected = robot_lane_detect.substr(7, 1);

    _robot_colour_lane_count_decimal = stringToDecimal(robot_colour_lane_count);
    _robot_no_colour_lane_decimal = stringToDecimal(robot_colour_lane_detect);
    _robot_no_pos_decimal = stringToDecimal(robot_no_pos);
    _tag_detected_decimal = stringToDecimal(tag_detected);

    // Get the angle from the byte array [Byte 11-12]
    bitset<7> angle_second_byte(_pgv_serial_buffer[10]);
    bitset<7> angle_first_byte(_pgv_serial_buffer[11]);

    string robot_angle = angle_second_byte.to_string() + angle_first_byte.to_string();
    _robot_angle_decimal = stringToDecimal(robot_angle) / 10.0;

    if(_robot_angle_decimal > 180.0)
    {
        _robot_angle_decimal = _robot_angle_decimal - 360.0;
    }

    // Get the X-Position from the byte array [Bytes 3-4-5-6]
    bitset<3> robot_x_position_fourth_byte(_pgv_serial_buffer[2]);
    bitset<7> robot_x_position_third_byte(_pgv_serial_buffer[3]);
    bitset<7> robot_x_position_second_byte(_pgv_serial_buffer[4]);
    bitset<7> robot_x_position_first_byte(_pgv_serial_buffer[5]);
    
    string robot_x_position = robot_x_position_fourth_byte.to_string() + robot_x_position_third_byte.to_string() + robot_x_position_second_byte.to_string() + robot_x_position_first_byte.to_string();
    _robot_x_position_decimal = stringToDecimal(robot_x_position);
    
    if(_tag_detected_decimal != 0)
    {
        if(_robot_x_position_decimal > 2000.0)
        {
            _robot_x_position_decimal = _robot_x_position_decimal - pow(2, 24) - 1;
        }
    }

    // Get the Y-Poisition from the byte array [Bytes 7-8]
    bitset<7> y_position_second_byte(_pgv_serial_buffer[6]);
    bitset<7> y_position_first_byte(_pgv_serial_buffer[7]);

    string robot_y_position = y_position_second_byte.to_string() + y_position_first_byte.to_string();
    _robot_y_position_decimal = stringToDecimal(robot_y_position);
    
    if(_robot_y_position_decimal > 2000.0)
    {
        _robot_y_position_decimal = _robot_y_position_decimal - 16383.0;
    }

    if(_robot_no_pos_decimal)
    {
        _robot_y_position_decimal = _robot_y_position_decimal * -1;
    }

    _pgv_scan_message.angle = _robot_angle_decimal - _calibration_error_angle; // degree
    _pgv_scan_message.x_position = (_robot_x_position_decimal - _calibration_error_x) / 10.0; // mm
    _pgv_scan_message.y_position = (_robot_y_position_decimal - _calibration_error_y) / 10.0; // mm
    _pgv_scan_message.direction = _selected_direction;
    _pgv_scan_message.color_lane_count = _robot_colour_lane_count_decimal;
    _pgv_scan_message.no_color_lane = _robot_no_colour_lane_decimal;
    _pgv_scan_message.no_position = _robot_no_pos_decimal;
    _pgv_scan_message.tag_detected = _tag_detected_decimal;

    _pgv_scan_publisher->publish(_pgv_scan_message);
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PGV100>());
    return 0;
}