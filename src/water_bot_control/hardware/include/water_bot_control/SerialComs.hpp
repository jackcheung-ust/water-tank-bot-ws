#ifndef WATER_TANK_BASE_SERIAL_COMS_HPP
#define WATER_TANK_BASE_SERIAL_COMS_HPP


#include <unistd.h>
#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <iomanip>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Handle common baud rates including high-speed rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
#ifdef __linux__
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    case 500000: return LibSerial::BaudRate::BAUD_500000;
    case 576000: return LibSerial::BaudRate::BAUD_576000;
    case 921600: return LibSerial::BaudRate::BAUD_921600;
    case 1000000: return LibSerial::BaudRate::BAUD_1000000;
    case 1152000: return LibSerial::BaudRate::BAUD_1152000;
    case 1500000: return LibSerial::BaudRate::BAUD_1500000;
#endif
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class SerialComs {
    public:
        SerialComs() = default;

        void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms){
            time_out_ = timeout_ms;
            serial_com_.Open(serial_device);
            serial_com_.SetBaudRate(convert_baud_rate(baud_rate));
        }
        
        void disconnect(){
            serial_com_.Close();
        }

        bool is_Opened(){
            return serial_com_.IsOpen();
        }

        

        // void send_cmd_motor(int16_t cmd_left, int16_t cmd_right){
        //     //1.create data stack 
        //     uint8_t l_motor_velocity[2] = {static_cast<uint8_t>(cmd_left >> 8), static_cast<uint8_t>(cmd_left & 0xFF)};
        //     uint8_t r_motor_velocity[2] = {static_cast<uint8_t>(cmd_right >> 8), static_cast<uint8_t>(cmd_right & 0xFF)};
        //     //2.pack the data into data pack 
            
        //     //3.send data pack to serial port 

        //     //4.receive data from serial port 

        //     //5.unpack the data from data pack 
            
        // }

        int write_data(int16_t l_wheel_v, int16_t r_wheel_v){
            if (clean_mode_ == 0){
                //currently in the water mode, just send heartbeat 
                // std::cout<<"Sending empty heartbeat..."<<std::endl;
                send_heartbeat(pack_id);
                ack_send = false;
                // std::cout<<"func done"<< std::endl;
            }
            else if (clean_mode_ == 1 or clean_mode_ == 2){
                //send motor velocity 
                if (ack_send != true){
                    send_auto_ack(pack_id);
                    ack_send = true;
                }else {
                    send_motor_control(pack_id,l_wheel_v,r_wheel_v);
                }
            }

            else if (clean_mode_ == -1){
                // std::cout<<"inside write data left v : "<<l_wheel_v<<" right v : " <<r_wheel_v<<std::endl;
                send_motor_control(pack_id,l_wheel_v,r_wheel_v);
            }

            usleep(time_out_*1000);
            
            try
            {   
                // std::cout<<"waitting for response...."<<std::endl;
                LibSerial::DataBuffer response_buffer;
                
                // Check if data is available and get the number of bytes
                size_t bytes_available = serial_com_.GetNumberOfBytesAvailable();
                // std::cout << "Bytes available: " << bytes_available << std::endl;
                
                if (bytes_available > 0) {
                    // Read the exact number of bytes available
                    serial_com_.Read(response_buffer, bytes_available, time_out_);
                    // std::cout<<"Serial read done. Read " << response_buffer.size() << " bytes." <<std::endl;
                    parsing_data(response_buffer);
                } else {
                    // If no bytes immediately available, try to read with timeout using 0 bytes (blocking read)
                    // std::cout << "No bytes immediately available, trying blocking read..." << std::endl;
                    serial_com_.Read(response_buffer, 0, time_out_);
                    if (response_buffer.size() > 0) {
                        // std::cout<<"Serial read done. Read " << response_buffer.size() << " bytes." <<std::endl;
                        parsing_data(response_buffer);
                    } else {
                        std::cout << "No data received within timeout." << std::endl;
                    }
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            
            pack_id ++ ;
            //parse serial read data 
            return 0;
        }


        std::pair<int, int> read_wheel_vel(){
            return {left_w_speed_feedback_,right_w_speed_feedback_};
        }   
        
    private:
        uint16_t crc16_ccitt(uint8_t *data, uint16_t size) {
            uint16_t crc = 0x0000;
            uint16_t poly = 0x8408;

            for (uint16_t i = 0; i < size; i++) {
                crc ^= data[i];
                for (uint16_t j = 0; j < 8; j++) {
                    if ((crc & 0x0001) != 0 ) {
                        crc = (crc >> 1) ^ poly;
                    } else {
                        crc = crc >> 1;
                    }
                }
            }
            return crc;
        }
    
        uint8_t pack_data(uint8_t *pack, uint8_t *data, uint16_t len, uint16_t max_pack_size){
            // Input validation - CRITICAL for preventing buffer overflows
            if(pack == nullptr || data == nullptr || len == 0 || max_pack_size < (len * 2 + 6)){
                std::cerr << "ERROR: pack_data invalid parameters or insufficient buffer size" << std::endl;
                return 0;
            }
            
            uint16_t index = 0;
            uint16_t crc = 0;
            
            // Ensure we don't exceed buffer bounds
            if (index >= max_pack_size) return 0;
            pack[index++] = '{';
            
            // Reserve space for length bytes
            if (index + 2 >= max_pack_size) return 0;
            index += 2;
            
            // Pack data with escape sequences, checking bounds at each step
            for (int i = 0; i < len; i++){
                if(data[i] == '{' || data[i] == '}' || data[i] == '-'){
                    // Need 2 bytes for escaped character
                    if (index + 2 >= max_pack_size) {
                        std::cerr << "ERROR: Buffer overflow prevented in pack_data escape sequence" << std::endl;
                        return 0;
                    }
                    pack[index++] = '-';
                    pack[index++] = data[i]+1;
                }
                else{
                    if (index >= max_pack_size) {
                        std::cerr << "ERROR: Buffer overflow prevented in pack_data data copy" << std::endl;
                        return 0;
                    }
                    pack[index++] = data[i];
                }
            }
            
            // Set length bytes
            pack[1] = static_cast<uint8_t>(index>>8);
            pack[2] = static_cast<uint8_t>(index&0xFF);
            
            // Add CRC - need 2 bytes
            if (index + 2 >= max_pack_size) {
                std::cerr << "ERROR: Buffer overflow prevented in pack_data CRC" << std::endl;
                return 0;
            }
            crc = crc16_ccitt(pack, index);
            pack[index++] = static_cast<uint8_t>(crc >> 8);
            pack[index++] = static_cast<uint8_t>(crc & 0xFF);
            
            // Add tail - need 1 byte
            if (index >= max_pack_size) {
                std::cerr << "ERROR: Buffer overflow prevented in pack_data tail" << std::endl;
                return 0;
            }
            pack[index++] = '}';

            return static_cast<uint8_t>(index);
        }

        int16_t unpack_data(uint8_t *data, uint8_t *pack, uint16_t len) {
            if (data == NULL || pack == NULL) return 0;
            uint16_t pack_len = (uint16_t)pack[1] << 8 | pack[2];
            uint16_t index = 0;
            uint16_t crc = 0;

            if(len == 0 || (pack_len + 2) > len) return -1;

            if (pack[0] != '{' || pack[pack_len+2] != '}') return -2;

            crc = crc16_ccitt(pack, pack_len);
            if (crc != ((uint16_t)pack[pack_len] << 8 | pack[pack_len + 1])) return -3;

            for (int i = 3; i < pack_len; i++) {
                if(pack[i] == '-')
                { //将转义后数据还原
                    data[index++] = pack[++i] - 1;
                }
                else
                {
                    data[index++] = pack[i];
                }
            }

            return index;
        }
        
        int send_heartbeat(uint8_t packet_id) {
            // Create 5-byte dataframe for heartbeat
            if (!serial_com_.IsOpen()) {
                std::cerr << "Serial port is not open!" << std::endl;
                return -1;
            }

            //7B 00 08 31 33 01 01 00 FB E3 7D //
            uint8_t heartbeat_data[5] = {49, 51, packet_id, 1, 0};
            
            // FIXED: Calculate proper buffer size for worst-case escape scenario
            // 5 bytes data * 2 (worst case all escaped) + 6 overhead = 16 bytes minimum
            constexpr uint16_t HEARTBEAT_BUFFER_SIZE = 20; // Extra safety margin
            uint8_t packed_data[HEARTBEAT_BUFFER_SIZE];
            
            uint8_t packed_size = pack_data(packed_data, heartbeat_data, 5, HEARTBEAT_BUFFER_SIZE);
            
            if (packed_size == 0) {
                std::cerr << "ERROR: Failed to pack heartbeat data - buffer overflow prevented" << std::endl;
                return -1;
            }
            
            // Convert packed data to string
            std::string data_str(packed_data, packed_data + packed_size);
            
            // Print the data in hex format
            // std::cout << "Sending heartbeat data (hex): ";
            // std::cout << sizeof(packed_data) << " bytes, "<< std::endl;
            // std::cout << "Sending data (hex): ";
            
            // for(unsigned char c : data_str) {
            //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
            // }
            // std::cout << std::dec << std::endl;  // Reset to decimal output
            
            serial_com_.FlushIOBuffers();
            serial_com_.Write(data_str);
            // std::cout<<"sent data..."<<std::endl;
            
            return 0;
        }

        int send_auto_ack(uint8_t packet_id){
            if (!serial_com_.IsOpen()) {
                std::cerr << "Serial port is not open!" << std::endl;
                return -1;
            }

            uint8_t ack_data[6] = {49, 51,packet_id,2,0,1};
            
            // FIXED: Calculate proper buffer size for worst-case escape scenario
            // 6 bytes data * 2 (worst case all escaped) + 6 overhead = 18 bytes minimum
            constexpr uint16_t ACK_BUFFER_SIZE = 24; // Extra safety margin
            uint8_t packed_data[ACK_BUFFER_SIZE];
            
            uint8_t packed_data_size = pack_data(packed_data, ack_data, sizeof(ack_data), ACK_BUFFER_SIZE);

            if (packed_data_size == 0) {
                std::cerr << "ERROR: Failed to pack auto ack data - buffer overflow prevented" << std::endl;
                return -1;
            }

            std::string data_str(packed_data, packed_data + packed_data_size);

            serial_com_.FlushIOBuffers();
            serial_com_.Write(data_str);

            return packed_data_size;
        }

        int send_motor_control(uint8_t packet_id,int16_t l_motor_v, int16_t r_motor_v){
            if (!serial_com_.IsOpen()) {
                std::cerr << "Serial port is not open!" << std::endl;
                return -1;
            }

            uint8_t l_motor_v_upper = static_cast<uint8_t>(l_motor_v >> 8);
            uint8_t l_motor_v_lower = static_cast<uint8_t>(l_motor_v & 0xFF);
            
            uint8_t r_motor_v_upper = static_cast<uint8_t>(r_motor_v >> 8); 
            uint8_t r_motor_v_lower = static_cast<uint8_t>(r_motor_v & 0xFF);

            uint8_t control_data[10] = {49, 51, packet_id,3,0,l_motor_v_upper,l_motor_v_lower,r_motor_v_upper,r_motor_v_lower,0};

            // FIXED: This was the CRITICAL buffer overflow causing stack smashing!
            // Original: 16 bytes buffer (10+6) but could need up to 26 bytes worst case
            // 10 bytes data * 2 (worst case all escaped) + 6 overhead = 26 bytes minimum
            constexpr uint16_t MOTOR_CONTROL_BUFFER_SIZE = 32; // Extra safety margin
            uint8_t packed_data[MOTOR_CONTROL_BUFFER_SIZE];

            uint8_t packed_data_size = pack_data(packed_data, control_data, sizeof(control_data), MOTOR_CONTROL_BUFFER_SIZE);

            if (packed_data_size == 0) {
                std::cerr << "ERROR: Failed to pack motor control data - buffer overflow prevented" << std::endl;
                return -1;
            }

            std::string data_str(packed_data, packed_data + packed_data_size);
            serial_com_.FlushIOBuffers();
            serial_com_.Write(data_str);
            return packed_data_size;
        }

        void parsing_data(LibSerial::DataBuffer &dataBuffer){
            // Print dataBuffer in hex format
            // std::cout << "Received dataBuffer (hex): ";
            // for(uint8_t byte : dataBuffer) {
            //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::dec << std::endl;  // Reset to decimal output
            
            // FIXED: Add proper buffer size validation
            constexpr size_t MAX_UNPACKED_SIZE = 50;
            uint8_t unpacked_data[MAX_UNPACKED_SIZE];
            
            // Validate input buffer size before processing
            if (dataBuffer.size() > 100) { // Reasonable upper limit
                std::cerr << "ERROR: Input data buffer too large: " << dataBuffer.size() << " bytes" << std::endl;
                return;
            }
            
            // Pass the actual dataBuffer size to unpack_data, not a hardcoded 30
            int unpacked_size = unpack_data(unpacked_data, const_cast<uint8_t*>(dataBuffer.data()), 
                                          static_cast<uint16_t>(dataBuffer.size()));
            
            // Validate unpacked data size and check bounds for all array accesses
            if (unpacked_size < 5) {
                std::cerr << "ERROR: Unpacked data too small: " << unpacked_size << " bytes" << std::endl;
                return;
            }
            
            if (unpacked_size < 0) {
                std::cerr << "ERROR: Failed to unpack data, error code: " << unpacked_size << std::endl;
                return;
            }

            // Bounds-checked parsing with proper validation
            if (unpacked_size >= 19 && unpacked_data[3]==2 && unpacked_data[4]==0){
                operate_mode_ = unpacked_data[7];
                clean_mode_ = unpacked_data[8];
                f_safe_d = unpacked_data[9] << 8 | unpacked_data[10];
                b_safe_d = unpacked_data[11] << 8 | unpacked_data[12];
                l_safe_d = unpacked_data[13] << 8 | unpacked_data[14];
                r_safe_d = unpacked_data[15] << 8 | unpacked_data[16];
                clean_width = unpacked_data[17] << 8 | unpacked_data[18];
            }
            
            else if (unpacked_size >= 18 && unpacked_data[3] == 1 && unpacked_data[4]==1){
                operate_mode_ = unpacked_data[7];
                left_w_speed_feedback_ = unpacked_data[8] << 8 | unpacked_data[9];
                right_w_speed_feedback_ = unpacked_data[10] << 8 | unpacked_data[11];
                roll_ = unpacked_data[12] << 8 | unpacked_data[13];
                pitch_ = unpacked_data[14] << 8 | unpacked_data[15];
                yaw_ = unpacked_data[16] << 8 | unpacked_data[17];
            }

            else if (unpacked_size >= 19 && unpacked_data[3]==1 && unpacked_data[4]==2){
                operate_mode_ = unpacked_data[7];
                left_w_speed_feedback_ = unpacked_data[8] << 8 | unpacked_data[9];
                right_w_speed_feedback_ = unpacked_data[10] << 8 | unpacked_data[11];
                roll_ = unpacked_data[12] << 8 | unpacked_data[13];
                pitch_ = unpacked_data[14] << 8 | unpacked_data[15];
                yaw_ = unpacked_data[16] << 8 | unpacked_data[17];
                clean_mode_feedback_ = unpacked_data[18];
            }
        
            else if (unpacked_size >= 12 && unpacked_data[3]==1 && unpacked_data[4]==0){
                operate_mode_ = unpacked_data[7];
                left_w_speed_feedback_ = unpacked_data[8] << 8 | unpacked_data[9];
                right_w_speed_feedback_ = unpacked_data[10] << 8 | unpacked_data[11];
                clean_mode_ = 0;
                // roll_ = unpacked_data[12] << 8 | unpacked_data[13];
                // pitch_ = unpacked_data[14] << 8 | unpacked_data[15];
                // yaw_ = unpacked_data[16] << 8 | unpacked_data[17];
                // clean_mode_feedback_ = unpacked_data[18];
            }
            else {
                std::cerr << "WARNING: Unknown or malformed data packet received (size: " 
                          << unpacked_size << ")" << std::endl;
            }
        }

    private:
        //create serial com 
        // std::vector<uint8_t> pack_buffer_; 
        // std::vector<uint8_t> data_buffer_; 
        LibSerial::SerialPort serial_com_ ; 
        bool ack_send = false;
        int time_out_; 
        int operate_mode_; //0: standby, 1:soft stop, 2:Recovery mode, 3:manual mode, 4:auto mode start, 5:auto mode stop, 6:auto mode pause, 7:auto mode resume
        int clean_mode_ = -1;//0: water clean 1: water-free clean 2: water-free flush -1:function test
        int clean_mode_feedback_; 
        uint8_t pack_id = 1; 
        uint16_t f_safe_d;
        uint16_t b_safe_d;
        uint16_t l_safe_d;
        uint16_t r_safe_d;

        uint16_t clean_width;

        int16_t left_w_speed_feedback_= 0; 
        int16_t right_w_speed_feedback_ = 0; 
        int16_t roll_;
        int16_t pitch_;
        int16_t yaw_;
};


#endif
