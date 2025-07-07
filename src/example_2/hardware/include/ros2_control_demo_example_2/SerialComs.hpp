#ifndef WATER_TANK_BASE_SERIAL_COMS_HPP
#define WATER_TANK_BASE_SERIAL_COMS_HPP


#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>
#include <iomanip>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
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
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
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

        int send_heartbeat(int packet_id) {
            // Create 6-byte dataframe: 1 3 1 1 0 1
            uint8_t heartbeat_data[6] = {1, 3, packet_id, 1, 0, 1};
            
            // Pack the data
            uint8_t packed_data[256];  // Buffer for packed data
            uint8_t packed_size = pack_data(packed_data, heartbeat_data, 6);
            
            // Convert packed data to string
            std::string data_str(packed_data, packed_data + packed_size);
            
            // Print the data in hex format
            std::cout << "Sending data (hex): ";
            for(unsigned char c : data_str) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
            }
            std::cout << std::dec << std::endl;  // Reset to decimal output
            
            // Print the data in raw format
            std::cout << "Sending data (raw): " << data_str << std::endl;
            
            serial_com_.FlushIOBuffers();
            serial_com_.Write(data_str);
            return packed_size;
        }

        void write_cmd_motor(int cmd_left, int cmd_right){
            //1.create data stack 

            //2.pack the data into data pack 

            //3.send data pack to serial port 

            //4.receive data from serial port 

            //5.unpack the data from data pack 
            
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
    
        uint8_t pack_data(uint8_t *pack, uint8_t *data, uint16_t len){
            // if(pack == NULL || data == NULL || length == 0){
            //     return 0;
            // }
            uint16_t index = 0;
            uint16_t crc = 0;
            pack[index++] = '{';
            index += 2;
            for (int i = 0; i<len; i++){
                if(data[i] == '{' || data[i] == '}' || data[i] == '-'){
                    pack[index++] = '-';
                    pack[index++] = data[i]+1;
                }
                else{
                    pack[index++] = data[i];
                }
            }
            pack[1] = index>>8;
            pack[2] = index&0xFF;
            crc = crc16_ccitt(pack, index);
            pack[index++] = crc >> 8 ;
            pack[index++] = crc & 0xFF;
            pack[index++] = '}'; // 包尾

            return index; 
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

    private:
        //create serial com 
        std::vector<uint8_t> pack_buffer_; 
        std::vector<uint8_t> data_buffer_; 
        LibSerial::SerialPort serial_com_ ; 
        int time_out_; 
};


#endif 