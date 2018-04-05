/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
//
// Created by Valentin Barral on 16/4/15.
//


#include "PozyxRangingReader.h"


#define INST_REPORT_LEN   (20)
#define INST_VERSION_LEN  (16)
#define INST_CONFIG_LEN   (1)


PozyxRangingReader::PozyxRangingReader() {
    _header_loaded = false;
}

PozyxRangingReader::~PozyxRangingReader() {
    if (_serialUWB) {
        _serialUWB->close();
    }
}

int PozyxRangingReader::openSerialPort(std::string name, int portType) {
    int error = -1;
    boost::system::error_code errorCode;

    if (portType == PORT_TYPE_UWB) {
        if (!_serialUWB) {
            error = 0;
            try {

                _serialUWB = serial_port_ptr(new boost::asio::serial_port(io_service_));
                _serialUWB->open(name, errorCode);
                if (errorCode.value() != 0) {
                    std::cout << "Error code opening port: " << errorCode.message() << "\n";
                    return (-1);
                }

                _serialUWB->set_option(boost::asio::serial_port_base::baud_rate(115200));
                _serialUWB->set_option(boost::asio::serial_port_base::character_size(8));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                _serialUWB->set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

                //std::string decaIni = "deca$";
                //size_t written = writeData(decaIni.c_str(), decaIni.length(), PORT_TYPE_UWB);

            } catch (boost::system::system_error boostError) {
                error = -1;
            }
        }
    }

    return error;
}


// int PozyxRangingReader::writeData(const char *data, int size, int portType) {

//     boost::system::error_code ec;
//     if (size == 0) return 0;
//     if (portType == PORT_TYPE_UWB) {
//         if (_serialUWB) {
//             size_t written = _serialUWB->write_some(boost::asio::buffer(data, size), ec);
//             if (ec.value() == 0) {
//                 return written;
//             } else {
//                 std::cout << "Write error: " << ec.message() << "\n";
//                 return -1;
//             }

//         } else {
//             return -1;
//         }
//     }
//     return -1;
// }


// void PozyxRangingReader::newHeaderData(const std::string &data) {

//     int length = data.length();
//     int offset = 0;
//     if (length >= INST_REPORT_LEN) {
//         while (length >= INST_REPORT_LEN) {
//             std::string header = data.substr(offset, 2);
//             std::string headerValue("nV");
//             if (header.compare(0, headerValue.length(), headerValue) == 0) {
//                 _header_loaded = true;
//                 break;
//             }

//             offset += 2;
//             length -= 2;
//         }

//         if (length < INST_REPORT_LEN) {
//             return;
//         }
//     }
// }


void PozyxRangingReader::async_read_some_() {

    //ROS_INFO("async_read_some");

    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_INFO("DEBUG:SeriaUSB ==NULL || !is_open()");
        return;
    }

    _serialUWB->async_read_some(
        boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(
            &PozyxRangingReader::on_receive_,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void PozyxRangingReader::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred) {

    bool hasError = false;

    //Deberia ser un mensaje de ranging de UWB
    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) {
        ROS_INFO("DEBUG:SeriaUSB ==NULL || !is_open()");
        hasError = true;
    }

    if (ec) {
        ROS_INFO("DEBUG:Error Code %d, %s ", (int) ec.value(), ec.message().c_str());
        hasError = true;
    }


    if (!hasError) {
        for (unsigned int i = 0; i < bytes_transferred; ++i) {
            char c = read_buf_raw_[i];
            read_buf_str_ += c;
        }

        if (read_buf_str_.length() >= TOF_REPORT_LEN) {
            bool somethingToProcess = true;
            while (somethingToProcess) {
                //std::string all = read_buf_str_.substr(0);
                //ROS_INFO("DEBUG:Procesando Length: %d ***[ %s ]***", (int) read_buf_str_.length(), all.c_str());
                somethingToProcess = false;
                int maPos = 0;
                bool maFound = false;
                int bufLength = read_buf_str_.length();

                while (maPos < bufLength - 2) {

                    std::string header = read_buf_str_.substr(maPos, 2);

                    if (header.compare("ma") == 0) //loop here until we reach header ("ma")
                    {
                        maFound = true;
                        break;
                    }

                    maPos += 1;
                }

                bool enoughSize = (maPos + TOF_REPORT_LEN) <= bufLength;

                if (enoughSize && maFound) {
                    //Comprobamos si acaba con el salto de linea
                    std::string lastTwo = read_buf_str_.substr(maPos + TOF_REPORT_LEN - 2, 2);
                    bool correctLastTwo = (lastTwo.compare("\r\n") == 0);


                    if (maFound && enoughSize && correctLastTwo) {
                        std::string newReport = read_buf_str_.substr(maPos, TOF_REPORT_LEN);
                        //ROS_INFO("DEBUG:LINEA maPos: %d {{{[ %s ]}}}", maPos, newReport.c_str());
                        newData(newReport);
                        read_buf_str_.erase(0, maPos + TOF_REPORT_LEN);
                        if (read_buf_str_.length() >= TOF_REPORT_LEN) {
                            somethingToProcess = true;
                            //ROS_INFO("DEBUG: somethingToProcess = true");
                        }
                    } else if (maFound && enoughSize && !correctLastTwo) {
                        //Hay un mensaje cortado, borramos el ma encontrado para que se siga procesando el
                        //resto del buffer si es que hay algo
                        std::string newReport = read_buf_str_.substr(maPos, TOF_REPORT_LEN);
                        ROS_INFO("DEBUG:ERROR TRUNCADO maPos: %d {{{[ %s ]}}}", maPos, newReport.c_str());
                        read_buf_str_.erase(0, maPos + 2);
                        if (read_buf_str_.length() >= TOF_REPORT_LEN) {
                            somethingToProcess = true;
                            //ROS_INFO("DEBUG: somethingToProcess = true");
                        }
                    }

                }


            }

        }

        async_read_some_();
    }





}




void PozyxRangingReader::start(std::string usbPort, ros::Publisher aPub) {

    ROS_INFO("Iniciando UWB receiver");
    uwb_port_name = usbPort;
    ros_pub = aPub;

    //Abrimos el puerto por donde recibiremos las medidas de UWB
    ROS_INFO("Intentando abrir puerto %s", uwb_port_name.c_str());
    bool uwbPortOpen = (openSerialPort(uwb_port_name, PORT_TYPE_UWB) == 0);

    if (uwbPortOpen) {
        ROS_INFO("Puerto abierto");

        //Empezamos a leer asincronamente desde el puerto UWB y cuando recibamos datos los parseamos,
        async_read_some_();
        boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

    } else {
        ROS_INFO("No se pudo abrir el puerto");
    }

}


void PozyxRangingReader::newData(const std::string &data) {

    std::string nowstr = "T:hhmmsszzz:";
    int length = data.length();
    int idx = 0, lastSeq = 0, count = 0;
    bool trilaterate = false;
    bool newposition = false;
    int offset = 0;
    int nolocation = 0;
    std::string statusMsg;


    while (length >= TOF_REPORT_LEN) //we have received a range report from an anchor or a tag
    {
        while (length >= TOF_REPORT_LEN) {

            std::string header = data.substr(offset, 2);

            if (header.compare("ma") == 0) //loop here until we reach header ("ma")
            {
                break;
            }

            offset += 2;
            length -= 2;
        }

        if (length < TOF_REPORT_LEN) {
            return;
        }


        std::string tofReport = data.substr(offset, TOF_REPORT_LEN);


        //POZYX version
        // ido - identificador nodo origen
        // ot - tipo nodo origen: 0: tag, 1: anchor
        // idd - identificador nodo destino
        // dt - tipo nodo destino: 0: tag, 1:anchor
        // r - ranging (mm)
        // ts - timestamp (ms)
        // tx - RSS (dBm)
        // sq - seq number
        // ch - UWB channel: 1, 2, 3, 4, 5, 7
        // dr - Data rate: 0: 110kbits/s, 1: 850kbits/s, 2: 6.8Mbits/s.
        // pr - prf: 1: 16MHz, 2: 64MHz
        // pl - preamble length 0x0C : 4096 symbols., 0x28 : 2048 symbols., 0x18 : 1536 symbols., 0x08 : 1024 symbols., 0x34 : 512 symbols., 0x24 : 256 symbols., 0x14 : 128 symbols., 0x04 : 64 symbols.
        // g - transmision gain 0dB - 33.5 db, steps 0.5
        //ma ido6044 ot0 idd6040 dt1 r000005d4 ts000005d4 tx00AA sq01 ch00 dr00 pr00 pl00 g00


        int originId, originType, destinationId, destinationType, range, rangetime, seq, rss;
        int channel, prf, datarate, preambleLength, gain;

        //                                 ma ido0 ot0 idd3 dt1 r000017E8 ts0000A497 txFFA5 sqE9 ch05 dr00 pr02 pl08 g16
        int n = sscanf(tofReport.c_str(), "ma ido%x ot%x idd%x dt%x r%x ts%x tx%x sq%x ch%x dr%x pr%x pl%x g%x", &originId, &originType, &destinationId, &destinationType, &range, &rangetime, &rss, &seq, &channel, &datarate, &prf, &preambleLength, &gain);

        double channelValue, datarateValue;
        int prfValue, preambleLengthValue, pacSizeValue;

        if (channel == 1) {
            channelValue = 3494.4;
        } else if (channel == 2) {
            channelValue = 3993.6;
        } else if (channel == 3) {
            channelValue = 4492.8;
        } else if (channel == 4) {
            channelValue = 3993.6;
        } else if (channel == 5) {
            channelValue = 6489.6;
        } else if (channel == 7) {
            channelValue = 6489.6;
        } else {
            channelValue = -1.0;
        }

        if (prf == DWT_PRF_16M) {
            prfValue = 16;
        } else if (prf == DWT_PRF_64M) {
            prfValue = 64;
        } else {
            prfValue = -1;
        }

        if (datarate == DWT_BR_110K) {
            datarateValue = 0.11;
        } else if (datarate == DWT_BR_6M8) {
            datarateValue = 6.8;
        } else if (datarate == DWT_BR_850K) {
            datarateValue = 0.85;
        } else {
            datarateValue = -1.0;
        }

        switch (preambleLength) {
        case DWT_PLEN_4096:
            preambleLengthValue = 4096;
            break;
        case DWT_PLEN_2048:
            preambleLengthValue = 2048;
            break;
        case DWT_PLEN_1536:
            preambleLengthValue = 1536;
            break;
        case DWT_PLEN_1024:
            preambleLengthValue = 1024;
            break;
        case DWT_PLEN_512:
            preambleLengthValue = 512;
            break;
        case DWT_PLEN_256:
            preambleLengthValue = 256;
            break;
        case DWT_PLEN_128:
            preambleLengthValue = 128;
            break;
        case DWT_PLEN_64:
            preambleLengthValue = 64;
            break;
        default:
            preambleLengthValue = -1;
            break;
        }


        offset += TOF_REPORT_LEN;
        length -= TOF_REPORT_LEN;

        gtec_msgs::PozyxRanging ranging_msg;

        ranging_msg.originId = originId;
        ranging_msg.originType = originType;
        ranging_msg.destinationId = destinationId;
        ranging_msg.destinationType = destinationType;
        ranging_msg.range = range;
        ranging_msg.seq = seq;
        ranging_msg.ts = rangetime;
        ranging_msg.rxPower = rss;
        ranging_msg.channel = channelValue;
        ranging_msg.prf = prfValue;
        ranging_msg.datarate = datarateValue;
        ranging_msg.preambleLength = preambleLengthValue;
        ranging_msg.txGain = gain;

        if (seq > 255) {
            //ERROR
            //ROS_INFO("ERROR: MENSAJE TRUNCADO RR:[OriginId:%x, Destination:%x, Range:%d, SEQ:%d]", originId, destinationId, range, seq);
            ROS_INFO("DEBUG:LINEA  {{{[ %s ]}}}", data.c_str());
        } else {
            //Publish array
            ros_pub.publish(ranging_msg);
        }
    }
}
