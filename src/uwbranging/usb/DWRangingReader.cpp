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


#include "DWRangingReader.h"


#define INST_REPORT_LEN   (20)
#define INST_VERSION_LEN  (16)
#define INST_CONFIG_LEN   (1)


DWRangingReader::DWRangingReader() {
    _header_loaded = false;
}

DWRangingReader::~DWRangingReader() {
    if (_serialUWB) {
        _serialUWB->close();
    }
}

int DWRangingReader::openSerialPort(std::string name, int portType) {
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

                std::string decaIni = "deca$";
                size_t written = writeData(decaIni.c_str(), decaIni.length(), PORT_TYPE_UWB);

            } catch (boost::system::system_error boostError) {
                error = -1;
            }
        }
    }

    return error;
}


int DWRangingReader::writeData(const char *data, int size, int portType) {

    boost::system::error_code ec;
    if (size == 0) return 0;
    if (portType == PORT_TYPE_UWB) {
        if (_serialUWB) {
            size_t written = _serialUWB->write_some(boost::asio::buffer(data, size), ec);
            if (ec.value() == 0) {
                return written;
            } else {
                std::cout << "Write error: " << ec.message() << "\n";
                return -1;
            }

        } else {
            return -1;
        }
    }
    return -1;
}


void DWRangingReader::newHeaderData(const std::string &data) {

    int length = data.length();
    int offset = 0;
    if (length >= INST_REPORT_LEN) {
        while (length >= INST_REPORT_LEN) {
            std::string header = data.substr(offset, 2);
            std::string headerValue("nV");
            if (header.compare(0, headerValue.length(), headerValue) == 0) {
                _header_loaded = true;
                break;
            }

            offset += 2;
            length -= 2;
        }

        if (length < INST_REPORT_LEN) {
            return;
        }
    }
}


void DWRangingReader::async_read_some_() {
    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) return;

    _serialUWB->async_read_some(
        boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
        boost::bind(
            &DWRangingReader::on_receive_,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void DWRangingReader::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred) {


    //Deberia ser un mensaje de ranging de UWB
    if (_serialUWB.get() == NULL || !_serialUWB->is_open()) return;

    if (ec) {
        return;
    }

    for (unsigned int i = 0; i < bytes_transferred; ++i) {
        char c = read_buf_raw_[i];
        read_buf_str_ += c;
    }

    if (read_buf_str_.length() >= TOF_REPORT_LEN) {
        bool somethingToProcess = true;
        while (somethingToProcess) {
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

                maPos += 2;
            }

            bool enoughSize = (maPos + TOF_REPORT_LEN) <= bufLength;
            if (maFound & enoughSize) {
                std::string newReport = read_buf_str_.substr(maPos, TOF_REPORT_LEN);
                newData(newReport);
                read_buf_str_.erase(maPos, TOF_REPORT_LEN);
                if (read_buf_str_.length() >= TOF_REPORT_LEN) {
                    somethingToProcess = true;
                }
            }



        }

    }

    async_read_some_();
}




void DWRangingReader::start(std::string usbPort, ros::Publisher aPub) {

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


void DWRangingReader::newData(const std::string &data) {

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

        //e.g. ma00 t00 000005d4 00000520 000008d5 46 00000000 4022 4022 a0
        //GTEC version: ma00 t00 00000259 00000173 001e a7 00009c0b 4041 4041 a0 dmn03fd dsn0024 da1098a da209be da309a0 dmg0178 dpc0079 dfiba5b

        int range_corrected = 0;
        int aid, tid, range, lnum, seq, i;
        int txant, rxant, raw_range, rangetime, bias_correction;
        char c;

        //GTEC quality params
        int maxNoise, stdNoise, firstPathAmp1, firstPathAmp2, firstPathAmp3, maxGrowthCIR, rxPreamCount, firstPath;

        //GTEC config params
        int channel, prf, datarate, preambleLength, pacSize;

        int n = sscanf(tofReport.c_str(), "ma%x t%x %x %x %x %x %x %x %x %x %c%d dmn%x dsn%x da1%x da2%x da3%x dmg%x dpc%x dfi%x ch%x pr%x dr%x pl%x ps%x", &aid, &tid, &range, &raw_range, &bias_correction, &lnum,
                       &seq, &rangetime, &txant, &rxant, &c, &i, &maxNoise, &stdNoise, &firstPathAmp1, &firstPathAmp2, &firstPathAmp3, &maxGrowthCIR, &rxPreamCount, &firstPath, &channel, &prf, &datarate, &preambleLength, &pacSize);

        double channelValue, datarateValue;
        int prfValue, preambleLengthValue, pacSizeValue;

        if (channel == 2) {
            channelValue = 3993.6;
        } else if (channel == 3) {
            channelValue = 4492.8;
        } else if (channel == 5) {
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

        if (pacSize == DWT_PAC8) {
            pacSizeValue = 8;
        } else if (pacSize == DWT_PAC16) {
            pacSizeValue = 16;
        } else if (pacSize == DWT_PAC32) {
            pacSizeValue = 32;
        } else if (pacSize == DWT_PAC64) {
            pacSizeValue = 64;
        } else {
            pacSizeValue = -1;
        }


        aid &= 0x3;

        offset += TOF_REPORT_LEN;
        length -= TOF_REPORT_LEN;

        if (n != TOF_REPORT_ARGS) {
            // QString string1 = QString::fromLocal8Bit(tofReport, TOF_REPORT_LEN);
            continue;
        }

        //std_msgs::Int32MultiArray ranging_msg;

        gtec_msgs::DWRanging ranging_msg;

        ranging_msg.anchorId = aid;
        ranging_msg.tagId = tid;
        ranging_msg.range = range;
        ranging_msg.rawrange = raw_range;
        ranging_msg.seq = seq;
        ranging_msg.maxNoise = maxNoise;
        ranging_msg.stdNoise = stdNoise;
        ranging_msg.firstPathAmp1 = firstPathAmp1;
        ranging_msg.firstPathAmp2 = firstPathAmp2;
        ranging_msg.firstPathAmp3 = firstPathAmp3;
        ranging_msg.maxGrowthCIR = maxGrowthCIR;
        ranging_msg.rxPreamCount = rxPreamCount;
        ranging_msg.firstPath = firstPath;
        ranging_msg.channel = channelValue;
        ranging_msg.prf = prfValue;
        ranging_msg.datarate = datarateValue;
        ranging_msg.preambleLength = preambleLengthValue;
        ranging_msg.pacSize = pacSizeValue;

        //Publish array
        ros_pub.publish(ranging_msg);

        ROS_INFO("RR:[AnchorId:%d, TagId:%d, Range:%d, RawRange:%d, SEQ:%d]", aid, tid, range, raw_range, seq);
    }
}
