#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

const std::string ip = "192.168.1.100";
const std::string port = "8001";

const size_t maxLength = 1024 * 1024 * 5;
const size_t frameSize = 552960; //1280 x (720/2) cv::Mat
const size_t replySize = 4;

const size_t sampleCount = 2000;

int main() {

    std::vector<double> samples;
    samples.reserve(sampleCount);

    try {

        boost::asio::io_context io_context;
        tcp::socket sock(io_context);
        tcp::resolver resolver(io_context);
        boost::asio::connect(sock, resolver.resolve(ip, port));

        uint8_t* request = new uint8_t[maxLength];
        for (size_t i = 0; i < maxLength; i++) request[i] = 'e';
        auto requestBuff = boost::asio::buffer(request, maxLength);

        uint16_t response[4];
        auto responseBuff = boost::asio::buffer(response, 4);


        bool running = true;

        while (running) {

            auto t1 = std::chrono::high_resolution_clock::now();

            boost::asio::write(sock, responseBuff);
            //size_t reply_length = sock.read_some(buff);
            size_t length = boost::asio::read(sock, requestBuff, boost::asio::transfer_exactly(replySize));

            auto t2 = std::chrono::high_resolution_clock::now();


            auto t1_ = std::chrono::time_point_cast<std::chrono::microseconds>(t1).time_since_epoch().count();
            auto t2_ = std::chrono::time_point_cast<std::chrono::microseconds>(t2).time_since_epoch().count();
            double us = (t2_ - t1_) * 0.001;
            samples.push_back(us);

            if (samples.size() == sampleCount) {
                double avg = 0.;
                double stdDev = 0.;

                for (double sample : samples) {
                    avg += sample;
                }

                avg /= sampleCount;

                for (double sample : samples) {
                    double temp = (sample - avg);
                    stdDev += temp * temp;
                }
                stdDev /= sampleCount;
                stdDev = sqrt(stdDev);

                std::cout << "Avg latency: " << avg << " ms, Standard deviation: " << stdDev << "\n";
                running = false;
            }
        }

    } catch (std::exception & e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}