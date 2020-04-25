#include <cstdlib>
#include <iostream>
#include <thread>
#include <utility>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

const unsigned short port = 8001;
const size_t frameSize = 552960; //1280 x (720/2) cv::Mat

void session(tcp::socket sock) {

    uint8_t* frame = new uint8_t[frameSize];
    auto buff = boost::asio::buffer(frame, frameSize);

    uint16_t* lanePositions = new uint16_t[4];
    auto returnBuff = boost::asio::buffer(lanePositions, 4);

    try {
        while(true) {

            auto t1 = std::chrono::high_resolution_clock::now();

            size_t length = boost::asio::read(sock, buff, boost::asio::transfer_exactly(frameSize));

            for (int i = 0; i < 4; i++) lanePositions[i] = rand() % 15;
            boost::asio::write(sock, returnBuff);

            auto t2 = std::chrono::high_resolution_clock::now();
            auto t1_ = std::chrono::time_point_cast<std::chrono::microseconds>(t1).time_since_epoch().count();
            auto t2_ = std::chrono::time_point_cast<std::chrono::microseconds>(t2).time_since_epoch().count();
            double ms = (t2_ - t1_) * 0.001;
            std::cout << ms << " ms\n";
        }

    } catch (std::exception & e) {
        std::cerr << "Exception in thread: " << e.what() << "\n";
    }
}

void server(boost::asio::io_context& io_context, unsigned short port) {
    tcp::acceptor a(io_context, tcp::endpoint(tcp::v4(), port));
    while(true) {
        std::thread(session, a.accept()).detach();
    }
}

int main() {

    boost::asio::io_context io_context;
    server(io_context, port);

    return 0;
}