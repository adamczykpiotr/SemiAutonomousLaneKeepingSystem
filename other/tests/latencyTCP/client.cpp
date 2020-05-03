#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <chrono>

const size_t frameSize = 552960;
const uint16_t port = 8000;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[]) {
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");


	uint8_t * frameBuffer = new uint8_t[frameSize];


    while(true) {

        auto t1 = std::chrono::high_resolution_clock::now();

        n = write(sockfd,frameBuffer,frameSize);
        if (n < 0) error("ERROR writing to socket");

        n = read(sockfd,buffer,255);
        if (n < 0) error("ERROR reading from socket");

        auto t2 = std::chrono::high_resolution_clock::now();


        auto t1_ = std::chrono::time_point_cast<std::chrono::microseconds>(t1).time_since_epoch().count();
        auto t2_ = std::chrono::time_point_cast<std::chrono::microseconds>(t2).time_since_epoch().count();
        double ms = (t2_ - t1_) * 0.001;
        std::cout << "Time: " << ms << "\n";
        //samples.push_back(us);


        std::cout << "Reply is " << n << " bytes long\n";
        usleep(500);
    }

    close(sockfd);
    return 0;
}