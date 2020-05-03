/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <arpa/inet.h>
#include <linux/tcp.h>


const size_t frameSize = 552960;
const uint16_t port = 8000;

void error(const char *msg) {
	perror(msg);
	exit(1);
}

void readAmount(int socket, size_t count, void* buffer) {

    size_t bytesRead = 0;
	int result;
    while (bytesRead < count) {
        result = read(socket, buffer + bytesRead, count - bytesRead);

        if (result < 1 ) { std::cout << "ERR!\n"; }

        bytesRead += result;
    }
}

bool validate(uint8_t * buffer, size_t length) {
	return (buffer[0] + buffer[length - 1]) == 255;
}

int main(int argc, char *argv[]) {

	int sockfd, newsockfd, portno;
	socklen_t clilen;

	uint8_t * frameBuffer = new uint8_t[frameSize];

	struct sockaddr_in serv_addr, cli_addr;
	int n;

	sockfd = socket(PF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) error("ERROR opening socket");
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);
	if (bind(sockfd, (struct sockaddr *) &serv_addr,
			sizeof(serv_addr)) < 0) 
			error("ERROR on binding");
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if (newsockfd < 0) error("ERROR on accept");
	
	//read indefinately

	while(true) {

		readAmount(newsockfd, frameSize, frameBuffer);
		/*if(!validate(frameBuffer, frameSize)) {
			std::cerr << "ISSUE!\n";
		} else {
			std::cout << static_cast<int>(frameBuffer[0]) << ", " << static_cast<int>(frameBuffer[frameSize - 1]) << "\n";
		}*/

		//detection...
		//usleep(10000);

		n = write(newsockfd,"I got your message",18);
		/*if (n < 0) {
			std::cerr << "ERROR writing to socket\n";
			break;
		}*/
	}

	close(newsockfd);
	close(sockfd);
	return 0; 
	}
