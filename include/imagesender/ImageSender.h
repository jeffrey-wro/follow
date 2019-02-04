#ifndef IMAGE_SENDER_H
#define IMAGE_SENDER_H

#include <netinet/in.h>

#include "opencv2/core/mat.hpp"

#define SERVERPORT 4567

class ImageSender
{
public:
	int init();
	void send( cv::Mat* sendBuff );
	void closeClient();
	~ImageSender();
private:
	int listenfd = 0;
	int connfd = 0;
	struct sockaddr_in serv_addr;

};

#endif
