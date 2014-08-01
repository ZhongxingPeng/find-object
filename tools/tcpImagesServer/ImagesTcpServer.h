/*
 * ImagesTcpServer.h
 *
 *  Created on: 2014-05-21
 *      Author: mathieu
 */

#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_

#include "find_object/Camera.h"
#include <QtNetwork/QTcpServer>

class ImagesTcpServer : public QTcpServer
{
	Q_OBJECT

public:
	ImagesTcpServer(float hz = 10.0f, const QString & path = "", QObject * parent = 0);

	QHostAddress getHostAddress() const;
	quint16 getPort() const;


private Q_SLOTS:
	void addClient();
	void publishImage(const cv::Mat & image);

private:
	Camera camera_;
};

#endif /* TCPCLIENT_H_ */