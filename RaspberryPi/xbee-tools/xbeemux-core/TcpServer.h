#ifndef XBEEMUXCORE_TCPSERVER_H
#define XBEEMUXCORE_TCPSERVER_H

class TcpServer
{
	public:
		explicit TcpServer(unsigned int port);
		~TcpServer();

		int accept();
		int fd() const { return m_sk; }

	private:
		int m_sk;
};

#endif // XBEEMUXCORE_TCPSERVER_H
