#include "XBeeRtspClientHandler.h"
#include "../xbeemux-core/Utils.h"

#include "DeviceInfo.h"
#include "NotifyStatus.h"
#include "XBeeRtspJobs.h"
#include "XBeeRtspRequestParser.h"

#include <cstring>
#include <unistd.h>

XBeeRtspClientHandler::XBeeRtspClientHandler(XBeeRtspService *owner, int sk)
: m_owner(owner), m_sk(sk), m_status(SIGNATURE_R), m_requestParser(NULL),
  m_currentJob(NULL)
{
	setTcpKeepAlive(sk, 10, 5, 3);
}

XBeeRtspClientHandler::~XBeeRtspClientHandler()
{
	if (m_currentJob)
	{
		m_currentJob->stopAndWait();
		delete m_currentJob;
	}

	delete m_requestParser;
	close(m_sk);
}

XBeeRtspService *XBeeRtspClientHandler::getOwner() const
{
	return m_owner;
}

bool XBeeRtspClientHandler::receiveByte(uint8_t data)
{
	switch (m_status)
	{
		case SIGNATURE_R:
			if (data == 'R')
				m_status = SIGNATURE_T;
			else
				return false;
			break;
		case SIGNATURE_T:
			if (data == 'T')
				m_status = SIGNATURE_S;
			else
				return false;
			break;
		case SIGNATURE_S:
			if (data == 'S')
				m_status = SIGNATURE_P;
			else
				return false;
			break;
		case SIGNATURE_P:
			if (data == 'P')
				m_status = PROTOCOL_VERSION, write(m_sk, "RTSP", 4);
			else
				return false;
			break;
		case PROTOCOL_VERSION:
			if (data == 16)
				m_status = NODE_ID;
			else
				return false;
			break;
		case NODE_ID:
			if (data < 16)
				m_status = DEVICE_NAME, m_nodeId = data;
			else
				return false;
			break;
		case DEVICE_NAME:
			if (data == '\0')
			{
				m_deviceInfo = getDeviceInfo(m_deviceName.c_str());
				if (m_deviceInfo == NULL)
					return false;
				m_status = REQUEST_TYPE;
			}
			else if (m_deviceName.size() < 128)
				m_deviceName.push_back(data);
			else
				return false;
			break;
		case REQUEST_TYPE:
			switch (data >> 4)
			{
				case 1:
				{
					const bool verifyAfterwards = (data & 2) != 0;
					const bool resetAfterwards = (data & 1) != 0;
					m_requestParser = new XBeeRtspProgramRequestParser(this, m_nodeId, m_deviceInfo, verifyAfterwards, resetAfterwards);
					break;
				}
				case 2:
				{
					const bool resetAfterwards = (data & 1) != 0;
					m_requestParser = new XBeeRtspVerifyRequestParser(this, m_nodeId, m_deviceInfo, resetAfterwards);
					break;
				}
				case 3:
				{
					m_requestParser = new XBeeRtspResetRequestParser(this, m_nodeId, m_deviceInfo);
					break;
				}
				default:
				{
					return false;
				}
			}
			m_status = REQUEST_PARSING;
			break;
		case REQUEST_PARSING:
			if (!m_requestParser->receiveByte(data))
				return false;
			break;
		case JOB_RUNNING: // non ci aspettiamo ulteriore input durante esecuzione
		case JOB_DONE: // ci aspettiamo di ricevere l'ACK finale
			return false;
	}

	if (m_status == REQUEST_PARSING)
	{
		m_currentJob = m_requestParser->testDoneAndCreateJob();
		if (m_currentJob)
		{
			delete m_requestParser;
			m_requestParser = NULL;

			std::string text = nsbegin + "In attesa del turno sul server XBeeRtsp..." + nsend;
			write(m_sk, text.c_str(), text.length());

			m_status = JOB_RUNNING;
			m_currentJob->start();
		}
	}

	return true;
}

void XBeeRtspClientHandler::jobCompleted(XBeeRtspJob *job)
{
	char c = job->getSuccessFlag() ? '\x81' : '\x80';
	write(m_sk, &c, 1);

	delete m_currentJob;
	m_currentJob = NULL;

	m_status = JOB_DONE;
}
