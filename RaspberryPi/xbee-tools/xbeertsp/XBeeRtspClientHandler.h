#ifndef XBEERTSP_CLIENTHANDLER_H
#define XBEERTSP_CLIENTHANDLER_H

#include <string>
#include <stdint.h>

class DeviceInfo;
class XBeeRtspJob;
class XBeeRtspRequestParser;
class XBeeRtspService;

class XBeeRtspClientHandler
{
	public:
		XBeeRtspClientHandler(XBeeRtspService *owner, int sk);
		~XBeeRtspClientHandler();

		XBeeRtspService *getOwner() const;

		bool receiveByte(uint8_t data);
		void jobCompleted(XBeeRtspJob *job);

		int fd() const { return m_sk; }

	private:
		XBeeRtspService *m_owner;
		int m_sk;

		enum
		{
			SIGNATURE_R,
			SIGNATURE_T,
			SIGNATURE_S,
			SIGNATURE_P,
			PROTOCOL_VERSION,
			NODE_ID,
			DEVICE_NAME,
			REQUEST_TYPE,
			REQUEST_PARSING,
			JOB_RUNNING,
			JOB_DONE
		} m_status;

		uint8_t m_nodeId;
		std::string m_deviceName;
		const DeviceInfo *m_deviceInfo;
		XBeeRtspRequestParser *m_requestParser;
		XBeeRtspJob *m_currentJob;
};

#endif // XBEERTSP_CLIENTHANDLER_H
