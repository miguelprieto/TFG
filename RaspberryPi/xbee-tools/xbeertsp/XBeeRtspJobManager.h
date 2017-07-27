#ifndef XBEERTSP_JOBMANAGER_H
#define XBEERTSP_JOBMANAGER_H

#include "../xbeemux-core/MessageQueue.h"
#include "../xbeemux-core/XBee.h"

#include <fstream>
#include <vector>
#include <set>
#include <ext/stdio_filebuf.h>
#include <stdint.h>

class XBeeRtspClientHandler;
class XBeeRtspJob;

// Coda FIFO di job da eseguire
class XBeeRtspJobManager
{
	friend class XBeeRtspJob;
	friend class XBeeRtspService;

	public:
		XBeeRtspJobManager();
		~XBeeRtspJobManager();

		int fd() const { return m_eventfd; }
		void handleJobCompleted();

	private:
		void addJob(XBeeRtspJob *job);
		void stopAndWait(XBeeRtspJob *job);

		void startNextJob();
		static void *jobRunner(void *job);

		std::vector<XBeeRtspJob*> m_readyJobs;
		XBeeRtspJob *m_runningJob;
		std::set<XBeeRtspJob*> m_completedJobs;

		// Utilizzato per segnalare al thread principale il completamento
		// del job in esecuzione
		int m_eventfd;
};

class XBeeRtspJob
{
	friend class XBeeRtspJobManager;
	friend class XBeeRtspService;

	public:
		XBeeRtspJob(XBeeRtspClientHandler *client);

		// NOTA: eseguire sempre stopAndWait() prima di distruggere
		virtual ~XBeeRtspJob();

		// registra job nella FIFO
		void start();

		// Richiamato dal ClientHandler per annullare e attendere il
		// completamento del thread (non fa nulla se il job non era
		// ancora stato avviato)
		void stopAndWait();

		bool getSuccessFlag() const { return m_success; }

	protected:
		// Eseguito su thread dedicato.
		// Al termine viene richiamato owner->jobCompleted(this) sul
		// thread principale, a meno che sia già stato richiamato
		// stopAndWait()
		virtual void run() = 0;

		void setSuccessFlag(bool v = true) { m_success = v; }

	private:
		XBeeRtspClientHandler *m_client;
		XBeeRtspJobManager *m_jobManager;
		__gnu_cxx::stdio_filebuf<char> m_filebuf;
		bool m_success;

		MessageQueue m_incomingMessages;

		struct XBeeProxy : public XBee
		{
			void sendto(const void *buffer, size_t length, const XBeeAddress &dstaddr);
			ssize_t recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr = NULL);
			int fd() const { return m_incomingMessages->fd(); }
			XBee *m_outgoingMessages;
			MessageQueue *m_incomingMessages;
		} m_xbeeProxy;

	protected:
		// Variabili utilizzabili dalla sottoclasse
		std::ostream userout; // output verso utente
		XBee &xbee;
		volatile bool cancelled; // diventa true se l'utente chiude la connessione
};

#endif // XBEERTSP_JOBMANAGER_H
