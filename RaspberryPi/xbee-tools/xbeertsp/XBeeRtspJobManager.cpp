#include "XBeeRtspJobManager.h"
#include "XBeeRtspClientHandler.h"
#include "XBeeRtspService.h"

#include <cstdlib>
#include <cstring>
#include <err.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <unistd.h>

XBeeRtspJobManager::XBeeRtspJobManager()
: m_runningJob(NULL)
{
	m_eventfd = eventfd(0, 0);
	if (m_eventfd == -1)
		err(EXIT_FAILURE, "eventfd()");
}

XBeeRtspJobManager::~XBeeRtspJobManager()
{
	close(m_eventfd);
}

void XBeeRtspJobManager::handleJobCompleted()
{
	XBeeRtspJob *job = m_runningJob;

	// Consuma notifica m_eventfd
	stopAndWait(job);

	// Invia notifica a client handler
	job->m_client->jobCompleted(job);
}

void XBeeRtspJobManager::addJob(XBeeRtspJob *job)
{
	m_readyJobs.push_back(job);

	if (m_runningJob == NULL)
		startNextJob();
}

void XBeeRtspJobManager::stopAndWait(XBeeRtspJob *job)
{
	if (m_runningJob == job)
	{
		job->cancelled = true;

		eventfd_t unused;
		if (eventfd_read(m_eventfd, &unused) == -1) // attesa completamento
			err(EXIT_FAILURE, "eventfd_read()");
		m_runningJob = NULL;

		// Avvia job successivo
		if (!m_readyJobs.empty())
			startNextJob();
	}
	else if (m_completedJobs.find(job) != m_completedJobs.end())
	{
		// Se il job è già stato completato ci limitiamo a rimuoverlo
		// dalla lista dei job completati
		m_completedJobs.erase(job);
	}
	else // Il job non era ancora stato avviato, rimuoviamolo dalla FIFO
	{
		for (std::vector<XBeeRtspJob*>::iterator it = m_readyJobs.begin();
			it != m_readyJobs.end(); ++it)
		{
			if (*it == job)
			{
				m_readyJobs.erase(it);
				return;
			}
		}

		// Se siamo arrivati qui vuol dire che il job non era nella
		// FIFO, e quindi c'è un problema da qualche parte
		abort();
	}
}

void XBeeRtspJobManager::startNextJob()
{
	pthread_t dummy;
	pthread_attr_t attr;

	m_runningJob = m_readyJobs[0];
	m_readyJobs.erase(m_readyJobs.begin());

	if (pthread_attr_init(&attr) != 0)
		err(EXIT_FAILURE, "pthread_attr_init() failed");
	if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0)
		err(EXIT_FAILURE, "pthread_attr_setdetachstate() failed");

	if (pthread_create(&dummy, &attr, jobRunner, (void*)m_runningJob) != 0)
		err(EXIT_FAILURE, "pthread_create() failed");

	pthread_attr_destroy(&attr);
}

void *XBeeRtspJobManager::jobRunner(void *job_)
{
	XBeeRtspJob *job = (XBeeRtspJob*)job_;
	XBeeRtspJobManager *manager = job->m_jobManager;

	job->run();

	// Segnala completamento al thread principale
	if (eventfd_write(manager->m_eventfd, 1) == -1)
		err(EXIT_FAILURE, "eventfd_write()");
}

XBeeRtspJob::XBeeRtspJob(XBeeRtspClientHandler *client)
: m_client(client), m_jobManager(client->getOwner()->getJobManager()),
  m_filebuf(dup(client->fd()), std::ios::out, 0), m_success(false),
  userout(&m_filebuf), xbee(m_xbeeProxy), cancelled(false)
{
	m_xbeeProxy.m_outgoingMessages = client->getOwner()->getXBee();
	m_xbeeProxy.m_incomingMessages = &m_incomingMessages;
}

XBeeRtspJob::~XBeeRtspJob()
{
}

void XBeeRtspJob::start()
{
	m_jobManager->addJob(this);
}

void XBeeRtspJob::stopAndWait()
{
	m_jobManager->stopAndWait(this);
}

void XBeeRtspJob::XBeeProxy::sendto(const void *buffer, size_t length, const XBeeAddress &dest)
{
	m_outgoingMessages->sendto(buffer, length, dest);
}

ssize_t XBeeRtspJob::XBeeProxy::recvfrom(void *buffer, size_t maxlength, XBeeAddress *out_srcaddr)
{
	XBeeAddress temp1;
	std::vector<uint8_t> temp2;
	m_incomingMessages->dequeueMessage(&temp1, &temp2);

	if (out_srcaddr) *out_srcaddr = temp1;
	memcpy(buffer, &temp2[0], std::min(temp2.size(), maxlength));

	return temp2.size();
}
