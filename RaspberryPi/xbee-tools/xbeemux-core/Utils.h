#ifndef XBEEMUXCORE_UTILS_H
#define XBEEMUXCORE_UTILS_H

void setNonBlockingFlag(int fd, bool newValue = true);
void setTcpNoDelayFlag(int sockfd, bool newValue = true);
void setTcpKeepAlive(int sockfd, int maxProbes, int secondsBeforeFirstProbe, int secondsBetweenProbes);
int connectTcp(const char *host, int port);

#endif // XBEEMUXCORE_UTILS_H
