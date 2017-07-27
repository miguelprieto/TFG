#ifndef NOTIFYSTATUS_H
#define NOTIFYSTATUS_H

#include <stdexcept>
#include <string>

extern const std::string nsbegin, nsend, nsendl;

class communication_error : public std::runtime_error
{
	public:
		explicit communication_error(const std::string &message);
		explicit communication_error(const char *message);
};

#endif // NOTIFYSTATUS_H
