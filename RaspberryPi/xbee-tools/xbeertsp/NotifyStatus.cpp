#include "NotifyStatus.h"

const std::string nsbegin = "\r"; // Torna a inizio riga
const std::string nsend = "\e[K"; // Cancella da cursore in poi
const std::string nsendl = nsend + "\n";

communication_error::communication_error(const std::string &message)
: std::runtime_error(message)
{
}

communication_error::communication_error(const char *message)
: std::runtime_error(message)
{
}
