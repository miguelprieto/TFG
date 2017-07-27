#include "testframework.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define BOLD "\e[1m"
#define BOLD_RED "\e[1;31m"
#define BOLD_GREEN "\e[1;32m"
#define NO_BOLD "\e[0m"

TestRegistration *TestRegistration::test_list_head = nullptr;

SandboxedExecutor::SandboxedExecutor(const char *test_name, const char *data_description)
{
	if (data_description == nullptr)
		jobDescription = strdup(test_name);
	else
		asprintf(&jobDescription, "%s/%s", test_name, data_description);
}

SandboxedExecutor::~SandboxedExecutor()
{
	free(jobDescription);
}

bool SandboxedExecutor::runInSandbox() const
{
	fprintf(stderr, BOLD "Esecuzione del test %s..." NO_BOLD "\n", jobDescription);

	const char *status;
	bool success = false;
	pid_t pid = fork();

	if (pid == -1)
	{
		status = BOLD_RED "fork() error";
	}
	else if (pid == 0)
	{
		sandboxedProcedure();
		_exit(EXIT_SUCCESS);
	}
	else
	{
		int exitstatus;
		waitpid(pid, &exitstatus, 0);

		if (WIFEXITED(exitstatus) && WEXITSTATUS(exitstatus) == EXIT_SUCCESS)
			status = BOLD_GREEN "SUCCESS", success = true;
		else if (WIFSIGNALED(exitstatus))
			status = BOLD_RED "FAILED (signal received)";
		else
			status = BOLD_RED "FAILED";
	}

	fprintf(stderr, "Risultato del test %s : %s" NO_BOLD "\n", jobDescription, status);
	return success;
}

TestRegistration::TestRegistration(const char *test_name, void (*test_procedure)())
: next_test(nullptr), test_name(test_name), test_procedure(test_procedure)
{
	TestRegistration **l = &test_list_head;
	while (*l != nullptr)
		l = &((*l)->next_test);
	*l = this;
}

TestRegistration::TestRegistration(const char *test_name)
: TestRegistration(test_name, nullptr)
{
}

bool TestRegistration::run() const
{
	return SandboxedExecutor::runInSandbox(
		[&]()
		{
			test_procedure();
		}, test_name);
}

bool TestRegistration::runAllRegisteredTests()
{
	bool some_failed = false;
	for (TestRegistration *t = TestRegistration::test_list_head;
		t != nullptr; t = t->next_test)
	{
		if (!t->run())
			some_failed = true;
	}

	return !some_failed;
}

int main(int argc, const char **argv)
{
	fprintf(stderr, "== Esecuzione di %s ==\n", program_invocation_short_name);
	return TestRegistration::runAllRegisteredTests() ? EXIT_SUCCESS : EXIT_FAILURE;
}
