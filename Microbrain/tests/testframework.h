#ifndef TESTFRAMEWORK_H
#define TESTFRAMEWORK_H

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// ---- INIZIO INTERFACCIA PUBBLICA ----

// Registra un test semplice, che consiste nell'esecuzione di una singola procedura
#define REGISTER_TEST(test_procedure) \
	namespace { static TestRegistration testRegistration_ ## test_procedure(#test_procedure, test_procedure); }

template<typename ...Args>
class TestWithDataRunner
{
	public:
		// Nei test registrati tramite REGISTER_TEST_WITH_DATA, la
		// data_generator_procedure deve richiamare questo metodo per
		// ogni input da testare. Il primo parametro è una stringa
		// che descrive l'input, i successivi parametri vengono
		// inoltrati alla test_procedure.
		virtual void runOnData(const char *data_description, const Args&... data) const = 0;
};

// Registra un test da eseguire più di una volta su input diversi.
// La procedura test_procedure viene eseguita ripetutamente per ciascuno degli
// input generati da data_generator_procedure. Ogni input consiste in un insieme
// di parametri da passare alla test_procedure. Gli argomenti __VA_ARGS__ devono
// elencare il tipo di ciascun parametro atteso dalla test_procedure
#define REGISTER_TEST_WITH_DATA(test_procedure, data_generator_procedure, ...) \
	namespace { static TestWithDataRegistration<__VA_ARGS__> testRegistration_ ## test_procedure(#test_procedure, test_procedure, data_generator_procedure); }

// ---- FINE INTERFACCIA PUBBLICA ----

#ifdef NDEBUG
#error Framework di test non utilizzabile con macro NDEBUG definita
#endif

class SandboxedExecutor
{
	public:
		template <typename T>
		static bool runInSandbox(T lambdaProcedure, const char *test_name, const char *data_description = nullptr)
		{
			struct AuxSandboxedExecutor : public SandboxedExecutor
			{
				AuxSandboxedExecutor(T lambdaProcedure, const char *test_name, const char *data_description)
				: SandboxedExecutor(test_name, data_description), lambdaProcedure(lambdaProcedure)
				{
				}

				void sandboxedProcedure() const
				{
					lambdaProcedure();
				}

				T lambdaProcedure;
			};

			AuxSandboxedExecutor aux(lambdaProcedure, test_name, data_description);
			return aux.runInSandbox();
		}

	private:
		SandboxedExecutor(const char *test_name, const char *data_description);
		virtual ~SandboxedExecutor();
		bool runInSandbox() const;

		virtual void sandboxedProcedure() const = 0;
		char *jobDescription;
};

class TestRegistration
{
	public:
		TestRegistration(const char *test_name, void (*test_procedure)());

		static bool runAllRegisteredTests();

		virtual bool run() const;

	protected:
		TestRegistration(const char *test_name);
		const char *test_name;

	private:
		static TestRegistration *test_list_head;
		TestRegistration *next_test;

		void (*test_procedure)();
};

template<typename ...Args>
class TestWithDataRegistration : public TestRegistration, private TestWithDataRunner<Args...>
{
	public:
		TestWithDataRegistration(const char *test_name, void (*test_procedure)(Args...), void (*data_generator_procedure)(const TestWithDataRunner<Args...> &));
		virtual bool run() const;

	private:
		void runOnData(const char *data_description, const Args&... data) const;

		mutable bool no_data, some_failed;
		void (*test_procedure)(Args...);
		void (*data_generator_procedure)(const TestWithDataRunner<Args...> &);
};

template <typename ...Args>
TestWithDataRegistration<Args...>::TestWithDataRegistration(const char *test_name, void (*test_procedure)(Args...),
	void (*data_generator_procedure)(const TestWithDataRunner<Args...> &))
: TestRegistration(test_name), test_procedure(test_procedure), data_generator_procedure(data_generator_procedure)
{
}

template <typename ...Args>
void TestWithDataRegistration<Args...>::runOnData(const char *data_description, const Args&... data) const
{
	if (SandboxedExecutor::runInSandbox(
		[&]()
		{
			test_procedure(data...);
		}, test_name, data_description) == false)
	{
		some_failed = true;
	}

	no_data = false;
}

template <typename ...Args>
bool TestWithDataRegistration<Args...>::run() const
{
	return SandboxedExecutor::runInSandbox(
		[&]()
		{
			no_data = true;
			some_failed = false;

			data_generator_procedure(*this);

			if (no_data)
			{
				fprintf(stderr, "Nessun set di dati generato dalla data_generator_procedure\n");
				_exit(EXIT_FAILURE);
			}

			if (some_failed)
			{
				_exit(EXIT_FAILURE);
			}
		}, test_name);
}

#endif
