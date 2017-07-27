// Test di esempio

#include "testframework.h"

// Esempio di test semplice, che consiste nell'esecuzione di una singola procedura
static void exampleTest1()
{
	int a, b;

	a = 5;
	b = ++a;

	assert(a == 6);
	assert(b == 6);
}

REGISTER_TEST(exampleTest1)

// Esempio di test reiterato, che consiste nell'esecuzione ripetuta della
// procedura di test, che viene eseguita una volta per ogni input prodotto
// dalla procedura di generazione dei dati
static void exampleTest2_data(const TestWithDataRunner<int, int> &testRunner)
{
	testRunner.runOnData("cinque", 5, 6);	// questo test ha successo
	testRunner.runOnData("sei", 6, 6);	// questo test fallisce
}

static void exampleTest2(int i, int j)
{
	assert(i != j);
}

REGISTER_TEST_WITH_DATA(exampleTest2, exampleTest2_data, int, int)
