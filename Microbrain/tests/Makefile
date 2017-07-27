.DEFAULT_GOAL := all
.PHONY: all test

OUTPUTDIR = output
TESTFRAMEWORK = testframework.cpp testframework.h
SVGFRAMEWORK = geometrysvg.h svgframework.cpp svgframework.h
TESTCFLAGS = -std=gnu++11 -I..

# Questo è un test di esempio
#TESTS += test_example
#$(OUTPUTDIR)/test_example: test_example.cpp $(TESTFRAMEWORK)
#	g++ -o $@ $(filter-out %.h,$^) $(TESTCFLAGS)

TESTS += test_geometry
$(OUTPUTDIR)/test_geometry: test_geometry.cpp ../geometry.cpp ../geometry.h $(TESTFRAMEWORK) $(SVGFRAMEWORK)
	g++ -o $@ $(filter-out %.h,$^) $(TESTCFLAGS)

all: $(addprefix $(OUTPUTDIR)/,$(TESTS))

clean:
	rm -rf $(OUTPUTDIR)/*

test: all
	@S=0; cd $(OUTPUTDIR) && for f in $(TESTS); do if ! ./$$f; then S=1; fi; done; exit $$S
