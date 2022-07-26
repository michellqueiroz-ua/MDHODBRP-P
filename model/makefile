CC		= g++ -Wall -g -fPIC -fexceptions -DNDEBUG -DIL_STD -std=c++11 -O3 -fopenmp -Wl,--no-as-needed -ldl
#CC		= g++ -Wall -g -fPIC -fexceptions -DNDEBUG -DIL_STD -std=c++11 -O3 -Wl

CFLAGS1 = -I/opt/ibm/ILOG/CPLEX_Studio128/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio128/concert/include/
CFLAGS2	= -L/opt/ibm/ILOG/CPLEX_Studio128/cplex/lib/x86-64_linux/static_pic/ -lilocplex -lcplex -L/opt/ibm/ILOG/CPLEX_Studio128/concert/lib/x86-64_linux/static_pic/ -lconcert -lm -lpthread


FILES_CC	= MDHODBRP_model.cpp Input.cpp
#FILES_CC	= mfvs-mwif-new.cpp


BINARY		= mdhodbrpmodel

all:	clean compile

clean:
	find . -name '*.o' -exec rm -f '{}' ';'
	rm -f $(BINARY);

compile:
	$(CC) $(CFLAGS1)  $(FILES_CC) -o $(BINARY)  $(CFLAGS2)
	#$(CC)  $(FILES_CC) -o $(BINARY)

run:
	./$(BINARY)