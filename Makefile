all:
	g++ -O2 -Wall -o exemple example.cpp `pkg-config --cflags --libs opencv`
	g++ -O2 -Wall -o gray main.cpp `pkg-config --cflags --libs opencv`
clean:
	rm -rf exemple
