all: find_ants snap
find_ants.o: find_ants.cpp
	g++ -ggdb `pkg-config --cflags opencv` -c find_ants.cpp 
find_ants: find_ants.o
	g++ -ggdb -o find_ants find_ants.o `pkg-config --libs opencv`
snap: snap.o
	g++ -ggdb -o snap snap.o `pkg-config --libs opencv`
