robot_map_test:
	g++ -std=c++17 -o robot_map_test.o robot_map_test.cpp
robot_planning:
	g++ -std=c++17 -o robot_planning.o robot_planning.cpp -lpthread
clean:
	rm *.o