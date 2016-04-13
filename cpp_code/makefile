libs = 
std = -std=c++11

demo:
	g++ main.cpp PathPlanner.cpp -o demo.out $(libs) $(std) -DDEBUG -g
	mv demo.out ../demo.out

lib:
	g++ -c PathPlanner.cpp	$(libs) $(std)

clean:
	rm -f *.o *.out
