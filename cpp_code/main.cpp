#include <cstdlib>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <stdlib.h>

#include "PathPlanner.h"


int main (int argc, char** argv) {

int startX = std::atoi(argv[1]);
int startY = std::atoi(argv[2]);
int start[2] = {startX,startY};

int goalX = std::atoi(argv[3]);
int goalY = std::atoi(argv[4]);
int goal[2] = {goalX,goalY};

PathPlanner planner("bitmap",0);
//planner.setScaling(0);
planner.Planning(start,goal);
#ifdef DEBUG
planner.displayCosts();
#endif
boost::asio::io_service io_service;
boost::asio::ip::tcp::resolver resolver(io_service);
boost::asio::ip::tcp::resolver::query query("127.0.0.1", "1234");
boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
boost::asio::ip::tcp::resolver::iterator end;

boost::asio::ip::tcp::socket socket(io_service);
boost::system::error_code error = boost::asio::error::host_not_found;

while (error && endpoint_iterator != end) {

	socket.close();
	socket.connect(*endpoint_iterator++, error);
	}
	
if(error) {
	throw boost::system::system_error(error);
	}
	
boost::asio::streambuf buffer;
std::ostream stream(&buffer);
	
for(int i =0; i< planner.getPath().nodes.size();i++) {
char bufferX [10];
char bufferY [10];

sprintf(bufferX,"%d",planner.getPath().nodes[i]->x);
sprintf(bufferY,"%d",planner.getPath().nodes[i]->y);

	stream << "(" << bufferX << "," << bufferY << ")";

}
	socket.write_some(buffer.data());
	
	
return 0;
}
