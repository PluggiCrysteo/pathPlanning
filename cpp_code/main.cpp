#include <string.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <stdio.h>

#include "PathPlanner.h"

#define BUF_SIZE 1024
#define BACKLOG 10
#define PORT 9876

#define BITMAP_PATH "~/content/bitmaps/"

#ifdef DEBUG
#define DEBUG_(x, ...) fprintf(stderr,"\033[31m%s/%s/%d: " x "\033[0m\n",__FILE__,__func__,__LINE__,##__VA_ARGS__);
#else
#define DEBUG_(x, ...)
#endif

#define ERROR_(x, ...) fprintf(stderr,"%s/%s/%d: " x "\n",__FILE__,__func__,__LINE__,##__VA_ARGS__);

std::vector<std::string> split(const std::string &s, char delim);
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
void handleConnection(int fd, char* buf,std::string oldBitmap,int* oldScaling,PathPlanner* planner);
ssize_t readLine(int fd, void *buffer, size_t n);

int main (int argc, char** argv) {

	if(argc < 2) 
		ERROR_("args missing");

	// parsing etc... variables
	std::string oldBitmap;
	int oldScaling;
	PathPlanner* pathPlanner = NULL;

	// socket variables
	struct sockaddr_un addr;
	int sfd, cfd;
	ssize_t numRead;
	char buf[BUF_SIZE];

	sfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sfd == -1)
		perror("socket");

	if (remove(argv[1]) == -1 && errno != ENOENT)
		perror("remove");

	memset(&addr, 0, sizeof(struct sockaddr_un));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, argv[1], sizeof(addr.sun_path) - 1);

	if (bind(sfd, (struct sockaddr *) &addr, sizeof(struct sockaddr_un)) == -1)
		perror("bind");

	if (listen(sfd, BACKLOG) == -1)
		perror("listen");

	DEBUG_("listening for connections");

	for (;;) {

		cfd = accept(sfd, NULL, NULL);
		if (cfd == -1)
			perror("accept");

		if((numRead = readLine(cfd, buf, BUF_SIZE)) > 0) {
			DEBUG_("received line");
			handleConnection(cfd,buf,oldBitmap,&oldScaling,pathPlanner);
		}
		else if (numRead == -1)
			perror("read");


		if (close(cfd) == -1)
			perror("close");
	}

	return 0;
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


void handleConnection(int fd, char* buf,std::string oldBitmap,int* oldScaling,PathPlanner* planner) {
	std::vector<std::string> tokens = split(std::string(buf),';');
	if(tokens[0] != oldBitmap) {
		DEBUG_("replacing bitmap with a new one: %s",tokens[0].c_str());
		delete planner;
		DEBUG_("instantiating new planner, with scaling: %d",std::stoi(tokens[5]));
		planner = new PathPlanner(tokens[0].c_str(),std::stoi(tokens[5]));
		oldBitmap = tokens[0];
		DEBUG_("instantiated planner");
		*oldScaling = std::stoi(tokens[5]);
	} else if (std::stoi(tokens[5]) != *oldScaling) {
		*oldScaling = std::stoi(tokens[5]);
		planner->setScaling(*oldScaling);
	}

	DEBUG_("getting values of start/goal point");
	int start[2] = {std::stoi(tokens[1]),std::stoi(tokens[2])};
	int goal[2] = {std::stoi(tokens[3]),std::stoi(tokens[4])};

	DEBUG_("starting the planner");
	planner->Planning(start,goal);
	std::string tosend;

	DEBUG_("sending back data");
	for(int i =0; i< planner->getPath().nodes.size();i++) {
		char buffer [23];

		sprintf(buffer,"(%d;%d)",planner->getPath().nodes[i]->x,planner->getPath().nodes[i]->y);

		tosend += buffer;
	}
	
	DEBUG_("tosend: %s",tosend.c_str());
	DEBUG_("tosend.size: %d",tosend.size());
	DEBUG_("written: %d",write(fd,tosend.c_str(),tosend.size()));
}

ssize_t readLine(int fd, void *buffer, size_t n)
{
	ssize_t numRead;                    /* # of bytes fetched by last read() */
	size_t totRead;                     /* Total bytes read so far */
	char *buf;
	char ch;

	if (n <= 0 || buffer == NULL) {
		errno = EINVAL;
		return -1;
	}

	buf = (char*)buffer;                       /* No pointer arithmetic on "void *" */

	totRead = 0;
	for (;;) {
		numRead = read(fd, &ch, 1);

		if (numRead == -1) {
			if (errno == EINTR)         /* Interrupted --> restart read() */
				continue;
			else
				return -1;              /* Some other error */

		} else if (numRead == 0) {      /* EOF */
			if (totRead == 0)           /* No bytes read; return 0 */
				return 0;
			else                        /* Some bytes read; add '\0' */
				break;

		} else {                        /* 'numRead' must be 1 if we get here */
			if (totRead < n - 1) {      /* Discard > (n - 1) bytes */
				totRead++;
				*buf++ = ch;
			}

			if (ch == '\n')
				break;
		}
	}

	*buf = '\0';
	return totRead;
}
