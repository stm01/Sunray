// HTTP server for socket-based client

#ifndef SERVER_H
#define SERVER_H

#include <Arduino.h>


class ServerClass {
    public:      
			String content;
			String receivedData;
			int hdrLen;
			void begin();
			void sendHTTPNotFound();
			void initHTTP();
			void finishHTTP();
			void sendInfo();
			void serveHTTP();
    protected:                 
};

extern ServerClass Server;

#endif

