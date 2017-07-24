#ifndef SSL_h
#define SSL_h

#include "TEE_UC20.h"


class SSL
{
public:
	SSL();
	bool Open(unsigned char pdpctxid,unsigned char sslctxid,unsigned char clientid,String serveraddr,String serverport,unsigned char access_mode);
	bool Open(String ip_url,String port);
	bool StartSend(unsigned char contexid);
	bool StartSend(unsigned char contexid,int len);
	bool StartSend();
	bool StopSend();
	void write(char data);
	void print(String data);
	void println(String data);
	void print(int data);
	void println(int data);
	bool Close(unsigned char contexid);
	bool Close();

	
	unsigned char ReceiveConnectID;
	
};

#endif