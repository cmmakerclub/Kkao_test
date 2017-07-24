
#include "ssl.h"




SSL::SSL()
{	
}
bool SSL::Open(unsigned char pdpctxid,unsigned char sslctxid,unsigned char clientid,String serveraddr,String serverport,unsigned char access_mode)
{
	const long interval = 1000; 
	bool ret = false;
			gsm.print("AT+QSSLOPEN=");
			gsm.print(String(pdpctxid));
			gsm.print(",");
			gsm.print(String(sslctxid));
			gsm.print(",");
			gsm.print(String(clientid));
			gsm.print(",\"");
			gsm.print(String(serveraddr));
			gsm.print("\",");
			gsm.print(String(serverport));
			gsm.print(",");
			gsm.print(String(access_mode));
			gsm.println("");
	
	unsigned long previousMillis = millis(); 
	while(1)
	{

		String req = gsm.readStringUntil('\n');
		gsm.debug(req);
		if(req.indexOf(F("OK"))!= -1)
			ret=true;
		if(req.indexOf(F("+QSSLOPEN:"))!= -1)
		{
			
			unsigned char index = req.indexOf(F(","));
			int ret_ = req.substring(index+1).toInt();
			int ret_2 = req.substring(index).toInt();
			//Serial.println(ret_);
			//Serial.println(ret_2);
			if((ret_==0)&&(ret_2==0))
			{
				
				ret=true;
				gsm.debug("Connect Server Success");
			}
			else
			{
				ret=false;
				gsm.debug("Connect Server Fail");
			}
			return(ret);
		}

		 unsigned long currentMillis = millis();

		if(currentMillis - previousMillis >= interval) 
		{			
			gsm.print("AT+QSSLOPEN=");
			gsm.print(String(pdpctxid));
			gsm.print(",");
			gsm.print(String(sslctxid));
			gsm.print(",");
			gsm.print(String(clientid));
			gsm.print(",\"");
			gsm.print(String(serveraddr));
			gsm.print("\",");
			gsm.print(String(serverport));
			gsm.print(",");
			gsm.print(String(access_mode));
			gsm.println("");
			previousMillis = currentMillis;
		}			
	}
}
bool SSL::Open(String ip_url,String port)
{
	return(Open(1, 0, 0, ip_url, port, 0));
}
bool SSL::StartSend(unsigned char clientid)
{
	const long interval = 3000; 
	gsm.print("AT+QSSLSEND=");
	gsm.println(String(clientid));
	
	unsigned long previousMillis = millis(); 
	while(1)
	{
		if(gsm.available())
		{
			if(gsm.read()=='>')
			{
				gsm.debug("send ready\r\n");
				return(true);
			}	
		}
		unsigned long currentMillis = millis();
		if(currentMillis - previousMillis >= interval) 
		{
			previousMillis = currentMillis;
			return(false);
		}			
		//String req = gsm.readStringUntil('>');
	}
}
bool SSL::StartSend(unsigned char contexid,int len)
{
	const long interval = 3000; 
	unsigned char flag_retry=0;
	gsm.print(F("AT+QSSLSEND="));
	gsm.print(String(contexid));
	gsm.print(",");
	gsm.println(String(len));
	String buffer_="";
	unsigned long previousMillis = millis(); 
	unsigned long currentMillis;
	while(1)
	{
		if(gsm.available())
		{
			char c = gsm.read();
			//Serial.write(c);
			if(c =='>')
			{
				//gsm.debug("send ready");
				return(true);
			}	
			else
			{
				buffer_ += c;
				if(buffer_.indexOf(F("ERROR"))!=-1)
				{
					gsm.debug("Send Error\r\n");
					return(false);
				}
				if(buffer_.indexOf(F("NO CARRIER"))!=-1)
				{
					gsm.debug("NO CARRIER\r\n");
					return(false);
				}
				
				
			}
		/*	if(c =='E')
			{
				gsm.debug("send fail");
				return(false);
			}
*/			
		}
		currentMillis = millis();
		if(currentMillis - previousMillis >= interval) 
		{
			
			flag_retry++;
			gsm.println(F("AT"));
			gsm.wait_ok_ndb(1000);
			gsm.print(F("AT+QSSLSEND="));
			gsm.print(String(contexid));
			gsm.print(",");
			gsm.println(String(len));
			
			previousMillis = currentMillis;
			if(flag_retry>=3)
			{
				gsm.debug("send error (timeout)\r\n");
				return(false);	
			}
			
		}			
		//String req = gsm.readStringUntil('>');
	}
}

bool SSL::StartSend()
{
	return(StartSend(0));
}

bool SSL::StopSend()
{
	
	gsm.write(0x1A);
	const long interval = 3000; 
	unsigned long previousMillis = millis(); 
	unsigned char cnt=0;
	while(!gsm.available())
	{
		unsigned long currentMillis = millis();
		if(currentMillis - previousMillis >= interval) 
		{
			gsm.write(0x1A);
			cnt++;
			if(cnt>3)
				return(false);
		}			
	}
	String req = gsm.readStringUntil('\n');
	if(req.indexOf(F("SEND OK"))!= -1)
	{
		return(true);	
	}
	else
	{
		return(false);
	}
}

bool SSL::Close(unsigned char contexid)
{
	gsm.print("AT+QSSLCLOSE=");
	gsm.println(String(contexid));
	//gsm.println("AT+QICLOSE=0");
	//while(!gsm.available()){}
	uint8_t WD = 100;
	while(gsm.available())
	{
			String req = gsm.readStringUntil('\n');
			//Serial.println(req);
			if(req.indexOf(F("OK"))!= -1)
			{
				return(true);
			}
			else
			{
				return(false);
			}
	}
	
}
bool SSL::Close()
{
	return(Close(0));
}


void SSL::write(char data)
{
	gsm.write(data);
}
void SSL::print(int data)
{
	gsm.print(String(data));
}
void SSL::println(int data)
{
	gsm.println(String(data));
}
void SSL::print(String data)
{
	gsm.print(data);
}
void SSL::println(String data)
{
	gsm.println(data);
}
