/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char *ssid = "Molk";
const char *password = "Molk0901";

ESP8266WebServer server ( 80 );

const int led = 13;
unsigned char incomingByte[2];
int fukt = 40;
int temperatur = 23;
int buff_pos = 0;
int data_pos = 0;
unsigned char temp_array[40];
unsigned char humidity_array[40];

void handleRoot() {
	digitalWrite ( led, 1 );
	char temp[600];
	int sec = millis() / 1000;
	int min = sec / 60;
	int hr = min / 60;

	snprintf ( temp, 600,

"<html>\
  <head>\
    <meta http-equiv='refresh' content='15'/>\
    <title>For Teh Loop\!</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1><b>ESP8266 bids you welcome!</b></h1>\
    <p>Uptime: %02d:%02d:%02d</p>\
    <p><b>Temperature</b></p>\
    <img src=\"/temp.svg\" />\
    <p><b>Humidity</b></p>\
    <img src=\"/hum.svg\" />\
    <p><b>\nLed off <a href=\"socket1Off\">OFF</a> \| <a href=\"socket1On\">ON</a></b></p>\
  </body>\
</html>",

		hr, min % 60, sec % 60
	);

  
	server.send ( 200, "text/html", temp );
	digitalWrite ( led, 0 );
}

void handleNotFound() {
	digitalWrite ( led, 1 );
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";

	for ( uint8_t i = 0; i < server.args(); i++ ) {
		message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
	}

	server.send ( 404, "text/plain", message );
	digitalWrite ( led, 0 );
}

void setup ( void ) {
	pinMode ( led, OUTPUT );
	digitalWrite ( led, 0 );
	Serial.begin ( 19200 );
	WiFi.begin ( ssid, password );

  // To get a static IP-Adress..
  //WiFi.config(IPAddress(192,168,153,21), IPAddress(192,168,153,1), IPAddress(255,255,255,0),IPAddress(192,168,153,1));
  
	Serial.println ( "" );

  incomingByte[0] = 0;
  incomingByte[1] = 0;

	// Wait for connection
	while ( WiFi.status() != WL_CONNECTED ) {
		delay ( 500 );
		Serial.print ( "." );
	}

	Serial.println ( "" );
	Serial.print ( "Connected to " );
	Serial.println ( ssid );
	Serial.print ( "IP address: " );
	Serial.println ( WiFi.localIP() );

	if ( MDNS.begin ( "esp8266" ) ) {
		Serial.println ( "MDNS responder started" );
	}

	server.on ( "/", handleRoot );
	server.on ( "/temp.svg", drawTemp );
  server.on ( "/hum.svg", drawHum );
	server.on ( "/inline", []() {
		server.send ( 200, "text/plain", "this works as well" );
	} );
  server.on("/socket1On", [](){
    server.send ( 200, "text/html", 
    
    "<html>\
        <head>\
          <meta http-equiv='refresh' content='0;URL=/'/>\
          <title>On</title>\
          <style>\
            body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
          </style>\
        </head>\
    <body>\
    </body>\
  </html>"
    
   );
    Serial.println ( 0x01 );
    delay(500);
  });
  server.on("/socket1Off", [](){
    server.send ( 200, "text/html", 
    
    "<html>\
        <head>\
          <meta http-equiv='refresh' content='0;URL=/'/>\
          <title>Off</title>\
          <style>\
            body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
          </style>\
        </head>\
    <body>\
    </body>\
  </html>"
    
   );
    Serial.println ( 0x00 );
    delay(500); 
  });
	server.onNotFound ( handleNotFound );
	server.begin();
	Serial.println ( "HTTP server started" );

  for (int i = 0 ; i < 40; i++){
    temp_array[i] = 0;
    humidity_array[i] = 0;
  }
}

void loop ( void ) {
	server.handleClient();

          // send data only when you receive data:
        if (Serial.available() > 0) {
                
                 // read the incoming byte:
                incomingByte[buff_pos++] = Serial.read();

                if (buff_pos > 1) {
                  temp_array[data_pos] = incomingByte[0];
                  humidity_array[data_pos++] = incomingByte[1];
                  buff_pos = 0;

                  if ( data_pos >= 41 ) {
                      data_pos = 0;
                      
                      for (int i = 0 ; i < 40; i++){
                        temp_array[i] = 0;
                        humidity_array[i] = 0;
                      }
                  }
                }

                // say what you got:
                //Serial.print("I received: ");
                //Serial.println(incomingByte, HEX);
        }
}

void drawTemp() {
	String out = "";
  
	char temp[100];
  int pos = 0;
 
	out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"60\">\n";
 	out += "<rect width=\"400\" height=\"60\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
 	out += "<g stroke=\"black\">\n";

  int y = temp_array[pos++];
 	for (int x = 10; x < 390; x+= 10) {
    int y2 = temp_array[pos++];
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 50 - y, x + 10, 50 - y2);
    
 		out += temp;
    y = y2;
 	}
	out += "</g>\n</svg>\n";

	server.send ( 200, "image/svg+xml", out);
}

void drawHum() {
  String out = "";
  
  char temp[100];
  int pos = 0;
 
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"110\">\n";
  out += "<rect width=\"400\" height=\"110\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";

  int y = humidity_array[pos++];
  for (int x = 10; x < 390; x+= 10) {
    int y2 = humidity_array[pos++];
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 100 - y, x + 10, 100 - y2);
    
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send ( 200, "image/svg+xml", out);
}
