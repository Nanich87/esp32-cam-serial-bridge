#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BluetoothSerial.h>

#define MAX_CLIENTS 4
#define MAX_SERVERS 1
#define BUFFER_SIZE 1024
#define SERVER_PORT 8888
#define WIFI_SSID "NEO-6M"
#define WIFI_PASSWORD "12345678"
#define SERIAL_BAUD_RATE 115200

HardwareSerial* hs = &Serial;
BluetoothSerial SerialBT;

WiFiServer server_0(SERVER_PORT);
WiFiServer *server[MAX_SERVERS] = {&server_0};

WiFiClient TCPClient[MAX_SERVERS][MAX_CLIENTS];

uint8_t tcp_buffer[BUFFER_SIZE];
uint16_t i1 = 0;

uint8_t serial_buffer[BUFFER_SIZE];
uint16_t i2 = 0;

uint8_t bt_buffer[BUFFER_SIZE];
uint16_t i3 = 0;

void setup() {
  delay(500);

  hs->begin(SERIAL_BAUD_RATE, SERIAL_8N1, 3, 1);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  SerialBT.begin(WIFI_SSID);

  server[0]->begin();
  server[0]->setNoDelay(true);

  esp_err_t esp_wifi_set_max_tx_power(50);
}

void loop()
{
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      bt_buffer[i3] = SerialBT.read();

      if (i3 < BUFFER_SIZE - 1)
      {
        i3++;
      }
    }

    hs->write(bt_buffer, i3);

    i3 = 0;
  }

  if (server[0]->hasClient())
  {
    for (byte i = 0; i < MAX_CLIENTS; i++)
    {
      if (!TCPClient[0][i] || !TCPClient[0][i].connected())
      {
        if (TCPClient[0][i])
        {
          TCPClient[0][i].stop();
        }

        TCPClient[0][i] = server[0]->available();

        continue;
      }
    }

    WiFiClient TmpserverClient = server[0]->available();
    TmpserverClient.stop();
  }

  if (hs != NULL)
  {
    for (byte i = 0; i < MAX_CLIENTS; i++)
    {
      if (TCPClient[0][i])
      {
        while (TCPClient[0][i].available())
        {
          tcp_buffer[i1] = TCPClient[0][i].read();

          if (i1 < BUFFER_SIZE - 1)
          {
            i1++;
          }
        }

        hs->write(tcp_buffer, i1);

        i1 = 0;
      }
    }

    if (hs->available())
    {
      while (hs->available())
      {
        serial_buffer[i2] = hs->read();

        if (i2 < BUFFER_SIZE - 1)
        {
          i2++;
        }
      }

      for (byte i = 0; i < MAX_CLIENTS; i++)
      {
        if (TCPClient[0][i])
        {
          TCPClient[0][i].write(serial_buffer, i2);
        }
      }

      if (SerialBT.hasClient())
      {
        SerialBT.write(serial_buffer, i2);
      }

      i2 = 0;
    }
  }
}
