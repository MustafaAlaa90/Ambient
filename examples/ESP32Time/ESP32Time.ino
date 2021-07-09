/**
  @file     ESP32Time.ino
  @author   kerikun11
  @date     2017.08.29
*/

#include <WiFi.h>
#include <ESP32Time.h>

const char* ssid = "Esraa";
const char* psk = "Esraa+28121995";

void setup() {
  Serial.begin(115200);

  Serial.printf("connecting to %s ...\n", ssid);
  WiFi.begin(ssid, psk);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi connection failed:(\n");
    while (1) delay(1000);
  }

  ESP32Time.begin(); //< adjusted the time
}

void loop() {
  time_t t = time(NULL);
  struct tm *t_st;
  t_st = localtime(&t);
  Serial.printf("year: %d\n", 1900 + t_st->tm_year);
  Serial.printf("month: %d\n", 1 + t_st->tm_mon);
  Serial.printf("month day: %d\n", t_st->tm_mday);
  Serial.printf("week day: %c%c\n", "SMTWTFS"[t_st->tm_wday], "uouehra"[t_st->tm_wday]);
  Serial.printf("year day: %d\n", 1 + t_st->tm_yday);
  Serial.printf("hour: %d\n", t_st->tm_hour);
  Serial.printf("minute: %d\n", t_st->tm_min);
  Serial.printf("second: %d\n", t_st->tm_sec);
  Serial.printf("ctime: %s\n", ctime(&t));
  delay(1000);
}

