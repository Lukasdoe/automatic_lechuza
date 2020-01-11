#include <PubSubClient.h> //simple MQTT pub sub lib
#include "DHTesp.h" //to work with the DHT
#include <ESP8266WiFi.h> //ESP Wifi connection
#include <Esp.h>
#include <esp8266_peri.h>
#include <eagle_soc.h>

#define VALVE D3 //Pin D3 is connected to Pump TIP120
#define MOIST_PIN D7
#define DHT_PIN D5 //DHT Port is connected to Pin D5
#define DHTTYPE DHT11 //DHT11 or DHT22
#define LED_RED D2
#define LED_GREEN D8
#define LED_BLUE D6

// compare 2 unsigned value
// true if X > Y while for all possible (X, Y), X - Y < Z
#define TIME_CMP(X, Y, Z) (((X) - (Y)) < (Z))

//Wifi Network Info
#define wifi_ssid "WIFI_NETWORK_SSID"
#define wifi_password "PASSWORD"

#define WDT_CTL 0x60000900
#define WDT_CTL_ENABLE (BIT(0))

//MQTT Access Information
#define mqtt_server "SERVER_URL"
#define mqtt_user "MQTT_USER_NAME"
#define mqtt_password "MQTT_PASSWORD"

//MQTT Topics
#define humidity_topic "sensor/humidity"
#define temperature_topic "sensor/temperature"
#define moisture_topic "sensor/soilmoisture"

WiFiClient espClient; //Esp internet connection client
PubSubClient client(espClient);
DHTesp dht;

unsigned long lastMillis = 0;
long lastMsg;
float temp;
float hum;
float moist;
int diff;

int minMoist;
int maxMoist;

//connect wifi
void setup_wifi() {
  delay(50);
  // Start by connecting esp to wifi
  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(wifi_ssid);

  minMoist = 68;
  maxMoist = 200;

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//reconnect wifi
void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    digitalWrite(LED_RED, HIGH);
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("switch/pump");
      client.subscribe("switch/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  digitalWrite(LED_RED, LOW);
}

void setup()
{
  pinMode(VALVE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(MOIST_PIN, OUTPUT);

  lastMsg = 0;
  temp = 22.0;
  hum = 0.0;
  diff = 0.5;
  moist = 0.0;

  Serial.begin(9600);
  digitalWrite(LED_BLUE, HIGH);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Start DHT
  dht.setup(DHT_PIN, DHTesp::DHTTYPE);
  digitalWrite(LED_BLUE, LOW);

  // disable hardware watchdog
  CLEAR_PERI_REG_MASK(WDT_CTL, WDT_CTL_ENABLE);

  while(!client.connected()) {
    reconnect();
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;

    float newTemp = dht.getTemperature(); //read DHT sensor
    float newHum = dht.getHumidity();

    float newMoist = checkMoisture();

    if (checkBound(newTemp, temp, diff)) {
        temp = newTemp;
        Serial.print("New temperature:");
        Serial.println(String(temp).c_str());
        client.publish(temperature_topic, String(temp).c_str(), true);
    }

    if (checkBound(newHum, hum, diff)) {
      hum = newHum;
      Serial.print("New humidity:");
      Serial.println(String(hum).c_str());
      client.publish(humidity_topic, String(hum).c_str(), true);
    }

    if (checkBound(newMoist, moist, diff)) {
      if(digitalRead(VALVE) == LOW){
        moist = newMoist;
        Serial.print("New soil moisture:");
        Serial.println(String(moist).c_str());
        client.publish(moisture_topic, String(moist).c_str(), true);
      }
    }
  }
}

//validate new readings
bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) && (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

inline void setHigh() {
  GPOS = (1 << MOIST_PIN);
}

inline void setLow() {
  GPOC = (1 << MOIST_PIN);
}

int checkMoisture(){
  unsigned long cycle_time = 4;
  unsigned long raising_edge = 0;
  unsigned long falling_edge = 2;
  
  // disable software watchdog
  ESP.wdtDisable();
  unsigned long prev_micros = micros();
  for(int i = 0; i < 100000; i++) {
      // raising edge should appear earlier
      while (TIME_CMP(micros(), prev_micros + raising_edge, cycle_time)); setHigh();
      while (TIME_CMP(micros(), prev_micros + falling_edge, cycle_time)); setLow();
    prev_micros += cycle_time;
  }
  int res = analogRead(A0);
  ESP.wdtEnable(1000);
  //Serial.println(res);
  if(res < minMoist) minMoist--;
  if(res > maxMoist) maxMoist++;
  return map(res, maxMoist, minMoist, 0, 100);
}

void callback(char* topic, byte* payload, unsigned int length) {
  //convert topic to string to make it easier to work with
  String topicStr = topic;
  //EJ: Note:  the "topic" value gets overwritten everytime it receives confirmation (callback) message from MQTT

  //Print out some debugging info
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.println(payload[0]);

  if (topicStr == "switch/pump")
  {
    if (payload[0] == 48) {
      digitalWrite(VALVE, HIGH);
      delay(2000);
      digitalWrite(VALVE, LOW);
    }

    else if (payload[0] == 49) {
      digitalWrite(VALVE, LOW);
    }
  }
  else if (topicStr == "switch/led")
  {
    if (payload[0] == 48) {
      digitalWrite(LED_RED, HIGH);
    }
    else if (payload[0] == 49) {
      digitalWrite(LED_RED, LOW);
    }
    else if (payload[0] == 50) {
      digitalWrite(LED_GREEN, HIGH);
    }
    else if (payload[0] == 51) {
      digitalWrite(LED_GREEN, LOW);
    }
    else if (payload[0] == 52) {
      digitalWrite(LED_BLUE, HIGH);
    }
    else if (payload[0] == 53) {
      digitalWrite(LED_BLUE, LOW);
    }
  }
}
