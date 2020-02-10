//upload the sensor data to thingspeak
bool uploadToThingspeak() {
  Serial.println("Uploading to thingspeak");
  if (thingspeakApi == "") {
    Serial.println("Thingspeak API key not set");
    return false;
  }
  
  printUploadValues();
  
  String thingspeakHost = "api.thingspeak.com";
  String thingspeakUrl = "/update";
  thingspeakUrl += "?api_key=" + thingspeakApi;
  if (tsfWindSpeed != 0)
    thingspeakUrl += "&field" + String(tsfWindSpeed) + "=" + String(windSpeedAvg*3.6); //km/h
  if (tsfWindDir != 0)
    thingspeakUrl += "&field" + String(tsfWindDir) + "=" + String(windDirAvg);
  if (tsfRainAmount != 0)
    thingspeakUrl += "&field" + String(tsfRainAmount) + "=" + String(rainAmountAvg);
  if (tsfTemperature != 0)
    thingspeakUrl += "&field" + String(tsfTemperature) + "=" + String(temperature);
  if (tsfHumidity != 0)
    thingspeakUrl += "&field" + String(tsfHumidity) + "=" + String(humidity);
  if (tsfAirpressure != 0)
    thingspeakUrl += "&field" + String(tsfAirpressure) + "=" + String(pressure);
  if (tsfPM1 != 0)
    thingspeakUrl += "&field" + String(tsfPM1) + "=" + String(PM1);
  if (tsfPM2 != 0)
    thingspeakUrl += "&field" + String(tsfPM2) + "=" + String(PM2);
  if (tsfPM10 != 0)
    thingspeakUrl += "&field" + String(tsfPM10) + "=" + String(PM10);

  String resp = performRequest(false, thingspeakHost, thingspeakUrl);
  return (resp != "" && !resp.startsWith("0"));
}

//upload the sensor values to senseBox
bool uploadToSenseBox() {
  Serial.println("Uploading to SenseBox");
  if (senseBoxStationId == "") {
    Serial.println("SenseBox station ID not set");
    return false;
  }
  
  printUploadValues();
  
  String csv;
  if (senseBoxWindSId != "")
    csv += senseBoxWindSId + "," + String(windSpeedAvg*3.6) + "\r\n"; //km/h
  if (senseBoxWindDId != "")
    csv += senseBoxWindDId + "," + String(windDirAvg) + "\r\n";
  if (senseBoxRainId != "")
    csv += senseBoxRainId + "," + String(rainAmountAvg) + "\r\n";
  if (senseBoxTempId != "")
    csv += senseBoxTempId + "," + String(temperature) + "\r\n";
  if (senseBoxHumId != "")
    csv += senseBoxHumId + "," + String(humidity) + "\r\n";
  if (senseBoxPressId != "")
    csv += senseBoxPressId + "," + String(pressure) + "\r\n";
  if (senseBoxPM1Id != "")
    csv += senseBoxPM1Id + "," + String(PM1) + "\r\n";
  if (senseBoxPM2Id != "")
    csv += senseBoxPM2Id + "," + String(PM2) + "\r\n";
  if (senseBoxPM10Id != "")
    csv += senseBoxPM10Id + "," + String(PM10) + "\r\n";

  if (csv == "") {
    Serial.println("Sensor API keys not set");
    return false;
  }
  
  String senseBoxHost = "api.opensensemap.org";
  String senseBoxUrl = "/boxes/" + senseBoxStationId + "/data";
  String headers = "Content-Type: text/csv\r\n";
  headers += "Connection: close\r\n";
  headers += "Content-Length: " + String(csv.length()) + "\r\n";

  String resp = performRequest(true, senseBoxHost, senseBoxUrl, 443, "POST", headers, csv);
  return true;
}

//upload to ubidots
bool uploadToUbidots(){
  if (!mqttclient.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attemp to connect
    if (mqttclient.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again next time");
      return false;
    }
  }

  if (mqttclient.connected()){
    Serial.println("Publishing data to Ubidots Cloud");
    
    sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    String payload_str = "";
    payload_str +="{";
    payload_str +="\"temp\":";
    payload_str +=temperature;
    payload_str +=",";
    payload_str +="\"pm1\":";
    payload_str +=PM1;
    payload_str +=",";
    payload_str +="\"pm2\":";
    payload_str +=PM2;
    payload_str +=",";
    payload_str +="\"pm10\":";
    payload_str +=PM10;
    payload_str +=",";
    payload_str +="\"uva\":";
    payload_str +=UVA;
    payload_str +=",";
    payload_str +="\"uvb\":";
    payload_str +=UVB;
    payload_str +=",";
    payload_str +="\"uvindex\":";
    payload_str +=UVI;
    payload_str +="}";
    Serial.println(topic);
    Serial.println(payload_str);
    mqttclient.publish(topic, payload_str.c_str());
    mqttclient.loop();

    payload_str = "";
    payload_str +="{";
    payload_str +="\"tsl_lux\":";
    payload_str +=tsl_lux;
    payload_str +=",";
    payload_str +="\"tsl_ir\":";
    payload_str +=tsl_ir;
    payload_str +=",";
    payload_str +="\"tsl_full\":";
    payload_str +=tsl_full;
    payload_str +=",";
    payload_str +="\"tsl_vis\":";
    payload_str +=tsl_vis;
    payload_str +="}";
    Serial.println(payload_str);
    mqttclient.publish(topic, payload_str.c_str());
    mqttclient.loop();

    delay(1000);
    
    payload_str = "";
    payload_str +="{";
    payload_str +="\"bmp_temperature\":";
    payload_str +=bmp_temperature;
    payload_str +=",";
    payload_str +="\"bmp_pressure\":";
    payload_str +=bmp_pressure;
    payload_str +=",";
    payload_str +="\"bmp_altitude\":";
    payload_str +=bmp_altitude;
    payload_str +="}";
    Serial.println(payload_str);
    mqttclient.publish(topic, payload_str.c_str());
    mqttclient.loop();

    payload_str = "";
    payload_str +="{";
    payload_str +="\"winddirdeg\":";
    payload_str +="{\"value\":";
    payload_str +=  windDirDeg;
    payload_str +=",";
    payload_str +="\"context\":{\"direction\":\"";
    payload_str += windDirStr;
    payload_str +="\"}}";
    payload_str +="}";
    Serial.println(payload_str);
    mqttclient.publish(topic, payload_str.c_str());
    mqttclient.loop();

    return true;
  }

  return false;
}


void printUploadValues() {
  Serial.println("Values:");
  Serial.println("WindSpeedAvg:  " + String(windSpeedAvg));
  Serial.println("WindDirAvg:    " + String(windDirAvg));
  Serial.println("RainAmountAvg: " + String(rainAmountAvg));
  Serial.println("Temperature:   " + String(temperature));
  Serial.println("Humidity:      " + String(humidity));
  Serial.println("Pressure:      " + String(pressure));
  Serial.println("PM1:          " + String(PM1));
  Serial.println("PM2:          " + String(PM2));
  Serial.println("PM10:          " + String(PM10));
}

