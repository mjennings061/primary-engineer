void WiFi_setup() {
  // check for the presence of the WiFi shield / peripheral:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield / peripheral not present");
    // Don't continue if there isn't any WiFi capability on the device
    while (true);
  }
  Serial.println("WiFi shield / peripheral present");


  // Create open network if switch is in host position. Change this line if you want to create an WEP network:
  if (Host){
  Serial.println("Roadflash in host mode");
  setStrip(purple); // Display using Neopixel that we are in Host Mode
  delay(4000);
  setStrip(black);
  
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point has failed");
    // don't continue
    while (true);
  }

  Serial.println("Access point created, setting ");

  WiFi.config(IPAddress(192, 168, 1, 1));
  
  // you're connected now, so print out the status
  printWiFiStatus();
  }

  

  else{
    Serial.println("Roadflash in device mode");
    setStrip(amber); // Display using Neopixel that we are in Device Mode
    delay(4000);
    setStrip(black);

    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);

      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid);

      setStrip(blue);
      // wait 4 seconds for connection:
      delay(4000);
      setStrip(black);
      }

      setStrip(aqua);
      Serial.println("WiFi Connected, settting IP Address");
      delay(1000); // Delay to make sure the host is ready for the IP Address change, I had issues without this.
      WiFi.config(IPAddress(192, 168, 1, 5));
      
      setStrip(green);      
      Serial.println("IP Address Set");
      delay(1000);
      setStrip(black);
      printWiFiStatus();
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}


void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
