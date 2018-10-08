/* This program was created by ScottC on 8/5/2012 to receive serial
  signals from a computer to turn on/off 1-9 LEDs */

char radii[100];
char angle[100];
String byteRead;
void setup() {
  Serial.begin(9600);
}

void loop() {
  
  
  /* check if data has been sent from the computer: */
  if (Serial.available()) {

    /* read the most recent byte */
    byteRead = Serial.readStringUntil('\n');
    Serial.println(byteRead);
  }
  
  //TO DO: now serial communication worked, change the Processing code and check the csv file next
}
