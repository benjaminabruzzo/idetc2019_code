//Board Arduino Mega 2560 or Mega ADK
//14.04 Serial port /dev/ttyUSB*
// OSX Port /dev/tty.usbmodem14**


int sig = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(sig, OUTPUT);     
}


void loop() {
  digitalWrite(sig, HIGH);  
  delay(33);
  digitalWrite(sig, LOW);   
  delay(33);
} // 25ms high 25ms low --> 50ms wavelength :: 20fps image rate

/*
  This report would have more information with
 "Show verbose output during compilation"
 enabled in File > Preferences.
 Arduino: 1.0.6 (Mac OS X), Board: "Arduino Uno"
 
 */

