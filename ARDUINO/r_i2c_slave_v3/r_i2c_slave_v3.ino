/*
  ROBUTER ENCODER PULSES COUNTER

  Counts and increments the values of each encoder (Left(L) and Right(R)) and sends them to
  Arduino Leonardo ETH via I2C

  Counts the encoder pulses via ISR and transmits them when prompted by the master (Arduino Leonardo)

  created Mar 2017
  by Bruno Vieira
  in Universidade de Aveiro

*/

//=========================================================================================================
//=============================================== LIBRARIES ===============================================
//=========================================================================================================

#include <Wire.h>


//=========================================================================================================
//========================================== VARIABLES / DEFINES ==========================================
//=========================================================================================================

#define encoderLBpin 6
#define encoderRBpin 7
#define ENCODER_SLAVE 1

// unsigned 16 bits for encoder pulses (Left chA, Left chB, Right chA, Right chB)
uint16_t count1 = 0, count2 = 0;


//=========================================================================================================
//================================================= SETUP =================================================
//=========================================================================================================

void setup() {

  pinMode(encoderLBpin, INPUT);
  pinMode(encoderRBpin, INPUT);

  Serial.begin(9600);

  // Start the I2C Bus as Slave on address 1 (ENCODER_SLAVE)
  Wire.begin(ENCODER_SLAVE);
  // Attach a function to trigger when something is received.
  Wire.onRequest(requestEvent);


  // interrupt for Left encoder channel A rising edge, pin 1 (Arduino Micro)
  attachInterrupt(3, encoderL, RISING);
  // interrupt for Right encoder channel A rising edge, pin 0 (Arduino Micro)
  attachInterrupt(2, encoderR, RISING);
}

void requestEvent() {

  Wire.write((count1 >> 8) ); // sends the MSB of variable & 0xFF
  Wire.write(count1 );    // sends LSB of variable


  Wire.write(count2);    // sends LSB of variable
  Wire.write((count2 >> 8)); // sends the MSB of variable

  Serial.print("------------------");
  Serial.print("REQ");
  Serial.print("------------------");

  // reset counter values
  count1 = 0;
  count2 = 0;
}


//=========================================================================================================
//============================================== MAIN LOOP ================================================
//=========================================================================================================

void loop() {


  //  delay(2000);
  //  Serial.print("L SIDE :");
  //  Serial.println(count1);
  //  Serial.print("R SIDE :");
  //
  //  Serial.println(count2);
}

//=========================================================================================================
//======================================== INTERRUPTS / FUNCTIONS =========================================
//=========================================================================================================

/*
   ISR from Left encoder chA
   channel A - blue wire
   channel B - red wire
   GND - yellow wire
   When moving forward, channel A goes HIGH, channel B is HIGH
   When moving backward, channel A goes HIGH, channel B is LOW
*/

void encoderL()
{
  count1++;

//  if (digitalRead(encoderLBpin))
//    Serial.println("forward 1"); // count1++;
//  else
//    Serial.println("backward 1"); // count1--;
}


/*
   ISR from Right encoder chA
   channel A - yellow wire
   channel B - orange wire
   GND - green wire
   When moving forward, channel A goes HIGH, channel B is HIGH
   When moving backward, channel A goes HIGH, channel B is LOW
*/

void encoderR()
{
  count2++;
//  if (digitalRead(encoderRBpin))
//    Serial.println("forward 2"); // count1++;
//  else
//    Serial.println("backward 2"); //count1--;

}





/*

  void requestEvent() {

  String data; // buffer containing all data to send to ROS


  String str1 = String(count1);
  String str2 = String(count2);

  data = '<' + str1 + '-' + str2 + '-' + '>';

  Wire.write(data.c_str());

  //Serial.print(data);

  // reset counter values
  count1 = 0;
  count2 = 0;
  }
   /
*/
