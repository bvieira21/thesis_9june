/*
  ROBUTER MOTOR AND BRAKES ACTUATION

  Ethernet server that communicates with main coputer unit, running ROS
  and actuate robuter motor's and brakes

  This interface responds to commands like:
  - DT: request for RPM's and brakestate
  - BK'x': actuate brake (BK1 disables brakes, 48V, to allow rotation. BK0 enables brakes)
  - SP1'x': actuate motor 1 to 'x' RPM (RPM validation is made in GTK software)
  - SP2'x': actuate motor 2 to 'x' RPM (RPM validation is made in GTK software)

  created Mar 2017
  by Bruno Vieira
  in Universidade de Aveiro

*/

//=========================================================================================================
//=============================================== LIBRARIES ===============================================
//=========================================================================================================

#include <Ethernet2.h>
#include <Wire.h>
#include <TimerOne.h>
#include <MsTimer2.h>


//=========================================================================================================
//========================================== VARIABLES / DEFINES ==========================================
//=========================================================================================================

//I2C slave #
#define ENCODER_SLAVE 1

//Ethernet variables
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xBC, 0xB8};
IPAddress ip(169, 254, 0, 50);
EthernetServer server(50000);
EthernetClient client;


//===PID CONTROL====
int pwm1 = 128, pwm2 = 128;
uint16_t pulse_setpoint1 = 0, pulse_setpoint2 = 0, enc1_read = 0, enc2_read = 0;
byte t[4]; // used to store the encoder values transmitted via I2C, after they will be concatenated

volatile float kP = 0.12;
volatile float kI = 0.0; //0.05; //kI = 0.001
volatile float kD = 0.0; //0.06; //kd = 2.5
volatile int I_term1 = 0, I_term2 = 0;
volatile int new_PWM1 = 0, new_PWM2 = 0, old_PWM1 = 0, old_PWM2 = 0;
volatile int lastInput1 = 0, lastInput2 = 0;

float prevTime=millis();

//Error handling
int msg_count = 0;


// VALUES - used to set some system defines, like frequency of the messages
#define MSG_FREQ 10 // message frequency, used to detect error/failure in transmission
#define MAX_PULSE 10000   // maximum speed, in pulses
#define MAX_PWM 128   // maximum PWM value


//IO's - falta completar descricao, HIGH, LOW para bumper e EMR e niveis para baterias
#define AI_B_LVL        0       // Analog Input - Monitors battery level
#define DI_EMG_STATE    11      // Digital Input - Emergency state (Detects emergency and collision with bumpers)
#define DO_BRAKES       7       // Digital Output - Relay module to activate eletromagnetic brakes
#define DO_B_R          12      // Digital Output - Recharge battery LED
#define DO_B_OK         13      // Digital Output - Battery OK LED
#define AO_PWM1         9       // Analog Output - drive 1 REF + ( 2nd top left) - Left motor
#define AO_PWM2         5       // Analog Output - drive 2 REF + ( 2nd top left) - Right motor
// SCL and SDA pin used for I2C communication

//=========================================================================================================
//========================================== FUNCTION PROTOTYPES ==========================================
//=========================================================================================================

String parse_encoderDATA(void);
bool process_reception(String rcv);
void setPWM (int pwmL, int pwmR, char op);
bool rot_motor(char pin, char dir, char vel);
void comm_lost (void);
void PID_control(uint16_t pulse_setpoint1, uint16_t pulse_setpoint2, char op, uint16_t enc1_read, uint16_t enc2_read);

//=========================================================================================================
//================================================= SETUP =================================================
//=========================================================================================================

void setup() {

  //change PWM frequencies
  TCCR3B = TCCR3B & 0b11111000 | 0x01; // pin 5 32kHz
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // pin 9 32kHz



  // ============= INITIALIZATIONS ==============

  pinMode(AO_PWM1, OUTPUT);
  pinMode(AO_PWM2, OUTPUT);
  pinMode(DO_BRAKES, OUTPUT);
  pinMode(DO_B_R, OUTPUT);
  pinMode(DO_B_OK, OUTPUT);

  digitalWrite(DO_BRAKES, LOW); // unbreak system
  digitalWrite(DO_B_R, HIGH); // LED OFF
  digitalWrite(DO_B_OK, HIGH); // LED OFF


  analogWrite(AO_PWM1, 128); // zero value
  analogWrite(AO_PWM2, 128); // zero value


  delay(50);

  //start I2C module as master
  Wire.begin();
  delay(50);


  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  //while (!Serial);


  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  delay(50);


  // Timer 1 conflicts with PWM pins, this way doesn't
  MsTimer2::set(500, timer2_overflow); // 500ms period
  MsTimer2::start();
  //
  //  Timer1.initialize(1000000);             //Inicializa timer1 com tempo de contagem de 100ms
  //  Timer1.attachInterrupt(timer1_overflow);
}


//=========================================================================================================
//============================================== MAIN LOOP ================================================
//=========================================================================================================

void loop() {

  // listen for incoming clients
  client = server.available();
  String rcv;  // received buffer from C

  if (client) {
    Serial.println("new client");
    

    while (client.connected()) {
      if (client.available()) {

        char c = client.read();

        //checks if is valid char, otherwise will not process it
        if (c > 10 && c < 127 )
        {
          //end of message char
          if (c != '>') {
            rcv += c;
            client.println("X"); // dummy value to mantain connection
            continue;
          }
        }
        else {
          client.println("WARNING: Invalid char passed");
          //Serial.print("WARNING: Invalid char passed");
          rcv = ""; // empty buffer
          continue;
        }

        //Serial.print(rcv);
        // ================== REQUEST ARDUINO DATA =====================
        if (!process_reception(rcv))
          client.println("WARNING: Invalid message, not processed");
        else
          client.println("PASSED HERE!");

        rcv = ""; // empty buffer

      }
    }
    // give the web browser time to receive the data
    delay(0.01);

    // close the connection:
    client.stop();
    Serial.println("client disconnected");

    comm_lost(); // actuate brakes and stop motors
  }
}


//=========================================================================================================
//======================================== INTERRUPTS / FUNCTIONS =========================================
//=========================================================================================================

/*
   @brief: processes income data to actuate brakes and motors. Example messages in ROS: "BK0>"; "MMF30-B60"; "EN>"
   @param: rcv - received string to be processed
   @return: TRUE in case of valid and processed message
*/
bool process_reception(String rcv)
{
  String aux1;
  String aux2;
  char op = 0;

  // ======================== ENCODER DATA ========================
  if (rcv.substring(0, 2) == "EN") // message to request for encoder data
  {
    parse_encoderDATA();

    //sends acknowledge message informing it's the operation is DONE
    client.println("ACK-EN");

    return true;
  }
  // ====================== ACTUATE BRAKE =======================
  if (rcv.substring(0, 2) == "BK") // message to process brake actuation
  {
    if (rcv.substring(2, 3) == "1")
      digitalWrite(DO_BRAKES, HIGH); // unbraked
    else
      digitalWrite(DO_BRAKES, LOW);  // braked

    msg_count++;

    //sends acknowledge message informing it's the operation is DONE
    client.println("ACK-BK");
    client.println(msg_count);


    return true;
  }

  // ================== ACTUATE MOTOR 1 (rpm values) ====================
  if (rcv.substring(0, 2) == "MM")
  {

    aux1 = rcv.substring(3, rcv.indexOf('-'));

    aux2 = rcv.substring(rcv.indexOf('-') + 2, rcv.length());

    pulse_setpoint1 = (uint16_t)aux1.toInt();
    pulse_setpoint2 = (uint16_t)aux2.toInt();


    if (rcv.substring(2, 3) == "F") //forward way
      op += 1; //op = op || 0b00000001;
    else
      op += 2; //op = op || 0b00000010;;

    if (rcv.substring(rcv.indexOf('-') + 1, rcv.indexOf('-') + 2) == "F")
      op += 4; //op = op || 0b00000100;
    else
      op += 8; //op = op || 0b00001000;

    // acquisition of encoder data before PID
    parse_encoderDATA();


    if ( (pulse_setpoint1 >= 0  && pulse_setpoint1 < MAX_PULSE) &&
         (pulse_setpoint2 >= 0  && pulse_setpoint2 < MAX_PULSE))
    {
      PID_control(pulse_setpoint1, pulse_setpoint2, op, enc1_read, enc2_read);
    }


    //sends acknowledge message informing it's the operation is DONE
    client.println("ACK-SP");
    return true;
  }

  // message not processed - INVALID
  return false;
}



/*
   @brief: builds string with Left and right encoder pulses
           FORMAT: < Pulses Count Left - Pulses Count Right ->
   @return: data - string in above FORMAT
*/
String parse_encoderDATA (void) {

  String data;
  byte index = 0;

  Serial.print("Before Req");
  Wire.requestFrom(ENCODER_SLAVE, 4); // request to ENCODER_SLAVE, device #1
  Serial.print("After Req");
  while (Wire.available())   // slave may send less than requested
  {
    t[index] = Wire.read();
    index++;
  }

  enc1_read = t[0];
  enc1_read = (enc1_read << 8) | t[1];


  enc2_read = t[3];
  enc2_read = (enc2_read << 8) | t[2];

  /*
    client.println("-------------");
    client.println(enc1_read);
    client.println(enc2_read);
    client.println("-------------");

    Serial.println("-------------");
    Serial.println(enc1_read);
    Serial.println(enc2_read);
    Serial.println("-------------");
  */

  //Serial.print(data);
  //client.println(data);
  return "0";
}



/*
   @brief: sets the PWM in both wheels with given direction
   @param:
*/
void setPWM (int pwm1, int pwm2, char op) {

  Serial.print("op->");
  Serial.println(op, DEC);


  //Left motor
  analogWrite(AO_PWM1, 128); //analogWrite(AO_PWM1, pwm1);
  //Right motor
  analogWrite(AO_PWM2, 128); //analogWrite(AO_PWM2, pwm2);


  Serial.println("ERROR: Couldn't set PWM values");

}


/*
   @brief: Stops the motors and actuates brake in case of disconnection
   @param:
*/
void comm_lost (void) {

  rot_motor(AO_PWM1, 'S', 0);
  rot_motor(AO_PWM2, 'S', 0);
  digitalWrite(DO_BRAKES, HIGH); // break system
}


/*
   @brief: used to make velocity input standard and facilitate the PID algorithm
   @param:
   pin - where the PWM wave will be applied
   dir - "F" Forward; "B" Backward; "S" Stop
   vel - desired velocity on scale from 0 to 128
   @return: TRUE in case of valid and processed message
*/
bool rot_motor(char pin, char dir, char vel) {

  if (vel > 128 && vel < 0)
  {
    client.print("Invalid function call"); // replace for client.print
    return false;
  }
  else
  {
    switch (dir)
    {
      case 'B': analogWrite(pin, 128 - vel);
        return true;
        break;
      case 'F': analogWrite(pin, 127 + vel);
        return true;
        break;
      case 'S': analogWrite(pin, 128);
        return true;
        break;
      default: Serial.print("Invalid option for direction");
        return false;
    }
  }
}


/*
   @brief: timer to monitor communications, disable motors in case of insuficient messages received
*/
void timer2_overflow() {

  if (msg_count < MSG_FREQ)
    //Serial.println("->>   Falhou");
    //else
    //Serial.println("->>   Resultou");
    msg_count = 0;

}

/*
   @brief: sets the PWM in both wheels with given direction
   @param:
*/
void PID_control( uint16_t sp1, uint16_t sp2, char op, uint16_t read1, uint16_t read2)
{
  int error1, error2;
  int P_term1, D_term1, P_term2, D_term2;

  //time calculation
  float deltaT=(millis()-prevTime)/1000;
  prevTime=millis();

  // ========= MOTOR 1 =========
  //error parcels calculation
  error1 = sp1 - read1;
  P_term1 = kP * error1;
  I_term1 += kI * error1 * deltaT;
  D_term1 = ((read1 - lastInput1)/deltaT) * kD;

  lastInput1 = read1; //stores this value to next function call

  // new PWM calculation
  new_PWM1 = old_PWM1 + P_term1 + I_term1 + D_term1;
  //new_PWM=P_Term+I_Term - Kd * dInput;

  //PWM limitation
  if (new_PWM1 > MAX_PWM) {
    new_PWM1 = MAX_PWM;
  } else if (new_PWM1 < 0) {
    new_PWM1 = 0;
  }

  old_PWM1 = new_PWM1; //stores this value to next function call



  // ========= MOTOR 2 =========
  //error parcels calculation
  error2 = sp2 - read2;
  P_term2 = kP * error2;
  I_term2 += kI * error1;
  D_term2 = (read2 - lastInput2);

  lastInput2 = read2; //stores this value to next function call

  // new PWM calculation
  new_PWM2 = old_PWM2 + P_term2 + I_term2 - kD * D_term2;
  //new_PWM=P_Term+I_Term - Kd * dInput;

  //PWM limitation
  if (new_PWM2 > MAX_PWM) {
    new_PWM2 = MAX_PWM;
  } else if (new_PWM2 < 0) {
    new_PWM2 = 0;
  }

  old_PWM2 = new_PWM2; //stores this value to next function call


  client.println("------1-------");
  client.print(enc1_read); client.print(" - SP1 "); client.print(sp1); client.print(" - err1 "); client.println(error1);
  client.println("------2-------");
  client.print(enc2_read); client.print(" - SP2 "); client.print(sp2); client.print(" - err2 "); client.println(error2);


  //REMOVE THIS LINES AFTER PID IS IMPLEMENTED
  
//    new_PWM1 = sp1;
//    new_PWM2 = sp2;
//
//    client.print("sp1 - ");
//    client.print(new_PWM1); client.print(" | enc1 - ");
//    client.println(enc1_read);
//
//    client.print("sp2 - ");
//    client.print(new_PWM2);client.print(" | enc2 - ");
//    client.println(enc2_read);
  
  switch (op) {
    case 5: // 0b00000101
      client.println("ML F, MR F");
      rot_motor(AO_PWM1, 'F', new_PWM1);
      rot_motor(AO_PWM2, 'F', new_PWM2);
      break;
    case 9: // 0b00001001
      client.println("ML F, MR B");
      rot_motor(AO_PWM1, 'F', new_PWM1);
      rot_motor(AO_PWM2, 'B', new_PWM2);
      break;
    case 6: // 0b00000110
      client.println("ML B, MR F");
      rot_motor(AO_PWM1, 'B', new_PWM1);
      rot_motor(AO_PWM2, 'F', new_PWM2);
      break;
    case 10: // 0b000001010
      client.println("ML B, MR B");
      rot_motor(AO_PWM1, 'B', new_PWM1);
      rot_motor(AO_PWM2, 'B', new_PWM2);
      break;
    default: Serial.println("ERROR: Couldn't set PWM values");
  }


}




// ===================================================================================================
// ==================================== DEBUG SECTION ================================================
// ===================================================================================================


//                Serial.print("\nQ='");
//                Serial.print(c, DEC);
//                Serial.print("*");
//                Serial.print(c);
//                Serial.println("'");

//debug
//        Serial.print("R->");
//        Serial.print(c);
//        Serial.println("<-");
//debug
//        Serial.print("RCV->");
//        Serial.print(rcv);
//        Serial.println("<-");
