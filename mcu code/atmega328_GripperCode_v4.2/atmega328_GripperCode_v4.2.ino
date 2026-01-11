#include <Servo.h>
#include <Wire.h>

Servo myServo;
/////////////////pin setup////////////////////
//special pins                              //
const int servoPin = 6;                     //
const int fsrPin = A1;                      //
//inputs                                    //
const int I_OPEN = 7;                       //
const int I_CLOSE = 8;                      //
const int I_ESTOP = 9;                      //  
const int I_REST = 10;                      //
//outputs                                   //
const int O_BIT1 = 2;                       //
const int O_BIT2 = 3;                       //
const int O_CONFIRM = 4;                    //
const int O_ERROR = 5;                      //
//////////////////////////////////////////////

////////////////sensor value//////////////////
//expected value small 163                  //
const int value_Small_max = 180;            //
const int value_Small_min = 141;            //
                                            //
//expected value small 133                  //
const int value_Medium_max = 140;           //
const int value_Medium_min = 106;           //
                                            //
//expected value small  97                  //
const int value_Big_max = 105;              //
                              //97          //
const int value_Big_min = 0;                //
//////////////////////////////////////////////

int valueFSR = 50000; // initinalsing fsr to prevent problems with visualization of force
bool Flag = LOW;      // FLag to avoid duplicate execution

const int drempel[] = { 1024, 992, 960, 928, 896, 864, 832, 800,};

enum State { IDLE, CLOSE, OPEN, ESTOP };
State currentState = IDLE;

void setup() {
  myServo.attach(servoPin); // setup servo pin
  Serial.begin(9600);       // setup rx tx comunication for debuging
  
  Wire.begin();             // IC2 setup

  
  // INPUTS (robot → MCU)
  pinMode(I_OPEN, INPUT_PULLUP);
  pinMode(I_CLOSE, INPUT_PULLUP);
  pinMode(I_ESTOP, INPUT_PULLUP);
  pinMode(I_REST, INPUT_PULLUP);

    // OUTPUTS (MCU → robot)
  pinMode(O_BIT1, OUTPUT);
  pinMode(O_BIT2, OUTPUT);
  pinMode(O_CONFIRM, OUTPUT);
  pinMode(O_ERROR, OUTPUT);

}

void loop() {

  delay(100);
  Serial.print("state: ");
  Serial.println(currentState); // debug statment  
  // force display
  valueFSR = analogRead(fsrPin);      
  display_force(valueFSR);  
    
  switch (currentState) {

    case IDLE:
      display_force(valueFSR);


      if (Flag == LOW) {              // to avoid duplicate execution
        if (digitalRead(I_OPEN) == LOW) {       // statment to start open ( priority to open)
          Serial.println("→ knop OPEN ingedrukt");
          currentState = OPEN;
          Flag = HIGH;
        } 
        else if (digitalRead(I_CLOSE) == LOW) {   // statment to start close
          Serial.println("→ knop CLOSE ingedrukt");
          currentState = CLOSE;
          Flag = HIGH;
        }
      } 
      else if (digitalRead(I_CLOSE) == HIGH && digitalRead(I_OPEN) == HIGH) {  // reset flag
        Flag = LOW;
      }
      break;

    case CLOSE:
/////////////// Servo Movement /////////////////
// Loop moves servo from min to max position  //
// Closes from 10 deg (open) to 180 (close).  //
////////////////////////////////////////////////
      for (int pos = 10; pos <= 180; pos += 1) { // moving servo from min pos to max pos

        valueFSR = analogRead(fsrPin);            // Read pressure sensor
        delay(50);                                // Slow down the closing speed
//////////////// Detection Logic ///////////////////////////////////////////////
// Stop if sensor feels pressure (< 800)                                      //
// OR if servo reaches maximum limit (180).                                   //
                                                                              //
        if (valueFSR < 800 || pos >= 180) {                                   //
          Serial.println(pos);                                                //
                                                                              //                                                                          
                                                                              //
///////////////// Size Determination ///////////////////////////////////////  //
// Checks the size of the object based on                                 //  //
// the servo position 'pos' at impact.                                    //  //
                                                                          //  //
          if (pos >= value_Big_min && pos <= value_Big_max) {             //  //
            Serial.println("OUTPUT → BIG");                               //  //
            digitalWrite(O_BIT1, LOW);                                    //  //
            digitalWrite(O_BIT2, HIGH);                                   //  //
            digitalWrite(O_CONFIRM, HIGH);                                //  //
            delay (100);                                                  //  //
            digitalWrite(O_CONFIRM, LOW);                                 //  //
          }                                                               //  //
          else if (pos >= value_Medium_min && pos <= value_Medium_max) {  //  //
            Serial.println("OUTPUT → MEDIUM");                            //  //
            digitalWrite(O_BIT1, HIGH);                                   //  //
            digitalWrite(O_BIT2, LOW);                                    //  //
            digitalWrite(O_CONFIRM, HIGH);                                //  //
            delay (100);                                                  //  //
            digitalWrite(O_CONFIRM, LOW);                                 //  //
          }                                                               //  //
          else if (pos >= value_Small_min && pos <= value_Small_max) {    //  //
            Serial.println("OUTPUT → SMALL");                             //  //
             digitalWrite(O_BIT1, LOW);                                   //  //  
            digitalWrite(O_BIT2, LOW);                                    //  //
            digitalWrite(O_CONFIRM, HIGH);                                //  //  
            delay (100);                                                  //  //
            digitalWrite(O_CONFIRM, LOW);                                 //  //
          }                                                               //  //
          else {                                                          //  //
            Serial.println("OUTPUT → ERROR");                             //  //
          // digitalWrite(O_BIT1, LOW);                                   //  //
            //digitalWrite(O_BIT2, LOW);                                  //  //
            //digitalWrite(O_CONFIRM, LOW);                               //  //
            //digitalWrite(O_ERROR, HIGH);                                //  //
          }                                                               //  //
////////////////////////////////////////////////////////////////////////////  //
          currentState = IDLE;      // reset to IDLE                          //
          break;    // Break out of the 'for' loop                            //
        }                                                                     //
////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
 //Only happens if nothing detected yet                         //
        display_force(valueFSR);                                //
        moveServo(pos);                                         //
        Serial.print("FSR: ");                                  //
        Serial.println(valueFSR);                               //
 /////////////////////////////////////////////////////////////////
      }
      break;   // end close

    case OPEN:
      moveServo(10); // Moves the servo back to the open position

      delay(500); // servo time
      // Output Signaling
      digitalWrite(O_BIT1, HIGH);
      digitalWrite(O_BIT2, HIGH);
      digitalWrite(O_CONFIRM, HIGH);
      delay (100);
        // reset Output Signaling
      digitalWrite(O_CONFIRM, LOW);
      //Serial.println("OUTPUT → OPEN"); // debug

      currentState = IDLE;
      break;
  }
}

// --- FUNCTIONS ---

void moveServo(int position) {
  myServo.write(position);
}
// Function that converts the force value into a byte and sends it to the I2C device at address 0x20
void display_force(int force_int) {
  byte output_byte = B11111111;
 
 // If the force is greater than or equal to drempel[i],
  for (int i = 0; i < 8; i++) {// Loop through all 8 threshold values
    if (force_int >= drempel[i]) {
      output_byte &= ~(1 << i);
      //Create a mask: (1 << i)
      //Invert the mask ~(1 << i)
      //Apply mask with Bitwise AND
    }
  }
// send output byte
  Wire.beginTransmission(0x20);
  Wire.write(output_byte);
  Wire.endTransmission();
}
