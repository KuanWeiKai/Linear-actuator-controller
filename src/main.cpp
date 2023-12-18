#include <EEPROM_int.h>
#include <Encoder.h>
#include <TwoLinearActuator.h>
#include <Arduino.h>


// put function declarations here:

void clear_EEPROM();

//pins
#define motor_speed_1 7
#define motor_direction_1 6
#define motor_speed_2 9
#define motor_direction_2 8
#define encoder_1A 4
#define encoder_1B 2
#define encoder_2A 5
#define encoder_2B 3
#define reset_pin 10


// variables
TwoLinearActuator LA(motor_speed_1,motor_direction_1,encoder_1A,encoder_1B,motor_speed_2,motor_direction_2,encoder_2A,encoder_2B);
float serialInput;
const int eeAddress=0;
int prev_enc_count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting");
  pinMode(reset_pin,INPUT_PULLUP);
  /*
  prev_enc_count=readIntFromEEPROM(eeAddress);
  if (!(prev_enc_count>LA.max_count || prev_enc_count<=0)){
    LA.Enc1.write(prev_enc_count);
    LA.Enc2.write(prev_enc_count);
  }
  else{
    LA.reset_to_zero(); // move to home position, 0mm
  }
  */
  LA.reset_to_zero();
  LA.motor_moveto(50);
  Serial.print("Current postion:");
  Serial.println(LA.get_enc_dist());

}

void loop() {
  if (Serial.available()) {
      // read the incoming byte:
      serialInput = Serial.parseFloat();
      LA.motor_moveto(serialInput);
      writeIntIntoEEPROM(eeAddress,LA.Enc1.read());
      Serial.println(LA.Enc1.read()*LA.dist_per_count);
      Serial.println(LA.Enc2.read()*LA.dist_per_count);
    }
  if (digitalRead(reset_pin)==0){
    LA.reset_to_zero();
    clear_EEPROM();
  }
}

void clear_EEPROM(){
  for (unsigned i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}


