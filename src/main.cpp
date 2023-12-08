#include <EEPROM_int.h>
#include <Encoder.h>
#include <LinearActuator.h>


// put function declarations here:

void clear_EEPROM();

//pins
#define motor_speed_1 6
#define motor_direction_1 7
#define encoder_1A 4
#define encoder_1B 3
#define reset_pin 8


// variables
LinearActuator LA1(motor_speed_1,motor_direction_1,encoder_1A,encoder_1B);
float serialInput;
const int eeAddress=0;
int prev_enc_count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(reset_pin,INPUT_PULLUP);
  prev_enc_count=readIntFromEEPROM(eeAddress);
  if (!(prev_enc_count>LA1.max_count || prev_enc_count<=0)){
    LA1.Enc.write(prev_enc_count);
  }
  else{
    LA1.reset_to_zero(); // move to home position, 0mm
  }
  Serial.print("Current postion:");
  Serial.println(LA1.get_enc_dist());
  //motor_move(150);

}

void loop() {
 //Serial.println(Enc1.read());

  
 if (Serial.available()) {
    // read the incoming byte:
    serialInput = Serial.parseFloat();
    LA1.motor_moveto(serialInput);
    writeIntIntoEEPROM(eeAddress,LA1.Enc.read());
    Serial.println(LA1.Enc.read()*LA1.dist_per_count);
  }
if (digitalRead(reset_pin)==0){
  LA1.reset_to_zero();
  clear_EEPROM();
}
}

void clear_EEPROM(){
  for (unsigned i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}


