#include <EEPROM_int.h>
#include <Encoder.h>


// put function declarations here:
void encoder_count_func_1();
void reset_to_zero();
void motor_moveto(double ref_dist);
void motor_move(int speed);
void clear_EEPROM();
#define sgn(x) ({ __typeof__(x) _x = (x); _x < 0 ? -1 : _x ? 1 : 0; }) //signum function

//pins
#define motor_speed_1 6
#define motor_direction_1 7
#define encoder_1A 3
#define encoder_1B 4
#define reset_pin 8


// variables
Encoder Enc1(3,4);
double volatile encoder_dist=0;
int max_count=5320;
const double dist_per_count=(float)250/float(max_count); //250mm/max count
float serialInput;
const int eeAddress=0;
int prev_enc_count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(motor_speed_1,OUTPUT);
  pinMode(motor_speed_1,OUTPUT);
  //pinMode(encoder_1A,INPUT);
  //pinMode(encoder_1B,INPUT);
  pinMode(reset_pin,INPUT_PULLUP);
  prev_enc_count=readIntFromEEPROM(eeAddress);
  if (prev_enc_count>max_count || prev_enc_count<=0){
  reset_to_zero(); // move to home position, 0mm
  }
  else{
    Enc1.write(prev_enc_count);
    encoder_dist=Enc1.read()*dist_per_count;
  }
  Serial.print("Current postion:");
  Serial.println(encoder_dist);
  //motor_move(150);
}

void loop() {
 //Serial.println(Enc1.read());

  
 if (Serial.available()) {
    // read the incoming byte:
    serialInput = Serial.parseFloat();
    motor_moveto(serialInput);
    writeIntIntoEEPROM(eeAddress,Enc1.read());
    Serial.println(Enc1.read()*dist_per_count);
  }
if (digitalRead(reset_pin)==0){
  reset_to_zero();
  clear_EEPROM();
}

}

void motor_moveto(double ref_dist){ //move motor to position 0mm to 250mm
  double prev_dist=-999;
  int motor_speed=200;
  static unsigned long prev_millis=0;
  encoder_dist=Enc1.read()*dist_per_count;
  if (abs(encoder_dist-ref_dist)>0.2){
    if (encoder_dist<ref_dist){
      while (encoder_dist<ref_dist){
        motor_move(motor_speed);
        encoder_dist=Enc1.read()*dist_per_count;
        if((millis()-prev_millis)>=1000){
        prev_millis=millis();
        //Serial.print(encoder_dist);
        //Serial.print(",");
        //Serial.println(prev_dist);
        if (encoder_dist==prev_dist){
          analogWrite(motor_speed_1,0);
          Enc1.write(max_count);
          encoder_dist=250.0;
          Serial.println("reached far limit");
          break;
        }
        else{
          prev_dist=encoder_dist;
        }
    }
        //Serial.println(encoder_dist);
      }
    }
    else if (encoder_dist>ref_dist){
        while (encoder_dist>ref_dist){
        motor_move(-1*motor_speed);
        encoder_dist=Enc1.read()*dist_per_count;
        if((millis()-prev_millis)>=1000){
        prev_millis=millis();
        //Serial.print(encoder_dist);
        //Serial.print(",");
        //Serial.println(prev_dist);
        if (encoder_dist==prev_dist){
          analogWrite(motor_speed_1,0);
          Enc1.write(0);
          encoder_dist=0.0;
          Serial.println("reached close limit");
          break;
        }
        else{
          prev_dist=encoder_dist;
        }
    }
        //Serial.println(encoder_dist);
      }
    }
  }
  motor_move(0);
}

void motor_move(int speed){ //speed is 0-255,negative to indicate retraction
  if (sgn(speed)==1){
    digitalWrite(motor_direction_1,LOW);
  }
  else{
    digitalWrite(motor_direction_1,HIGH);
  }
  analogWrite(motor_speed_1,abs(speed));
}

void reset_to_zero(){ //reset motor to 0mm position
  long prev_count=0;
  unsigned long prev_millis=0;
  prev_millis=millis();
  digitalWrite(motor_direction_1,HIGH);
  analogWrite(motor_speed_1,255);
  while (true){
    if((millis()-prev_millis)>=1000){
      prev_millis=millis();
      if (Enc1.read()==prev_count){
        analogWrite(motor_speed_1,0);
        Enc1.write(0);
        Serial.println("actuator set to home ");
        delay(500);
        break;
      }
      else{
        prev_count=Enc1.read();
      }
    }
  } 
}

void clear_EEPROM(){
  for (unsigned i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}


