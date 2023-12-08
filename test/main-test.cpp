#include <Arduino.h>
#include <Encoder.h>


// put function declarations here:
void encoder_count_func_1();
void reset_to_zero();
void motor_moveto(double ref_dist);
void motor_move(int speed);
#define sgn(x) ({ __typeof__(x) _x = (x); _x < 0 ? -1 : _x ? 1 : 0; }) //signum function

//pins
#define motor_speed_1 6
#define motor_direction_1 7
#define encoder_1A 3
#define encoder_1B 4


// variables
volatile bool encoder_B_signal;
volatile signed int encoder_count=0;
double volatile encoder_dist=0;
int max_count=1300;
const double dist_per_count=(float)250/float(max_count); //250mm/1300count
float serialInput;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Beginning Test");
  pinMode(motor_speed_1,OUTPUT);
  pinMode(motor_speed_1,OUTPUT);
  pinMode(encoder_1A,INPUT);
  pinMode(encoder_1B,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_1A),encoder_count_func_1,FALLING);
  reset_to_zero();
  motor_move(200);
}

void loop() {
  Serial.println(encoder_count);
 

  
 if (Serial.available()) {
    // read the incoming byte:
    serialInput = Serial.parseFloat();
    motor_moveto(serialInput);
    Serial.println(encoder_dist);
  }

}

void motor_moveto(double ref_dist){ //move motor to position 0mm to 250mm
  double prev_dist;
  unsigned long prev_millis=0;
  if (abs(encoder_dist-ref_dist)>0.5){
    if (encoder_dist<ref_dist){
      while (encoder_dist<ref_dist){
        motor_move(255);
        if((millis()-prev_millis)>=1000){
        prev_millis=millis();
        //Serial.print(encoder_dist);
        //Serial.print(",");
        //Serial.println(prev_dist);
        if (encoder_dist==prev_dist){
          analogWrite(motor_speed_1,0);
          encoder_count=max_count;
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
        motor_move(-255);
        if((millis()-prev_millis)>=1000){
        prev_millis=millis();
        //Serial.print(encoder_dist);
        //Serial.print(",");
        //Serial.println(prev_dist);
        if (encoder_dist==prev_dist){
          analogWrite(motor_speed_1,0);
          encoder_count=0;
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
      if (encoder_count==prev_count){
        analogWrite(motor_speed_1,0);
        encoder_count=0;
        Serial.print("actuator set to home ");
        delay(500);
        break;
      }
      else{
        prev_count=encoder_count;
      }
    }
  } 
}

void encoder_count_func_1(){ //interupt function to count encoder
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
   if (interrupt_time - last_interrupt_time > 10) 
  {
    encoder_B_signal=digitalRead(encoder_1B);
    //Serial.println(millis());
    if (encoder_B_signal==0){
      encoder_count++;
    }
    else if(encoder_B_signal==1){
      encoder_count--;
    }
    }
  //Serial.print(encoder_count);
  //Serial.print(",");
  //Serial.print(digitalRead(encoder_1A));
  //Serial.print(",");
  //Serial.print(encoder_B_signal);
  //Serial.print(",");
  //Serial.println(millis());
  //encoder_dist=(encoder_count)*dist_per_count;
  last_interrupt_time = interrupt_time;
}

