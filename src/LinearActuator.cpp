#include <Arduino.h>
#include <Encoder.h>

class LinearActuator{
    private:
        byte motor_speed_pin;
        byte motor_direction_pin;
        byte encoder_A_pin;
        byte encoder_B_pin;

        int sgn(int x){
            if (x<0){
                return -1;
            }
            else{
                return 1;
            }
        }

    public:
        Encoder Enc;
        int max_count;
        double encoder_dist;
        double max_dist;
        double dist_per_count=max_dist/double(max_count);
        LinearActuator(byte motor_speed_pin,byte motor_direction_pin,byte encoder_A_pin, byte encoder_B_pin,int max_count,
        double max_dist);

        void init(){
            pinMode(motor_speed_pin,OUTPUT);
            pinMode(motor_direction_pin,OUTPUT);
        }

        void motor_moveto(double ref_speed);
        void reset_to_zero();
        void motor_move(int speed){ //speed is 0-255,negative to indicate retraction
            if (sgn(speed)==1){
                digitalWrite(motor_direction_pin,LOW);
            }
            else{
                digitalWrite(motor_direction_pin,HIGH);
            }
            analogWrite(motor_speed_pin,abs(speed));
        }
        double get_enc_dist();
        };

LinearActuator::LinearActuator(byte motor_speed_pin,byte motor_direction_pin,byte encoder_A_pin, byte encoder_B_pin,int max_count,double max_dist):
Enc(encoder_B_pin,encoder_A_pin){
  this->motor_speed_pin=motor_speed_pin;
  this->motor_direction_pin=motor_direction_pin;
  this->encoder_A_pin=encoder_A_pin;
  this->encoder_B_pin=encoder_B_pin;
  this->max_count=max_count;
  this->max_dist=max_dist;
  this->dist_per_count = max_dist / double(max_count);
  LinearActuator::init();         
}

void LinearActuator::reset_to_zero(){
            long prev_count=0;
            unsigned long prev_millis=0;
            prev_millis=millis();
            digitalWrite(motor_direction_pin,HIGH);
            analogWrite(motor_speed_pin,255);
            while (true){
                if((millis()-prev_millis)>=1000){
                prev_millis=millis();
                if (Enc.read()==prev_count){
                    analogWrite(motor_speed_pin,0);
                    Enc.write(0);
                    Serial.println("actuator set to home ");
                    delay(500);
                    break;
                }
                else{
                    prev_count=Enc.read();
                }
                }
            } 
        }
void LinearActuator::motor_moveto(double ref_dist){ //move motor to position 0mm to 250mm
            double prev_dist=-999;
            int motor_speed=200;
            static unsigned long prev_millis=0;
            encoder_dist=Enc.read()*dist_per_count;
            if (abs(encoder_dist-ref_dist)>0.2){
              if (encoder_dist<ref_dist){
                while (encoder_dist<ref_dist){
                  motor_move(motor_speed);
                  encoder_dist=Enc.read()*dist_per_count;
                  if((millis()-prev_millis)>=1000){
                  prev_millis=millis();
                  //Serial.print(encoder_dist);
                  //Serial.print(",");
                  //Serial.println(prev_dist);
                  if (encoder_dist==prev_dist){
                    analogWrite(motor_speed_pin,0);
                    Enc.write(max_count);
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
                  encoder_dist=Enc.read()*dist_per_count;
                  if((millis()-prev_millis)>=1000){
                  prev_millis=millis();
                  //Serial.print(encoder_dist);
                  //Serial.print(",");
                  //Serial.println(prev_dist);
                  if (encoder_dist==prev_dist){
                    analogWrite(motor_speed_pin,0);
                    Enc.write(0);
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
double LinearActuator::get_enc_dist(){
          return (Enc.read()*dist_per_count);
        }

           
