#include <Arduino.h>
#include <Encoder.h>

class TwoLinearActuator{
    private:
        byte motor_speed_pin1;
        byte motor_direction_pin1;
        byte encoder_A_pin1;
        byte encoder_B_pin1;
        byte motor_speed_pin2;
        byte motor_direction_pin2;
        byte encoder_A_pin2;
        byte encoder_B_pin2;

        int sgn(int x){
            if (x<0){
                return -1;
            }
            else{
                return 1;
            }
        }

    public:
        Encoder Enc1;
        Encoder Enc2;
        int max_count;
        double encoder_dist1;
        double encoder_dist2;
        double max_dist;
        double dist_per_count=max_dist/double(max_count);
        TwoLinearActuator(byte motor_speed_pin1,byte motor_direction_pin1,byte encoder_A_pin1, byte encoder_B_pin1,
                          byte motor_speed_pin2,byte motor_direction_pin2,byte encoder_A_pin2, byte encoder_B_pin2,
                          int max_count,double max_dist);

        void init(){
            pinMode(motor_speed_pin1,OUTPUT);
            pinMode(motor_direction_pin1,OUTPUT);
            pinMode(motor_speed_pin2,OUTPUT);
            pinMode(motor_direction_pin2,OUTPUT);
        }

        void motor_moveto(double ref_dist);
        void reset_to_zero();
        void motor_move(int speed, byte motor_number){ //speed is 0-255,negative to indicate retraction
            static byte motor_direction_pin;
            static byte motor_speed_pin;
            if (motor_number==1){
              motor_direction_pin=motor_direction_pin1;
              motor_speed_pin=motor_speed_pin1;
            }
            else{
              motor_direction_pin=motor_direction_pin2;
              motor_speed_pin=motor_speed_pin2;
            }
            if (sgn(speed)==1){
                digitalWrite(motor_direction_pin,LOW);
            }
            else{
                digitalWrite(motor_direction_pin,HIGH);
            }
            analogWrite(motor_speed_pin,abs(speed));
        }
        double* get_enc_dist();
        };

TwoLinearActuator::TwoLinearActuator(byte motor_speed_pin1,byte motor_direction_pin1,byte encoder_A_pin1, byte encoder_B_pin1,
                                    byte motor_speed_pin2,byte motor_direction_pin2,byte encoder_A_pin2, byte encoder_B_pin2,int max_count,double max_dist):
Enc1(encoder_B_pin1,encoder_A_pin1),Enc2(encoder_B_pin2,encoder_A_pin2){
  this->motor_speed_pin1=motor_speed_pin1;
  this->motor_direction_pin1=motor_direction_pin1;
  this->encoder_A_pin1=encoder_A_pin1;
  this->encoder_B_pin1=encoder_B_pin1;
  this->motor_speed_pin2=motor_speed_pin2;
  this->motor_direction_pin2=motor_direction_pin2;
  this->encoder_A_pin2=encoder_A_pin2;
  this->encoder_B_pin2=encoder_B_pin2;
  this->max_count=max_count;
  this->max_dist=max_dist;
  this->dist_per_count = max_dist / double(max_count);
  TwoLinearActuator::init();         
}

void TwoLinearActuator::reset_to_zero(){
            bool home1=1,home2=1;
            long prev_count1=0, prev_count2=0;
            unsigned long prev_millis=0;
            prev_millis=millis();
            digitalWrite(motor_direction_pin1,HIGH);
            digitalWrite(motor_direction_pin2,HIGH);
            analogWrite(motor_speed_pin1,255);
            analogWrite(motor_speed_pin2,255);
            while (true){
                if((millis()-prev_millis)>=1000){
                  prev_millis=millis();
                  if (Enc1.read()==prev_count1){
                      analogWrite(motor_speed_pin1,0);
                      Enc1.write(0);
                      home1=0;
                  }
                  if (Enc2.read()==prev_count2){
                      analogWrite(motor_speed_pin2,0);
                      Enc2.write(0);
                      home2=0;
                  }
                else{
                    prev_count1=Enc1.read();
                    prev_count2=Enc2.read();
                }
                if(home1==0 && home2==0){
                  break;;
                }
            } 
        }
}
void TwoLinearActuator::motor_moveto(double ref_dist){ //move motor to position 0mm to 250mm
            double prev_dist=-999;
            int motor_speed=255;
            static unsigned long prev_millis=0;
            encoder_dist1=Enc1.read()*dist_per_count;
            encoder_dist2=Enc2.read()*dist_per_count;
            //check if 2 actuator are same position
            while (abs(encoder_dist1-encoder_dist2)>0.2){
              motor_move(sgn(encoder_dist1-encoder_dist2)*100,2);
            }
            motor_move(0,2);
            //Move both actuator
            if (abs(encoder_dist1-ref_dist)>0.2){
              if (encoder_dist1<ref_dist){
                while (encoder_dist1<ref_dist){
                  motor_move(motor_speed,1);
                  motor_move(motor_speed,2);
                  encoder_dist1=Enc1.read()*dist_per_count;
                  encoder_dist2=Enc2.read()*dist_per_count;
                  if((millis()-prev_millis)>=1000){
                  prev_millis=millis();
                  //Serial.print(encoder_dist);
                  //Serial.print(",");
                  //Serial.println(prev_dist);
                  if (encoder_dist1==prev_dist){
                    analogWrite(motor_speed_pin1,0);
                    analogWrite(motor_speed_pin2,0);
                    Enc1.write(max_count);
                    Enc2.write(max_count);
                    encoder_dist1=max_dist;
                    encoder_dist2=max_dist;
                    Serial.println("reached far limit");
                    break;
                  }
                  else{
                    prev_dist=encoder_dist1;
                  }
              }
                  //Serial.println(encoder_dist);
                }
              }
              else if (encoder_dist1>ref_dist){
                  while (encoder_dist1>ref_dist){
                  motor_move(-1*motor_speed,1);
                  motor_move(-1*motor_speed,2);
                  encoder_dist1=Enc1.read()*dist_per_count;
                  encoder_dist2=Enc2.read()*dist_per_count;
                  if((millis()-prev_millis)>=1000){
                  prev_millis=millis();
                  //Serial.print(encoder_dist);
                  //Serial.print(",");
                  //Serial.println(prev_dist);
                  if (encoder_dist1==prev_dist){
                    analogWrite(motor_speed_pin1,0);
                    analogWrite(motor_speed_pin2,0);
                    Enc1.write(0);
                    Enc2.write(0);
                    encoder_dist1=0.0,encoder_dist2=0.0;
                    Serial.println("reached close limit");
                    break;
                  }
                  else{
                    prev_dist=encoder_dist1;
                  }
              }
                  //Serial.println(encoder_dist);
                }
              }
            }
            motor_move(0,1);
            motor_move(0,2);
          }

double* TwoLinearActuator::get_enc_dist(){
  static double enc_dist[2]={(Enc1.read()*dist_per_count),(Enc2.read()*dist_per_count)};
  return enc_dist;
}

           
