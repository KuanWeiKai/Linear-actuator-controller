#ifndef LinearActuator_h
#define LinearActuator_h

#include <Arduino.h>
#include <Encoder.h>

class TwoLinearActuator{
    private:
    byte motor_speed_pin;
    byte motor_direction_pin;
    byte encoder_A_pin;
    byte encoder_B_pin;
    

    int sgn(int x);

    public:
    Encoder Enc1;
    Encoder Enc2;
    int max_count;
    double encoder_dist;
    double max_dist;
    double dist_per_count;

    TwoLinearActuator(byte motor_speed_pin1,byte motor_direction_pin1,byte encoder_A_pin1, byte encoder_B_pin1,
                    byte motor_speed_pin2,byte motor_direction_pin2,byte encoder_A_pin2, byte encoder_B_pin2,
                    int max_count=5330,double max_dist=250.0);
    

    void reset_to_zero();
    void init();
    void motor_move(int speed);
    void motor_moveto(double ref_dist);
    double get_enc_dist();

};


#endif