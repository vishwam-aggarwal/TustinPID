#ifndef TUSTINPID_H
#define TUSTINPID_H

class TustinPID
{
public:
    void setup(float Kp, float Ki, float Kd, float tau, float Ts);
    void setStates(float state_int, float state_der);
    void reset();

    float getControl(float e, float uMin, float uMax);
    float getControl(float e);

private:
    float getP(float e);
    float getI(float e);                    // pure Tustin integrator
    float getD(float e);

    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float tau = 0;
    float Ts = 0;

    float prev_e = 0;

    float i_state = 0;
    float i_state_prev = 0;

    float d_state = 0;
    float d_state_prev = 0;
};

#endif