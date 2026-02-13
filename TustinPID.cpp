#include "TustinPID.h"

void TustinPID::setup(float Kp, float Ki, float Kd, float tau, float Ts)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tau = tau;
    this->Ts = Ts;

    reset();
}

void TustinPID::setStates(float state_int, float state_der)
{
    i_state = state_int;
    i_state_prev = state_int;

    d_state = state_der;
    d_state_prev = state_der;
}

void TustinPID::reset()
{
    prev_e = 0;

    i_state = 0;
    i_state_prev = 0;

    d_state = 0;
    d_state_prev = 0;
}

float TustinPID::getControl(float e, float uMin, float uMax)
{
    float P = getP(e);
    float D = getD(e);

    // --- Compute integrator candidate (pure Tustin) ---
    float I_candidate = i_state_prev + (Ki * Ts * 0.5f) * (e + prev_e);

    // --- Compute unsaturated output ---
    float u_unsat = P + I_candidate + D;

    // --- Apply saturation ---
    float u_sat = u_unsat;
    if (u_sat > uMax) u_sat = uMax;
    if (u_sat < uMin) u_sat = uMin;

    // --- Anti-windup freeze logic ---
    bool freeze_integrator = (u_unsat != u_sat);

    if (!freeze_integrator)
        i_state = I_candidate;     // update normally
    else
        i_state = i_state_prev;    // freeze

    // --- Update stored states ---
    i_state_prev = i_state;
    prev_e = e;

    return u_sat;
}

float TustinPID::getControl(float e)
{
    float P = getP(e);
    float I = getI(e);
    float D = getD(e);

    prev_e = e;
    return P + I + D;
}

float TustinPID::getP(float e)
{
    return Kp * e;
}

// Pure Tustin integrator (no anti-windup logic here)
float TustinPID::getI(float e)
{
    i_state_prev = i_state;
    i_state = i_state_prev + (Ki * Ts * 0.5f) * (e + prev_e);
    return i_state;
}

// Tustin filtered differentiator
float TustinPID::getD(float e)
{
    d_state_prev = d_state;

    float a = (2.0f * tau - Ts) / (2.0f * tau + Ts);
    float b = (2.0f * Kd) / (2.0f * tau + Ts);

    d_state = a * d_state_prev + b * (e - prev_e);
    return d_state;
}