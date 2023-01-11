/*
 * pid.c
 *
 */

#include    "pid.h"


static PID_STRUCT sPID;

PID_STRUCT* Init_pid(void)
{
    // Set PID Coefficients
    sPID.Kp = 0.85;
    sPID.Ki = 0.001;
    sPID.Kd = 0.02;
    sPID.Kc = 0.8;


    // Set PID Setpoint
    sPID.RefSetPoint = 2068;        // Default output voltage 5.0V
    sPID.OutputMax = 3500;
    sPID.OutputMin = 10;

    return &sPID;
}


void pid_process(PID_STRUCT* psPID)
{
    // Compute the error
    psPID->Error = psPID->RefSetPoint - psPID->Feedback_AdcPoint;

    // Compute the proportional output
    psPID->Up = psPID->Kp * psPID->Error;

    // Compute the integral output
    psPID->Ui = psPID->Ui + (psPID->Ki * psPID->Up) + (psPID->Kc * psPID->SatErr);

    // Compute the derivative output
    psPID->Ud = psPID->Kd * (psPID->Up - psPID->Up1);

    // Compute the pre-saturated output
    psPID->OutputPreSat = psPID->Up + psPID->Ui + psPID->Ud;

    // Saturate the output
    if(psPID->OutputPreSat > psPID->OutputMax)
        psPID->Output = psPID->OutputMax;
    else if(psPID->OutputPreSat < psPID->OutputMin)
        psPID->Output = psPID->OutputMin;
    else
        psPID->Output = psPID->OutputPreSat;

    // Update the previous proportional output
    psPID->SatErr = psPID->Output - psPID->OutputPreSat;

    // Update the previous proportional output
    psPID->Up1 = psPID->Up;

}



