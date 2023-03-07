#include "PID.h"


void PID_Config(PID_t *userPID, double Kp, double Ki, double Kd, double tau, int maxLim, int minLim)
{
    userPID->kp = Kp;
    userPID->ki = Ki;
    userPID->kd = Kd;

    userPID->myMaxLim = maxLim;
    userPID->myMinLim = minLim;

    userPID->myInput = 0.0f;
    userPID->mySetpoint = 0.0f;
    userPID->myOuput = 0.0f;

    userPID->P_term = 0.0f;
    userPID->I_term = 0.0f;
    userPID->D_term = 0.0f;

    userPID->deltaT = 0;
    userPID->prevT = 0;

    userPID->myError = 0.0f;
    userPID->prevError = 0.0f;

    userPID->tau = tau;
    return;
}

double PID_Update(PID_t *userPID, double setPoint, double processVar)
{

    userPID->myError = setPoint - processVar;

    userPID->deltaT = HAL_GetTick() - userPID->prevT;

    // P term calculation

    userPID->P_term = userPID->kp * userPID->myError;

    // I term calculation
    userPID->I_term += userPID->ki * 0.5f * userPID->deltaT * (userPID->myError + userPID->prevError);

    // Avoid wind-up
    if (userPID->I_term > userPID->myMaxLim)
        userPID->I_term = userPID->myMaxLim;
    else if (userPID->I_term < userPID->myMinLim)
        userPID->I_term = userPID->myMinLim;

    // D term calculation with lowpass filter
    userPID->D_term = (2.0f * userPID->kd * (userPID->myError - userPID->prevError)
                        +  (userPID->tau - userPID->deltaT) * userPID->D_term)
                        / (2.0f * userPID->tau + userPID->deltaT);

    userPID->myOuput = userPID->P_term + userPID->I_term + userPID->D_term;


    // Avoid wind-up output
    if (userPID->myOuput > userPID->myMaxLim)
        userPID->myOuput = userPID->myMaxLim;
    else if (userPID->myOuput < userPID->myMinLim)
        userPID->myOuput = userPID->myMinLim;


    // Update variable
    userPID->prevError = userPID->myError;
    userPID->prevT = HAL_GetTick();

    return userPID->myOuput;
}