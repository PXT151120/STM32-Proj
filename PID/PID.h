typedef struct PID
{
    double kp;
    double ki;
    double kd;

    double P_term;
    double I_term;
    double D_term;

    unsigned long deltaT;
    unsigned long prevT;

    double myInput;
    double mySetpoint;
    double myOuput;

    int myMaxLim;
    int myMinLim;

    double prevError;
    double myError;

    double tau;

} PID_t;

void PID_Config(PID_t *, double, double, double, double, int, int);
double PID_Update(PID_t *, double, double);