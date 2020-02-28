/* Libraries */

#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <math.h>

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Definitions */

//Encoder thresholds
#define HIGH 2.0
#define LOW 1.0

//Color thresholds
#define REDL 0.0
#define REDH 0.6
#define BLUEL 0.9
#define BLUEH 1.5

//Speeds
#define RSLOW -35.0
#define LSLOW -39.0
#define RMED -65.0
#define LMED -57.5
#define LFAST -95.0
#define RFAST -87.5

//Encoder counts
#define NINETY 340

//Servo thresholds
#define RSERVOH 2343
#define RSERVOL 500
#define LSERVOH 2400
#define LSERVOL 500

//Misc
#define PI 3.14159265359

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Declarations */

/* CdS cell */
AnalogInputPin cds(FEHIO::P1_7);

/* Bumper switches */
AnalogInputPin fl(FEHIO::P0_3);
AnalogInputPin fr(FEHIO::P3_4);
AnalogInputPin bl(FEHIO::P0_0);
AnalogInputPin br(FEHIO::P3_7);

/* Shaft encoders */
AnalogEncoder le(FEHIO::P0_7);
AnalogEncoder re(FEHIO::P3_0);

/* Motors */
FEHMotor lmotor(FEHMotor::Motor3,7.2);
FEHMotor rmotor(FEHMotor::Motor0,7.2);

/* Servos */

FEHServo larm(FEHServo::Servo0);
FEHServo rarm(FEHServo::Servo7);

/* Line Following */

AnalogInputPin l(FEHIO::P1_3);
AnalogInputPin m(FEHIO::P1_5);
AnalogInputPin r(FEHIO::P1_7);

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Component functions */

void stop() //Stop both motors
{
    rmotor.Stop();
    lmotor.Stop();
    Sleep(200);
}

void reset() //Reset both encoder counts
{
    re.ResetCounts();
    le.ResetCounts();
}

float lset(int speed) //Returns speed value for left motor; 1 - slow, 2 - medium, 3 - fast
{
    float left;

    if (speed == 1)
        left = LSF;
    else if (speed == 2)
        left = LMF;
    else if (speed == 3)
        left = LFF;
    else if (speed == -1)
        left = LSB;
    else if (speed == -2)
        left = LMB;
    else if (speed == -3)
        left = LFB;

    return left;
}

float rset(int speed) //Returns speed value for right motor; 1 - slow, 2 - medium, 3 - fast
{
    float right;

    if (speed == 1)
        right = RSF;
    else if (speed == 2)
        right = RMF;
    else if (speed == 3)
        right = RFF;
    else if (speed == -1)
        right = RSB;
    else if (speed == -2)
        right = RMB;
    else if (speed == -3)
        right = RFB;

    return right;
}

void drive(int speed) //Turn both motors on going forward
{
    lmotor.SetPercent(lset(speed));
    rmotor.SetPercent(rset(speed));
}

void reverse(int speed) //Turn both motors on going back
{
    lmotor.SetPercent(lset(speed));
    rmotor.SetPercent(rset(speed));
}

void forward(int type, int speed, float distance, float time) //Move forward until hit wall, or until distance has been reached;  type 1 - distance, type 2 - time
{
    int counts;
    float x = -4.0, now; //Boost amount

    drive(speed); //Starts motors going forward

    //Set left and right default speeds
    float left = lset(speed), right = rset(speed);

    counts = distance*(200/2.5/PI); //Set counts

    switch (type) //1 - switches, 2 - some distance, 3 - time
    {
    case 1:
        while (le.Counts() < counts || re.Counts() < counts) //Waits until distance is reached while adjusting speeds
        {
            if (le.Counts() < re.Counts()) //If left motor is behind, boost left for 10 ms
            {
                lmotor.SetPercent(left + x);
                Sleep(10);
                lmotor.SetPercent(left);
            }

            if (re.Counts() < le.Counts()) //If right motor is behind, boost right for 10 ms
            {
                rmotor.SetPercent(right + x);
                Sleep(10);
                rmotor.SetPercent(right);
            }
        }
        break;

    case 2:
        now = TimeNow();
        while ((TimeNow() - now) < time) //Waits until specified time has passed while adjusting speeds
        {
            if (le.Counts() < re.Counts()) //If left motor is behind, boost left for 10 ms
            {
                lmotor.SetPercent(left + x);
                Sleep(10);
                lmotor.SetPercent(left);
            }

            if (re.Counts() < le.Counts()) //If right motor is behind, boost right for 10 ms
            {
                rmotor.SetPercent(right + x);
                Sleep(10);
                rmotor.SetPercent(right);
            }
        }
        break;
    }

    stop();
}

void backward(int type, int speed, float distance, float time) //Move backward until hit wall, or until distance has been reached;  type 1 - distance, type 2 - time
{
    int counts;
    float x = 4.0, now; //Boost amount

    reverse(speed); //Starts motors going forward

    //Set left and right default speeds
    float left = lset(speed), right = rset(speed);

    counts = distance*(200/2.5/PI); //Set counts

    switch (type) //1 - switches, 2 - some distance, 3 - time
    {
    case 1:
        while (le.Counts() < counts || re.Counts() < counts) //Waits until distance is reached while adjusting speeds
        {
            if (le.Counts() < re.Counts()) //If left motor is behind, boost left for 10 ms
            {
                lmotor.SetPercent(left + x);
                Sleep(10);
                lmotor.SetPercent(left);
            }

            if (re.Counts() < le.Counts()) //If right motor is behind, boost right for 10 ms
            {
                rmotor.SetPercent(right + x);
                Sleep(10);
                rmotor.SetPercent(right);
            }
        }
        break;

    case 2:
        now = TimeNow();
        while ((TimeNow() - now) < time) //Waits until specified time has passed while adjusting speeds
        {
            if (le.Counts() < re.Counts()) //If left motor is behind, boost left for 10 ms
            {
                lmotor.SetPercent(left + x);
                Sleep(10);
                lmotor.SetPercent(left);
            }

            if (re.Counts() < le.Counts()) //If right motor is behind, boost right for 10 ms
            {
                rmotor.SetPercent(right + x);
                Sleep(10);
                rmotor.SetPercent(right);
            }
        }
        break;
    }

    stop();
}

//rturn lturn and spin not yet using PID
void rturn(int speed, float angle) //Turns bot specified degrees to right
{
    int counts = 0;

    if (speed == 1 && angle > 0.0)
        rmotor.SetPercent(RSLOW);

    else if (speed == 2 && angle > 0.0)
        rmotor.SetPercent(RMED);

    else if (speed == 3 && angle > 0.0)
        rmotor.SetPercent(RFAST);

    else if (speed == 1 && angle < 0.0)
        rmotor.SetPercent(-RSLOW);

    else if (speed == 2 && angle < 0.0)
        rmotor.SetPercent(-RMED);

    else if (speed == 3 && angle < 0.0)
        rmotor.SetPercent(-RFAST);

    if (angle < 0.0)
        angle *= -angle;

    counts = angle/360*8.5*PI;

    while (re.Counts() < counts){}

    stop();
}

void lturn(int speed, float angle)
{
    int counts = 0;

    if (speed == 1 && angle > 0.0)
        lmotor.SetPercent(RSLOW);

    else if (speed == 2 && angle > 0.0)
        lmotor.SetPercent(RMED);

    else if (speed == 3 && angle > 0.0)
        lmotor.SetPercent(RFAST);

    else if (speed == 1 && angle < 0.0)
        lmotor.SetPercent(-RSLOW);

    else if (speed == 2 && angle < 0.0)
        lmotor.SetPercent(-RMED);

    else if (speed == 3 && angle < 0.0)
        lmotor.SetPercent(-RFAST);

    if (angle < 0.0)
        angle *= -angle;

    counts = angle/360*8.5*PI;

    while (le.Counts() < counts){}

    stop();
}

void spin(int speed, float angle)
{
    int counts = 0;

    if (speed == 1 && angle > 0.0)
    {
        rmotor.SetPercent(RSLOW);
        lmotor.SetPercent(-LSLOW);
    }

    else if (speed == 2 && angle > 0.0)
    {
        rmotor.SetPercent(RMED);
        lmotor.SetPercent(-LMED);
    }

    else if (speed == 3 && angle > 0.0)
    {
        rmotor.SetPercent(RFAST);
        lmotor.SetPercent(-LFAST);
    }

    else if (speed == 1 && angle < 0.0)
    {
        rmotor.SetPercent(-RSLOW);
        lmotor.SetPercent(LSLOW);
    }

    else if (speed == 2 && angle < 0.0)
    {
        rmotor.SetPercent(-RMED);
        lmotor.SetPercent(LMED);
    }

    else if (speed == 3 && angle < 0.0)
    {
        rmotor.SetPercent(-RFAST);
        lmotor.SetPercent(LFAST);
    }

    if (angle < 0.0)
        angle *= -angle;

    counts = angle/360*8.5*PI;

    while ((re.Counts() + le.Counts()) < counts){}

    stop();
}


/* Task 0: Start */

void task_0()
{
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    bool flag = true;
    while (flag == true) //Wait for red light to start
    {
        if (cds.Value() > 1.0)
        {
            LCD.Clear(FEHLCD::Black);
            LCD.WriteLine("Waiting...");
        }

        else if (cds.Value() < 0.5)
        {
            LCD.Clear(FEHLCD::Red);
            flag = false;
        }

        Sleep(200);
    }
}

/* Task 1: Ticket Slide */

void task_1()
{
    //Set initial arm positions
    larm.SetDegree(0);
}

/* Task 2: Jukebox */



/* Task 3: Tray Drop (Sink) */



/* Task 4: Borger Flip */



/* Task 5: Ice Cream Lever */



/* Final Task: Return to Start */



/* Main */

int main(void)
{
    return 0;
}
