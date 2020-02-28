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

void drive(int speed) //Set both motors to drive forward; 1 = slow, 2 = med, 3 = fast
{
    if (speed == 0)
    {
        rmotor.SetPercent(RSLOW);
        lmotor.SetPercent(LSLOW);
    }

    else if (speed == 1)
    {
        rmotor.SetPercent(RMED);
        lmotor.SetPercent(LMED);
    }

    else if (speed == 2)
    {
        rmotor.SetPercent(RFAST);
        lmotor.SetPercent(LFAST);
    }
}

void reverse(int speed) //Set both motors to drive backwards; 1 = slow, 2 = med, 3 = fast
{
    if (speed == 0)
    {
        rmotor.SetPercent(-RSLOW);
        lmotor.SetPercent(-LSLOW);
    }

    else if (speed == 1)
    {
        rmotor.SetPercent(-RMED);
        lmotor.SetPercent(-LMED);
    }

    else if (speed == 2)
    {
        rmotor.SetPercent(-RFAST);
        lmotor.SetPercent(-LFAST);
    }
}

void reset() //Reset both encoder counts
{
    re.ResetCounts();
    le.ResetCounts();
}

void avg(float goal) //Wait until average encoder counts reaches goal
{
    reset();

    while ((le.Counts() + re.Counts())/2 < goal){}
}

void forward(int speed, float distance) //Drive forward for some distance; speed 0-2, distance in inches
{
    float counts;

    drive(0); //Start moving forward

    counts = distance*(2.5*PI/200); //Set counts

    avg(counts); //Wait until distance is reached

    stop(); //Stop motors
}

void backwards(int speed, float distance) //Drive forward for some distance; speed 0-2, distance in inches
{
    float counts;

    reverse(0); //Start moving backwards

    counts = distance*(2.5*PI/200); //Set counts

    avg(counts); //Wait until distance is reached

    stop(); //Stop motors
}

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
