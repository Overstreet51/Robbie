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
#define RSF -34.5
#define LSF -40.0
#define RMF -56.5
#define LMF -65.0
#define RFF -84.8
#define LFF -90.0

#define RSB 34.5
#define LSB 40.0
#define RMB 56.8
#define LMB 65.0
#define RFB 84.8
#define LFB 90.0

//Encoder counts
#define NINETY 340

//Servo thresholds
#define RSERVOH 2343
#define RSERVOL 500
#define LSERVOH 2400
#define LSERVOL 500

//Misc
#define PI 3.14159265359
#define CW45 -39.0
#define CW90 -87.5
#define CCW90 84

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Declarations */

/* CdS cell */
AnalogInputPin cds(FEHIO::P2_3);

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

AnalogInputPin l(FEHIO::P1_1);
AnalogInputPin m(FEHIO::P1_4);
AnalogInputPin r(FEHIO::P1_7);

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Component functions */

void th()
{
    le.SetThresholds(LOW, HIGH);
    re.SetThresholds(LOW, HIGH);

    rarm.SetMin(RSERVOL);
    rarm.SetMax(RSERVOH);
    larm.SetMin(LSERVOL);
    larm.SetMax(LSERVOH);
}

void stop() //Stop both motors
{
    rmotor.Stop();
    lmotor.Stop();
    Sleep(200);
}

void drive(int speed) //Set both motors to drive forward; 1 = slow, 2 = med, 3 = fast
{
    if (speed == 1) //Slow speed
    {
        lmotor.SetPercent(LSF);
        Sleep(100);
        lmotor.SetPercent(LSF);
        rmotor.SetPercent(RSF);
    }

    else if (speed == 2) //Medium speed
    {
        lmotor.SetPercent(LMF);
        rmotor.SetPercent(RMF);
    }

    else if (speed == 3) //Fast speed
    {
        lmotor.SetPercent(LFF);
        rmotor.SetPercent(RFF);
    }
}

void reverse(int speed) //Set both motors to drive backwards; 1 = slow, 2 = med, 3 = fast
{
    if (speed == 1) //Slow speed
    {
        lmotor.SetPercent(LSB);
        Sleep(100);
        rmotor.SetPercent(RSB);
        lmotor.SetPercent(LSB);
    }

    else if (speed == 2) //Medium speed
    {
        rmotor.SetPercent(RMB);
        lmotor.SetPercent(LMB);
    }

    else if (speed == 3) //Fast speed
    {
        rmotor.SetPercent(RFB);
        lmotor.SetPercent(LFB);
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
    th();

    while (/*le.Counts() < goal || re.Counts() < goal*/(le.Counts() + re.Counts())/2 < goal)//Calculates average of two encoder counts on loop until >= goal
    {/*
        if (le.Counts() < re.Counts())
        {
            lmotor.SetPercent(LSF + 5.0);
            Sleep(50);
            lmotor.SetPercent(LSF);
        }

        else if (re.Counts() < le.Counts())
        {
            rmotor.SetPercent(RSF + 5.0);
            Sleep(50);
            rmotor.SetPercent(RSF);
        }*/
    }
}

void forward(int speed, float distance) //Drive forward for some distance; speed 0-2, distance in inches
{
    float counts;

    drive(speed); //Start moving forward

    counts = distance*(200/2.5/PI); //Set counts

    avg(counts); //Wait until distance is reached

    stop(); //Stop motors
}

void backwards(int speed, float distance) //Drive forward for some distance; speed 0-2, distance in inches
{
    float counts;

    reverse(speed); //Start moving backwards

    counts = distance*(200/2.5/PI); //Set counts

    avg(counts); //Wait until distance is reached

    stop(); //Stop motors
}

void lturn(int speed, float angle) //Turns bot specified degrees to left
{
    int counts = 0;

    reset();
    th();

    if (speed == 1 && angle > 0.0) //Slow speed, forward turn CCW
        rmotor.SetPercent(RSF);

    else if (speed == 2 && angle > 0.0) //Medium speed, forward turn CCW
        rmotor.SetPercent(RMF);

    else if (speed == 3 && angle > 0.0) //Fast speed, forward turn CCW
        rmotor.SetPercent(RFF);

    else if (speed == 1 && angle < 0.0) //Slow speed, backward turn CW
        rmotor.SetPercent(RSB);

    else if (speed == 2 && angle < 0.0) //Medium speed, backward turn CW
        rmotor.SetPercent(RMB);

    else if (speed == 3 && angle < 0.0) //Fast speed, backward turn CW
        rmotor.SetPercent(RFB);

    if (angle < 0.0) //Take absolute value of angle
        angle *= -1;

    counts = angle*2*660/360; //Calculate number of counts per degree

    while (re.Counts() < counts){} //Waits until right wheel encoder >= counts

    stop(); //Stops motor
}

void rturn(int speed, float angle) //Turns bot specified degrees to right
{
    int counts = 0;

    reset();

    if (speed == 1 && angle > 0.0) //Slow speed, forward turn CW
        lmotor.SetPercent(LSF);

    else if (speed == 2 && angle > 0.0) //Medium speed, forward turn CW
        lmotor.SetPercent(LMF);

    else if (speed == 3 && angle > 0.0) //Fast speed, forward turn CW
        lmotor.SetPercent(LFF);

    else if (speed == 1 && angle < 0.0) //Slow speed, backward turn CCW
        lmotor.SetPercent(LSB);

    else if (speed == 2 && angle < 0.0) //Medium speed, backward turn CCW
        lmotor.SetPercent(LMB);

    else if (speed == 3 && angle < 0.0) //Fast speed, backward turn CCW
        lmotor.SetPercent(LFB);

    if (angle < 0.0) //Take absolute value of angle
        angle *= -1;

    counts = angle*2*660/360; //Calculate number of counts per degree

    while (le.Counts() < counts){} //Waits until left wheel encoder >= counts

    stop(); //Stops motor
}

void spin(int speed, float angle) //Spins bot in place in either direction
{
    int counts = 0;

    reset();

    if (speed == 1 && angle > 0.0) //Slow speed, CCW spin
    {
        rmotor.SetPercent(RSF);
        lmotor.SetPercent(LSB);
    }

    else if (speed == 2 && angle > 0.0) //Medium speed, CCW spin
    {
        rmotor.SetPercent(RMF);
        lmotor.SetPercent(LMB);
    }

    else if (speed == 3 && angle > 0.0) //Fast speed, CCW spin
    {
        rmotor.SetPercent(RFF);
        lmotor.SetPercent(LFB);
    }

    else if (speed == 1 && angle < 0.0) //Slow speed, CW spin
    {
        rmotor.SetPercent(RSB);
        lmotor.SetPercent(LSF);
    }

    else if (speed == 2 && angle < 0.0) //Medium speed, CW spin
    {
        rmotor.SetPercent(RMB);
        lmotor.SetPercent(LMF);
    }

    else if (speed == 3 && angle < 0.0) //Fast speed, CW spin
    {
        rmotor.SetPercent(RFB);
        lmotor.SetPercent(LFF);
    }

    if (angle < 0.0) //Take absolute value of angle
        angle *= -1;

    counts = angle*660/360; //Calculates counts per degree of rotation per wheel

    bool flag = true;
    while (flag == true) //Wait until both wheels have completed full turns
    {
        if (re.Counts() >= counts && le.Counts() < counts)
            rmotor.SetPercent(0.0);
        else if (le.Counts() >= counts && re.Counts() < counts)
            lmotor.SetPercent(0.0);
        else if (re.Counts() >= counts && le.Counts() >= counts)
        {
            flag = false;
            stop();
        }
    }
}

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

bool read() //Read jukebox light color
{
    bool color;
    float value;

    for (int i = 0; i < 10; i++) //Find average of light over 1 second with 10 intervals
    {
        value += cds.Value();
        Sleep(1.0);
    }
    value /= 10;

    if (value > REDL && value < REDH) //Red light
    {
        LCD.Clear(FEHLCD::Red);
        color = true;
    }

    else if (value > BLUEL && value < BLUEH) //Blue light
    {
        LCD.Clear(FEHLCD::Blue);
        color = false;
    }

    return color;
}

void red() //Press red button
{
    //Drive forward until aligned with wall
    drive(1);
    while (fr.Value() > 0.1 || fl.Value() > 0.1){}
    stop();

    //Reverse, turn to button, and press button
    backwards(1, 2);
    spin(1, -90);
    reverse(1);
    Sleep(2.0);
    stop();
    forward(1, 5.0);
}

void blue() //Press blue button
{
    //Drive forward until aligned with wall
    drive(1);
    while (fr.Value() > 0.1 || fl.Value() > 0.1){}
    stop();

    //Reverse, turn to button, and press button
    backwards(1, 2);
    rturn(1, -90);
    reverse(1);
    Sleep(2.0);
    stop();
    forward(1, 5.0);
}

/*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

/* Start */

void task_0()
{
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    th();

    larm.SetDegree(0.0);
    rarm.SetDegree(20.0);

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

/* Ticket Slider */

void task_1()
{
    th();

    float x, y;

    spin(1, CW45 - 1.0);

    rarm.SetDegree(125);

    drive(1);
    while (fr.Value() > 0.1 || fl.Value() > 0.1){}
    stop();

    drive(1);

    for (int i = 0; i < 20; ++i)
    {
        rarm.SetDegree(125.0 - i*4.0);
        Sleep(100);
    }
    stop();

    for (int i = 0; i < 5; ++i)
    {
        rarm.SetDegree(35.5 + i*4.0);
        Sleep(100);
    }
    rarm.SetDegree(95.0);

    backwards(1, 7.0);

    rarm.SetDegree(20.0);

    spin(1, CCW90);

    reverse(1);
    while (br.Value() > 0.1 || bl.Value() > 0.1){}
    stop();

    forward(1, 15.0);

    spin(1, CW90);
    forward(2, 29);
    rturn(1, -87.0);
    lmotor.SetPercent(LSF - 10.0);
    rmotor.SetPercent(RSF);
    Sleep(1.0);
    rmotor.SetPercent(0.0);
    Sleep(1.0);
    //while (fl.Value() > 0.1 &&  bl.Value() > 0.1){}
    stop();

    larm.SetDegree(180.0);
    Sleep(1.0);
    larm.SetDegree(150.0);
    Sleep(200);
    larm.SetDegree(180.0);
    Sleep(200);
    larm.SetDegree(150.0);
    Sleep(200);
    larm.SetDegree(180.0);
    Sleep(200);
    larm.SetDegree(150.0);
    Sleep(200);
    larm.SetDegree(180.0);
    Sleep(200);
    larm.SetDegree(150.0);
    Sleep(200);
    larm.SetDegree(180.0);
    Sleep(200);
    larm.SetDegree(150.0);
    Sleep(200);
    larm.SetDegree(180.0);
    Sleep(200);
    larm.SetDegree(0.0);

    reverse(1);

    while (br.Value() > 0.1 || bl.Value() > 0.1) {}

    forward(1, 2.0);
    rturn(1, 90);
    drive(1);
    while (fr.Value() > 0.1 && fl.Value() > 0.1) {}
    stop();

}

/* Main */

int main(void)
{

    float x, y;
/*
    while(true)
    {
        //spin(1, CCW90);
        rturn(1, 90);

        while (!LCD.Touch(&x, &y)){}

       // spin(1, 80.0);

       // while (!LCD.Touch(&x, &y)){}
    }
*/

    //reverse(1);

    task_0();
    task_1();

    //drive(1);

    //forward(1, 20.0);

    /*while (true)
    {
        //LCD.WriteLine();
    }*/
    return 0;
}
