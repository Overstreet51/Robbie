#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>

#define RSLOW -35.0
#define LSLOW -39.0
#define RFAST -70.0
#define LFAST -80.0
#define NINE 27


AnalogInputPin cds(FEHIO::P1_7);
AnalogInputPin fl(FEHIO::P3_7);
AnalogInputPin fr(FEHIO::P0_0);
AnalogInputPin br(FEHIO::P0_3);
AnalogInputPin bl(FEHIO::P3_4);
AnalogInputPin le(FEHIO::P3_0);
AnalogInputPin re(FEHIO::P0_7);
FEHMotor lmotor(FEHMotor::Motor2,7.2);
FEHMotor rmotor(FEHMotor::Motor3,7.2);

int lencoder()
{
    bool flag;

    if (le.Value() > 2.0)
    {
        flag = true;
        while (flag == true)
        {
            if (le.Value() < 1.0)
                flag = false;
        }
        return 1;
    }

     else if (le.Value() < 1.0)
    {
        flag = true;
        while (flag == true)
        {
            if (le.Value() > 2.0)
                flag = false;
        }
        return 1;
    }
}

int rencoder()
{
    bool flag;

    if (re.Value() > 2.0)
    {
        flag = true;
        while (flag == true)
        {
            if (re.Value() < 1.0)
                flag = false;
        }
        return 1;
    }

     else if (re.Value() < 1.0)
    {
        flag = true;
        while (flag == true)
        {
            if (re.Value() > 2.0)
                flag = false;
        }
        return 1;
    }
}

void stop()
{
    rmotor.SetPercent(0.0);
    lmotor.SetPercent(0.0);
    Sleep(100);
}

void forward(int speed)
{
    if (speed == 0)
    {
        rmotor.SetPercent(RSLOW);
        lmotor.SetPercent(LSLOW);
    }

    else if (speed == 1)
    {
        rmotor.SetPercent(RFAST);
        lmotor.SetPercent(LFAST);
    }
}

void back(int speed)
{
    if (speed == 0)
    {
        rmotor.SetPercent(-RSLOW);
        lmotor.SetPercent(-LSLOW);
    }

    else if (speed == 1)
    {
        rmotor.SetPercent(-RFAST);
        lmotor.SetPercent(-LFAST);
    }
}


/* Main course functions */

void begin()
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

        Sleep(100);
    }
}

void move_to_light(float left, float right)
{
    int count;
    bool flag;

    forward(0);

    count = 0;
    while (count < 18) //Distance forward
    {
        count += lencoder();
    }

    stop();

    rmotor.SetPercent(right);

    count = 0;
    while (count < 14) //Right wheel turn
    {
         count += rencoder();
    }

    stop();

    forward(0);

    flag = true;
    while (flag == true) //Stop at light
    {
        if (cds.Value() < 1.5)
        {
            LCD.Clear(FEHLCD::Black);
            flag = false;
        }
    }

    stop();
}

int read_light() //Determine red/blue button to press
{
    int m = 3;
    while (m== 3)
    {

        if (cds.Value() > 0.0 && cds.Value() < 0.6)
        {
            LCD.Clear(FEHLCD::Red);
            m= 0;
        }

        else if (cds.Value() > 0.9 && cds.Value() < 1.5)
        {
            LCD.Clear(FEHLCD::Blue);
            m=1;
        }

    }
    return m;
}

void blue(float left, float right) //Go for red button
{
    int count;
    float t1;

    forward(0);

    while (fr.Value() > 0.1 || fl.Value() > 0.1){}

    stop();

    lmotor.SetPercent(-LSLOW);

    count = 0;
    while (count < NINE) //90 degree turn back left
    {
        count += lencoder();
    }

    forward(0);

    t1 = TimeNow();
    while ((TimeNow() - t1) < 2.0){}

    stop();

    lmotor.SetPercent(-LSLOW);

    count = 0;
    while (count < NINE)
    {
        count += lencoder();
    }

    stop();

    back(0);

    while (br.Value() > 0.1 || bl.Value() > 0.1){}

    stop();
}

void red(float left, float right) //Go for blue  button
{
    int count;
    float t1;

    forward(0);

    while (fr.Value() > 0.1 || fl.Value() > 0.1){}

    stop();

    lmotor.SetPercent(-LSLOW);

    count = 0;
    while (count < 11) //Turn back left
    {
        count += lencoder();
    }

    stop();

    rmotor.SetPercent(RSLOW);

    count = 0;
    while (count < 14)
    {
        count += rencoder();
    }

    stop();

    forward(0);

    t1 = TimeNow();
    while ((TimeNow() - t1) < 0.75){}

    stop();

    lmotor.SetPercent(-LSLOW);

    count = 0;
    while (count < NINE)
    {
        count += lencoder();
    }

    stop();

    back(0);

    while (br.Value() > 0.1 || bl.Value() > 0.1){}

    stop();
}

void ramp()
{
    int count;

    forward(0);

    count = 0;
    while (count < 22)
    {
        count += rencoder();
    }

    stop();

    rmotor.SetPercent(RSLOW);

    count = 0;
    while (count < NINE+1)
    {
        count += rencoder();
    }

    stop();

    forward(0);

    count = 0;
    while (count < 16)
    {
        count += rencoder();
    }

    forward(0);

    count = 0;
    while (count < 30)
    {
        count += rencoder();
    }

    stop();

    back(0);

    count = 0;
    while (count < 50)
    {
        count += rencoder();
    }

    stop();
}

int main(void)
{
    int light;

    begin();

    move_to_light(LSLOW, RSLOW);

    Sleep(1.0);

    light = read_light();

    if (light == 1) //Blue; forward, turn back left, forward
    {
        blue(LSLOW, RSLOW);
    }

    else if (light == 0) //Red; forward, turn back left, turn forward left, forward
    {
        red(LSLOW, RSLOW);
    }

    ramp();


    return 0;
}
