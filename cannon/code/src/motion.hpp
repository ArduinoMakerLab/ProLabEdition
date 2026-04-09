#include <RingBuf.h>

enum Motion_E
{
    MOTION_NONE = 0,
    MOTION_FRONT_BACK,
    MOTION_UP_DOWN,
    MOTION_LEFT_RIGHT,
};

class motion
{
private:
    RingBuf<float, 50> motionX, motionY, motionZ;
    float interval;

public:
    motion(float dt);
    ~motion();
    Motion_E update(float att[3], float acc[3]);
};

motion::motion(float dt)
{
    interval = dt;
}

motion::~motion()
{
}

Motion_E motion::update(float att[3], float acc[3])
{
    float max, min, val;
    uint32_t poscnt, negcnt;

    motionX.push(acc[0]);
    if (motionX.isFull())
    {
        // ready
        max = 0;
        min = 0;
        poscnt = 0;
        negcnt = 0;
        for (uint16_t ii = 0; ii < motionX.size(); ii++)
        {
            motionX.peek(val, ii);
            if (val > max)
                max = val;
            if (val < min)
                min = val;
            if (val > 400)
                poscnt++;
            if (val < -400)
                negcnt++;
            // Serial.printf("ele %d, %f\n", ii, val);
        }
        // Serial.printf("motion x %f, %f, %f, %d, %d\n", val, max, min, poscnt, negcnt);
        if (poscnt > 8 && negcnt > 8 && max > 1200 && min < -1600)
        {
            Serial.printf("back -> breakdown check\n");
            motionX.clear();
            Serial.printf("motion x %f, %f, %f, %d, %d\n", val, max, min, poscnt, negcnt);
            return MOTION_FRONT_BACK;
        }
        else if (negcnt > 8 && poscnt > 8 && max > 1600 && min < -1200)
        {
            Serial.printf("front -> breakdown check\n");
            motionX.clear();
            Serial.printf("motion x %f, %f, %f, %d, %d\n", val, max, min, poscnt, negcnt);
            return MOTION_FRONT_BACK;
        }
        else
        {
            float clear;
            motionX.pop(clear);
        }

        float clear;
        motionX.pop(clear);
    }

    motionY.push(acc[1]);
    if (motionY.isFull())
    {
        // ready
        max = 0;
        min = 0;
        poscnt = 0;
        negcnt = 0;
        for (uint16_t ii = 0; ii < motionY.size(); ii++)
        {
            motionY.peek(val, ii);
            if (val > max)
                max = val;
            if (val < min)
                min = val;
            if (val > 400)
                poscnt++;
            if (val < -400)
                negcnt++;
            // Serial.printf("ele %d, %f\n", ii, val);
        }
        Serial.printf("motion y %f, %f, %f, %d, %d\n", val, max, min, poscnt, negcnt);
        if (poscnt > 8 && negcnt > 8 && max > 1200 && min < -1600)
        {
            Serial.printf("right -> left check\n");
            motionY.clear();
            return MOTION_LEFT_RIGHT;
        }
        else if (negcnt > 8 && poscnt > 8 && max > 1200 && min < -1600)
        {
            Serial.printf("left -> right check\n");
            motionY.clear();
            return MOTION_LEFT_RIGHT;
        }
        else
        {
            float clear;
            motionY.pop(clear);
        }
    }

    motionZ.push(acc[2]);
    if (motionZ.isFull())
    {
        // ready
        max = 0;
        min = 0;
        poscnt = 0;
        negcnt = 0;
        for (uint16_t ii = 0; ii < motionZ.size(); ii++)
        {
            motionZ.peek(val, ii);
            if (val > max)
                max = val;
            if (val < min)
                min = val;
            if (val > 1000)
                poscnt++;
            if (val < -1000)
                negcnt++;
            // Serial.printf("ele %d, %f\n", ii, val);
        }

        float clear;
        motionZ.pop(clear);
    }

    return MOTION_NONE;
}