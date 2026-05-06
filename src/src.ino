#include <FastLED.h>
#include <esp_timer.h>

// 定义LED配置
#define LED_PIN 4 // 数据引脚连接到 GPIO4
#define TOUCH_PIN 2
#define IR_PIN 1
#define LDR_PIN 0

#define NUM_LEDS 4 // 串联的灯珠数量

#define LIGHT_DURATION (5000) // ms

CRGB leds[NUM_LEDS];
enum Style_E
{
    STYLE_WARM_WHITE = 0,
    STYLE_BRIGHT_WHITE,
    STYLE_SOFT_PINK,
    STYLE_COUNT
};
CRGB Style[STYLE_COUNT] = {
    {CHSV(30, 200, 255)},
    {CHSV(160, 30, 255)},
    {CHSV(230, 100, 200)}};

enum Style_E mode = STYLE_WARM_WHITE;
volatile bool detectHuman = false;
volatile bool detectTouch = false;
volatile uint64_t touchStart = 0;
volatile uint64_t touchEnd = 0;
uint16_t lightCycle = 0;
uint8_t brightness = 10;

void setup()
{
    Serial.begin(115200);

    // 初始化LED灯带，指定型号、引脚、颜色顺序
    // WS2812B通常使用GRB颜色顺序
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    // 设置全局亮度
    FastLED.setBrightness(brightness);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    pinMode(TOUCH_PIN, INPUT_PULLDOWN);
    pinMode(IR_PIN, INPUT_PULLDOWN);
    pinMode(LDR_PIN, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(IR_PIN), detectHumanISR, RISING);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), detectTouchISR, CHANGE);
}

void IRAM_ATTR detectHumanISR()
{
    detectHuman = true;
}

void IRAM_ATTR detectTouchISR()
{
    bool pinState = digitalRead(TOUCH_PIN);

    if (pinState)
    {
        touchStart = esp_timer_get_time() / 1000;
    }
    else
    {
        touchEnd = esp_timer_get_time() / 1000;
        detectTouch = true;
    }
}

void loop()
{
    if (detectHuman)
    {
        detectHuman = false;
        if (digitalRead(LDR_PIN))
        {
            // night
            lightCycle = (LIGHT_DURATION / 100);
            printf("detech human, night\n");
        }
        else
        {
            printf("detech human, day\n");
        }
    }
    if (lightCycle)
    {
        fill_solid(leds, NUM_LEDS, Style[mode]);
        FastLED.show();
        lightCycle--;
        printf("light trig\n");
        if (lightCycle == 0)
        {
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            printf("light close\n");
        }
    }
    if (detectTouch)
    {
        uint64_t pressDuration = touchEnd - touchStart;
        detectTouch = false;
        // printf("detech touch: %lld, %lld \n", touchStart, touchEnd);
        touchStart = touchEnd;

        if (pressDuration > 50 && pressDuration < 1000)
        {
            brightness += 40;
            if (brightness > 210)
            {
                brightness = 10;
            }
            FastLED.setBrightness(brightness);
        }
        else if (pressDuration >= 1000)
        {
            mode = (Style_E)(mode + 1);
            if (mode >= STYLE_COUNT)
            {
                mode = STYLE_WARM_WHITE;
            }
        }
        lightCycle += 10;
    }
    // printf("touch:%d, IR:%d, LDR:%d \n", digitalRead(TOUCH_PIN), digitalRead(IR_PIN), digitalRead(LDR_PIN));

    delay(100);
}
