#include "Servo.h"

extern "C"
{
#include <esp8266.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
}

// #define SERVO_DEBUG

#ifdef SERVO_DEBUG
#include <stdio.h>
#define debug(fmt, ...) printf("%s: " fmt "\n", "SERVO", ##__VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#define ENABLE_SR 1 // Comment if not using shift registers

#define MAX_ANGLE 180
#define CLK_DIVISOR 16
#define CLK_PER_US 80
#define US_TO_TICKS(_us) ((_us * CLK_PER_US) / CLK_DIVISOR) // assume 16 as CLK_DIVISOR
#define TICKS_TO_US(_ticks) (((_ticks * CLK_DIVISOR) / CLK_PER_US))

#define SERVO_INDEX_TO_CHN(_servo_nbr) ((_servo_nbr % SERVOS_PER_TIMER))
#define SERVO_INDEX(_channel) ((_channel * 1))
#define SERVO(_channel) ((servos[_channel]))

static Servo *servos[MAX_SERVOS];

// Only supports 1 timer since esp8266 only has 1 hardware timer not used for RTOS or sdk
static int8_t channels[1] = {-1}; // number of timer channels used to control servos, each channel can have max 12 servos

uint8_t servoCount = 0; // Total number of servos attached
uint32_t acculTicks = 0;
bool isInit = false;

#ifdef ENABLE_SR
void (*srCallback)(srReq); // Only 1 callback supported for SR mode
#endif

static void servo_interrupt_handler(void *arg)
{
#ifdef ENABLE_SR
    uint8_t pat; // only 1 pin can be on at a time
    bool patSet = false;
#endif
    // debug("Servo interrupt triggered.");
    if (channels[0] < 0)
    {
        // debug("Refresh interval finished. Resetting the cycle...");
        acculTicks = 0;
    }
    else if (SERVO_INDEX(channels[0]) < servoCount && (SERVO(channels[0])->servo.pin.isActive) != 0)
    {
        // clear gpio pin
        // debug("%d ticks fulfilled. Clearing pin.", SERVO(channels[0])->servo.ticks);
        acculTicks += SERVO(channels[0])->servo.ticks;
#ifdef ENABLE_SR
        if (SERVO(channels[0])->isUsingShiftReg())
        {
            pat = 0; // only 1 pin can be on at a time
            patSet = true;
        }
        else
        {
            gpio_write(SERVO(channels[0])->servo.pin.nbr, false);
        }
#else
        gpio_write(SERVO(channels[0])->servo.pin.nbr, false);
#endif
    }
    else
    {
        debug("Clear phase should not reach here.");
    }
    channels[0]++;
    if (SERVO_INDEX(channels[0]) < servoCount && (SERVO(channels[0])->servo.pin.isActive) != 0)
    {
        // debug("Asserting next pin %d.", SERVO(channels[0])->servo.pin.nbr);
#ifdef ENABLE_SR
        if (SERVO(channels[0])->isUsingShiftReg())
        {
            pat = (1 << (SERVO(channels[0])->servo.pin.nbr - 1)); // only 1 pin can be on at a time
            patSet = true;
        }
        else
        {
            gpio_write(SERVO(channels[0])->servo.pin.nbr, true);
        }
#else
        gpio_write(SERVO(channels[0])->servo.pin.nbr, true);
#endif
        timer_set_load(FRC1, SERVO(channels[0])->servo.ticks);
    }
    else if (SERVO_INDEX(channels[0]) >= servoCount)
    {
        // debug("All channels completed. Waiting for refresh interval. In %u ticks...", acculTicks);
        if (acculTicks < US_TO_TICKS(REFRESH_INTERVAL))
        {
            timer_set_load(FRC1, US_TO_TICKS(REFRESH_INTERVAL) - acculTicks);
        }
        else
        {
            timer_set_load(FRC1, 4); // to make it trigger almost immediately
        }
        channels[0] = -1;
    }
    else
    {
        debug("Set phase should not reach here.");
    }
#ifdef ENABLE_SR
    if (patSet)
    {
        srReq req = {pat, servoCount};
        srCallback(req);
        debug("SR Bit pattern is %x", req.bitPattern);
    }
#endif
}

static inline void initialise()
{
    debug("Initializing all interrupts and timers for servo control.");
    timer_set_interrupts(FRC1, false);
    timer_set_run(FRC1, false);
    timer_set_divider(FRC1, TIMER_CLKDIV_16);
    _xt_isr_attach(INUM_TIMER_FRC1, servo_interrupt_handler, NULL);
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);
    timer_set_load(FRC1, 4); // make timer trigger interrupt right after initialisation
    isInit = true;
}

static inline void reset()
{
    debug("Reset all interrupts and timers for servo control.");
    timer_set_interrupts(FRC1, false);
    timer_set_run(FRC1, false);
    _xt_isr_attach(INUM_TIMER_FRC1, NULL, NULL);
    isInit = false;
}

Servo::Servo()
{
    debug("Currently in the constructor.");
    this->_servoIndex = servoCount;
    this->_min = MIN_PULSE_WIDTH;
    this->_max = MAX_PULSE_WIDTH;
    servos[this->_servoIndex] = this;
    ServoPin_t servoPin{(uint8_t)255, 0};
    this->servo.pin = servoPin;
    servoCount++;
}

Servo::Servo(srCallback_t _srCallback)
{
    debug("Currently in the SR constructor.");
    this->_servoIndex = servoCount;
    this->_min = MIN_PULSE_WIDTH;
    this->_max = MAX_PULSE_WIDTH;
    servos[this->_servoIndex] = this;
    ServoPin_t servoPin{(uint8_t)255, 0};
    this->servo.pin = servoPin;
#ifdef ENABLE_SR
    if (srCallback == NULL)
        srCallback = _srCallback;
    this->_usesShiftReg = true;
#endif
    servoCount++;
}

Servo::~Servo()
{
    // temporarily stop interrupts so interrupts wouldn't come in middle
    timer_set_interrupts(FRC1, false);
    timer_set_run(FRC1, false);
    debug("Deleting servo object");
    servos[this->_servoIndex] = NULL;
    for (int i = _servoIndex; i < servoCount - 1; i++)
    {
        servos[i] = servos[i + 1];
    }
    servoCount--;
    if (servoCount == 0)
    {
        reset();
    }
    else
    {
        timer_set_interrupts(FRC1, true);
        timer_set_run(FRC1, true);
        timer_set_load(FRC1, 4); // make timer trigger interrupt immediately after
    }
}

uint8_t Servo::attach(int pin)
{
    debug("Attaching pin %d to servo.", pin);
    if (pin > 16)
    {
        return SERVO_ERR_INVLAID_PIN;
    }
    ServoPin_t servoPin{(uint8_t)pin, 1};
    this->servo.pin = servoPin;
    this->writeMicroseconds(DEFAULT_PULSE_WIDTH);
    if (!isInit)
        initialise(); // only set up linking of isr
    return SERVO_ERR_OK;
}

uint8_t Servo::attach(int pin, int min, int max)
{
    this->_min = min > 0 ? min : MIN_PULSE_WIDTH;
    this->_max = (max > 0 && max > min) ? max : MAX_PULSE_WIDTH;
    return this->attach(pin);
}

void Servo::detach()
{
    this->servo.pin.nbr = 255;
    this->servo.pin.isActive = 0;
}

uint8_t Servo::write(int value)
{
    if (value <= MAX_ANGLE)
    {
        uint16_t width = ((float)value / MAX_ANGLE) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) + MIN_PULSE_WIDTH;
        this->servo.ticks = US_TO_TICKS(width);
        return SERVO_ERR_OK;
    }
    return writeMicroseconds(value);
}

uint8_t Servo::writeMicroseconds(int value)
{
    debug("Received command to set %d us.", value);
    if (value < this->_min)
    {
        this->servo.ticks = US_TO_TICKS(this->_min);
        debug("1: Servo ticks set to %u", this->servo.ticks);
        return SERVO_ERR_INVALID_ANGLE;
    }
    debug("Max allowed %d", this->_max);
    if (value > this->_max)
    {
        this->servo.ticks = US_TO_TICKS(this->_max);
        debug("2: Servo ticks set to %u", this->servo.ticks);
        return SERVO_ERR_INVALID_ANGLE;
    }
    this->servo.ticks = US_TO_TICKS(value);
    debug("3: Servo ticks set to %u", this->servo.ticks);
    return SERVO_ERR_OK;
}

uint8_t Servo::read()
{
    uint16_t width = this->readMicroseconds();
    return (int)(((float)(width - MIN_PULSE_WIDTH) / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) * 180);
}

uint16_t Servo::readMicroseconds()
{
    return TICKS_TO_US(this->servo.ticks);
}

bool Servo::isAttached()
{
    return this->servo.pin.nbr != 255;
}

bool Servo::isUsingShiftReg()
{
#ifdef ENABLE_SR
    return this->_usesShiftReg;
#else
    return false;
#endif
}
