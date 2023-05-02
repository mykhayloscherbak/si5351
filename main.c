#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include "Si5351A-RevB-Registers.h"

#define SDA _BV(1)
#define SCL _BV(2)
#define ADDR (0x60)

static volatile uint8_t timerReady = 0;

ISR(TIM0_COMPA_vect)
{
    timerReady = 0xff;
}

static void waitTimer(void )
{
    cli();
    timerReady = 0;
    sei();
    while (0 == timerReady)
    {

    }
}

static inline void initGpioIdle(void)
{
    DDRB &= ~(SDA|SCL);
    PUEB |= SDA|SCL;
}
static inline void SDADown(void)
{
    PORTB &= ~(SDA);
    DDRB |= SDA;
    PUEB &= ~(SDA);
}
static inline void SCLDown(void)
{
    PORTB &= ~(SCL);
    DDRB |= SCL;
    PUEB &= ~(SCL);
}

static inline void SDAUp(void)
{
    DDRB &= ~(SDA);
    PUEB |= SDA;
}

static inline void SCLUp(void)
{
    DDRB &= ~(SCL);
    PUEB |= SCL;
}


static inline void initTim0(void)
{
    OCR0A = 159; /* ~50Khz */
    TCCR0B = _BV(CS00) | _BV(WGM02); /* no prescaler, CTC mode, | mode is not used because this is the only access*/
    TIMSK0 = _BV(OCIE0A);
}

static inline void deinitTim0(void)
{
    TCCR0B = 0;
}

static inline void start(void)
{
    initGpioIdle();
    waitTimer();
    SDADown();
    waitTimer();
    SCLDown();
}

static inline  void stop(void)
{
    SDADown();
    waitTimer();
    SCLUp();
    waitTimer();
    SDAUp();
}

static inline uint8_t readSDA(void)
{
    return PINB & SDA;
}
/**
 * Sends byte to i2c. No start or stop conditions are issued
 * @param _byte byte itself
 * @return Returns 0 if there was NAK, !0 on ACK
 */
static uint8_t sendByte(const uint8_t _byte)
{
    for (uint8_t bit = 0 ; bit < 8 ; bit++)
    {
        if ( 0 == (_byte & (0x80 >> bit)))
        {
            SDADown();
        }
        else
        {
            SDAUp();
        }
        waitTimer();
        SCLUp();
        waitTimer();
        SCLDown();
    }
    /* Waiting for ACK */
    waitTimer();
    SDAUp();
    waitTimer();
    SCLUp();
    waitTimer();
    const uint8_t retval = !readSDA();
    SCLDown();
    return retval;
}

static inline void sendRegData(void)
{
    for (uint8_t reg = 0; reg < SI5351A_REVB_REG_CONFIG_NUM_REGS; reg++)
    {
        uint8_t error = 0xFF;
        start();
        waitTimer();
        if (
                (0 != sendByte(ADDR << 1))  &&
                (0 != sendByte(si5351a_revb_registers[reg].address)) &&
                (0 != sendByte(si5351a_revb_registers[reg].value))
            )
        {
            error = 0;
        }
        stop();
        waitTimer();
        if (0 != error)
        {
            break;
        }
    }
}

static inline void waitms(const uint16_t _ms)
{
    for (uint32_t cycle = 0; cycle < _ms * 50;cycle++)
    {
        waitTimer();
    }
}

static inline void powerDown(void)
{
    SMCR = _BV(SM1);
}

void main()
{
    clock_prescale_set(clock_div_1); /* Using 8MHz RC and div1 = 8MHZ clock which is safe for 3.3v */
    initTim0();
    sei();
    waitms(10);
    sendRegData();
    waitms(10000);/* For connecting by isp */
    deinitTim0();
    powerDown();

    while(1)
    {
        sleep_cpu(); /* going to sleep for minimising noise */
    }

}
