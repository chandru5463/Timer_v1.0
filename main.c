/*===========================================================================
 * main.c — LED Timer Control + TM1637 3-Digit 7-Segment Display
 * Target : MS51 @ 16 MHz (HIRC)
 *
 * Keys 1-9  →  5 s – 45 s relay ON time  (key × 5 seconds)
 *
 * Display state machine
 * ─────────────────────
 *  BOOT      :  " On"  shown for 2 s  (TM1637 splash)
 *  IDLE      :  "000"  shown until a valid key is pressed
 *  COUNTDOWN :  remaining seconds (ceil, 3 digits, e.g. "045" → "001")
 *  DONE      :  "OFF"  shown until the next valid key press
 *
 * Pin map
 * ───────
 *  RELAY/LED  P0.5
 *  ROW1-4     P0.4, P0.3, P0.1, P0.0   (push-pull output)
 *  COL1-3     P1.0, P1.1, P1.2          (quasi-bidir, ext. pull-down)
 *  TM_CLK     P1.3                       (quasi-bidir, ext. 10 kΩ pull-up)
 *  TM_DIO     P1.4                       (quasi-bidir, ext. 10 kΩ pull-up)
 *===========================================================================*/

#include "numicro_8051.h"
#include "timer.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  PIN DEFINITIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define RELAY_LED   P05                /* Output: relay / LED                 */

/* Direct Buttons Mappings */
#define BTN1_PIN    P01                /* 5 min                               */
#define BTN2_PIN    P03                /* 10 min                              */
#define BTN3_PIN    P04                /* 15 min                              */
#define BTN4_PIN    P00                /* 20 min                              */
#define BTN5_PIN    P10                /* 25 min                              */
#define BTN6_PIN    P11                /* 30 min                              */
#define BTN7_PIN    P12                /* 40 min                              */
#define BTN8_PIN    P16                /* 60 min                              */
#define BTN9_PIN    P17                /* Emergency Stop / OFF                */

/* Button Configuration
 * Set BUTTON_ACTIVE_STATE to:
 *   0 : Active-Low (button connects pin to GND when pressed, e.g. using internal pull-up)
 *   1 : Active-High (button connects pin to VCC when pressed, e.g. using external pull-down)
 */
#define BUTTON_ACTIVE_STATE  1

#define READ_BTN(pin)        (BUTTON_ACTIVE_STATE ? ((pin) == 1) : ((pin) == 0))

/* TM1637 two-wire bus (external 10 kΩ pull-ups to 3.3 V / 5 V on both)     */
#define TM_CLK      P13                /* Serial clock                        */
#define TM_DIO      P14                /* Serial data I/O                     */

/* ═══════════════════════════════════════════════════════════════════════════
 *  TIMER 1 — 1 ms countdown  (ISR at vector 0x1B, interrupt 3)
 * ═══════════════════════════════════════════════════════════════════════════ */

volatile uint32_t g_ms_countdown = 0;   /* decremented each 1 ms by ISR      */
uint16_t g_prev_button_state = 0x0000;  /* last active-state tracking of BTN1-9 */

void Timer1_ISR(void) __interrupt(3)
{
    uint8_t sfrs_tmp = SFRS;
    SFRS = 0;

    TH1 = TH1TMP;                      /* reload for next 1 ms period        */
    TL1 = TL1TMP;
    clr_TCON_TF1;

    if (g_ms_countdown > 0)
        g_ms_countdown--;

    if (sfrs_tmp) { ENABLE_SFR_PAGE1; }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SOFTWARE DELAY — keypad debounce and splash screen timing only
 *  (~267 inner iterations ≈ 1 ms at 16 MHz; does NOT affect LED accuracy)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void Delay_ms_soft(uint16_t ms)
{
    volatile uint16_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 267; j++);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TM1637 DRIVER
 *
 *  Protocol summary (from Titan Micro TM1637 Datasheet V2.4)
 *  ──────────────────────────────────────────────────────────
 *  • START  : DIO HIGH → LOW while CLK is HIGH
 *  • STOP   : DIO LOW  → HIGH while CLK is HIGH
 *  • Data   : changed on CLK LOW, sampled on CLK rising edge, LSB first
 *  • ACK    : chip pulls DIO LOW on 8th CLK falling edge; released on 9th CLK
 *  • Max CLK: 500 kHz  (datasheet §4, Fmax)
 *
 *  Write flow (auto-increment mode)
 *  ─────────────────────────────────
 *   START → 0x40 → STOP          (Data Command: write, auto-increment)
 *   START → 0xC0 → d0 → d1 → d2 → STOP  (Address + 3 bytes of segment data)
 *   START → 0x8x → STOP          (Display Control: ON + brightness)
 *
 *  GPIO mode: quasi-bidirectional on both CLK and DIO.
 *   • Quasi-bidir acts as open-drain with weak pull (external 10 kΩ handles it).
 *   • Writing 1 → releases pin (external pull-up pulls HIGH).
 *   • Writing 0 → drives pin LOW.
 *   • Readable when set to 1 (needed for ACK on DIO).
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Segment encoding (common anode, SEG-A in bit 0) ──────────────────────
 *
 *    bit 0 = a  (top)
 *    bit 1 = b  (upper-right)
 *    bit 2 = c  (lower-right)
 *    bit 3 = d  (bottom)
 *    bit 4 = e  (lower-left)
 *    bit 5 = f  (upper-left)
 *    bit 6 = g  (middle)
 *    bit 7 = dp (decimal point — not connected on most 3-digit modules)
 * ──────────────────────────────────────────────────────────────────────── */

static const uint8_t SEG_TABLE[10] =
{
    0x3F,  /* 0 :  a b c d e f       */
    0x06,  /* 1 :    b c             */
    0x5B,  /* 2 :  a b   d e   g     */
    0x4F,  /* 3 :  a b c d     g     */
    0x66,  /* 4 :    b c     f g     */
    0x6D,  /* 5 :  a   c d   f g     */
    0x7D,  /* 6 :  a   c d e f g     */
    0x07,  /* 7 :  a b c             */
    0x7F,  /* 8 :  a b c d e f g     */
    0x6F   /* 9 :  a b c d   f g     */
};

/*  Special patterns used for "On" and "OFF" splash states                   */
#define SEG_BLANK   0x00U     /*  (all off)                                  */
#define SEG_O       0x3FU     /*  O  — segments a,b,c,d,e,f  (same as '0')  */
#define SEG_n       0x54U     /*  n  — segments c,e,g  (lower n shape)       */
#define SEG_F       0x71U     /*  F  — segments a,e,f,g                      */

/*  TM1637 command bytes                                                      */
#define TM_CMD_DATA_AUTO   0x40U   /* write data, auto-increment address     */
#define TM_CMD_ADDR_C0     0xC0U   /* starting address = GRID1               */
#define TM_CMD_DISP_ON     0x8BU   /* display ON, brightness = 10/16 pulse   */

/* ── TM_Delay ───────────────────────────────────────────────────────────────
 * ~5 µs at 16 MHz → effective CLK ≈ 100 kHz (datasheet max: 500 kHz)
 * The extra margin protects against worst-case 8051 instruction timing.
 * ─────────────────────────────────────────────────────────────────────────*/
static void TM_Delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 10; i++);
}

/* ── START condition ────────────────────────────────────────────────────── */
static void TM_Start(void)
{
    TM_DIO = 1;  TM_CLK = 1;  TM_Delay();
    TM_DIO = 0;               TM_Delay();  /* DIO HIGH→LOW while CLK HIGH   */
    TM_CLK = 0;               TM_Delay();
}

/* ── STOP condition ─────────────────────────────────────────────────────── */
static void TM_Stop(void)
{
    TM_CLK = 0;  TM_DIO = 0;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_DIO = 1;               TM_Delay();  /* DIO LOW→HIGH while CLK HIGH   */
}

/* ── Write one byte LSB-first; clock through ACK on 9th pulse ──────────────
 *
 *  The TM1637 pulls DIO LOW during the ACK slot (8th CLK falling edge).
 *  We release DIO = 1 on the 9th clock and do not check the ACK level —
 *  acceptable for display-only use; omitting the read simplifies the driver
 *  and avoids any quasi-bidir read-back glitch concerns.
 * ─────────────────────────────────────────────────────────────────────────*/
static void TM_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        TM_CLK = 0;                    TM_Delay();
        TM_DIO = (data & 0x01U) ? 1 : 0;  /* set bit while CLK low          */
                                       TM_Delay();
        TM_CLK = 1;                    TM_Delay();  /* chip latches on rising edge */
        data >>= 1;
    }

    /* 9th clock: release DIO so chip can drive ACK (we ignore its value)    */
    TM_CLK = 0;  TM_DIO = 1;  TM_Delay();
    TM_CLK = 1;               TM_Delay();
    TM_CLK = 0;               TM_Delay();
}

/* ── TM_ShowDigits ───────────────────────────────────────────────────────────
 *  Core display routine.
 *  d0 → GRID1 (leftmost), d1 → GRID2 (middle), d2 → GRID3 (rightmost)
 *
 *  Follows the auto-increment flow documented in the TM1637 datasheet:
 *    1. Send Data Command  (0x40 — write mode, auto-increment)
 *    2. Send Address 0xC0  followed by 3 segment bytes (GRID1..GRID3)
 *    3. Send Display Control command (ON + brightness)
 * ─────────────────────────────────────────────────────────────────────────*/
static void TM_ShowDigits(uint8_t d0, uint8_t d1, uint8_t d2)
{
    /* Step 1 — Data Command */
    TM_Start();
    TM_WriteByte(TM_CMD_DATA_AUTO);
    TM_Stop();

    /* Step 2 — Starting address + segment data */
    TM_Start();
    TM_WriteByte(TM_CMD_ADDR_C0);
    TM_WriteByte(d0);
    TM_WriteByte(d1);
    TM_WriteByte(d2);
    TM_Stop();

    /* Step 3 — Display Control: ON, brightness = 10/16 */
    TM_Start();
    TM_WriteByte(TM_CMD_DISP_ON);
    TM_Stop();
}

/* ── High-level display helpers ─────────────────────────────────────────── */

/*  Show remaining seconds as a 3-digit decimal (000 – 999)
 *  Uses ceiling division so the display reads the full second at start
 *  and transitions as each second expires, not partway through.            */
static void TM_ShowSeconds(uint32_t secs)
{
    if (secs > 999UL) secs = 999UL;                     /* clamp for safety */
    TM_ShowDigits(
        SEG_TABLE[secs / 100UL],
        SEG_TABLE[(secs / 10UL) % 10UL],
        SEG_TABLE[secs % 10UL]
    );
}

/*  " On" — power-on splash (blank + O + n)                                  */
static void TM_ShowON(void)
{
    TM_ShowDigits(SEG_O, SEG_n,SEG_BLANK);
}

/*  "OFF" — timer complete, waiting for next key                             */
static void TM_ShowOFF(void)
{
    TM_ShowDigits(SEG_O, SEG_F, SEG_F);
}

/*  "000" — idle/ready state shown after splash                              */
static void TM_ShowZeros(void)
{
    TM_ShowDigits(SEG_TABLE[0], SEG_TABLE[0], SEG_TABLE[0]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DIRECT BUTTON SCANNER (Edge-Triggered, Non-blocking)
 *  Returns 1–9 for a confirmed transition from released to pressed.
 *  Returns 0xFF if no transition is detected.
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t Scan_Buttons(void)
{
    uint16_t current_state = 0;
    uint8_t pressed_btn = 0xFFU;
    uint8_t i;

    /* Read all 9 pins into a single bitmask (1 = pressed, 0 = released) */
    current_state |= (READ_BTN(BTN1_PIN) ? 1U : 0) << 0;
    current_state |= (READ_BTN(BTN2_PIN) ? 1U : 0) << 1;
    current_state |= (READ_BTN(BTN3_PIN) ? 1U : 0) << 2;
    current_state |= (READ_BTN(BTN4_PIN) ? 1U : 0) << 3;
    current_state |= (READ_BTN(BTN5_PIN) ? 1U : 0) << 4;
    current_state |= (READ_BTN(BTN6_PIN) ? 1U : 0) << 5;
    current_state |= (READ_BTN(BTN7_PIN) ? 1U : 0) << 6;
    current_state |= (READ_BTN(BTN8_PIN) ? 1U : 0) << 7;
    current_state |= (READ_BTN(BTN9_PIN) ? 1U : 0) << 8;

    /* Detect rising edge of active state: was released (0) -> is pressed (1) */
    for (i = 0; i < 9; i++)
    {
        uint16_t mask = 1U << i;
        if (!(g_prev_button_state & mask) && (current_state & mask))
        {
            /* Debounce: wait 10 ms and re-read */
            Delay_ms_soft(10);
            
            /* Re-read the specific pin */
            uint8_t pin_pressed = 0;
            switch (i + 1)
            {
                case 1: pin_pressed = READ_BTN(BTN1_PIN); break;
                case 2: pin_pressed = READ_BTN(BTN2_PIN); break;
                case 3: pin_pressed = READ_BTN(BTN3_PIN); break;
                case 4: pin_pressed = READ_BTN(BTN4_PIN); break;
                case 5: pin_pressed = READ_BTN(BTN5_PIN); break;
                case 6: pin_pressed = READ_BTN(BTN6_PIN); break;
                case 7: pin_pressed = READ_BTN(BTN7_PIN); break;
                case 8: pin_pressed = READ_BTN(BTN8_PIN); break;
                case 9: pin_pressed = READ_BTN(BTN9_PIN); break;
            }
            
            if (pin_pressed)
            {
                pressed_btn = i + 1;
                /* Lock the state as pressed so we don't repeat-trigger */
                g_prev_button_state |= mask;
                break;
            }
        }
    }

    /* Release tracking: clear bits in g_prev_button_state if pins are released (0 in current_state) */
    g_prev_button_state &= current_state;

    return pressed_btn;
}

void main(void)
{
    /* ── Clock: switch to internal 16 MHz HIRC ──────────────────────────── */
    TA = 0xAA; TA = 0x55; CKEN  |=  0x20;   /* enable HIRC oscillator        */
    TA = 0xAA; TA = 0x55; CKSWT &= ~0x07;   /* select HIRC as system clock   */

    /* ── GPIO Configuration ─────────────────────────────────────────────── */

    /* P0.5  RELAY_LED — push-pull output                                     */
    P0M1 &= ~0x20U;
    P0M2 |=  0x20U;

    /* P0.0, P0.1, P0.3, P0.4  BTN4, BTN1, BTN2, BTN3 — quasi-bidirectional (mask = 0x1B) */
    P0M1 &= ~0x1BU;
    P0M2 &= ~0x1BU;

    /* P1.0, P1.1, P1.2, P1.6, P1.7  BTN5-9 — quasi-bidirectional (mask = 0xC7) */
    P1M1 &= ~0xC7U;
    P1M2 &= ~0xC7U;

    /* P1.3 (TM_CLK), P1.4 (TM_DIO) — quasi-bidirectional  (mask = 0x18)
     * Quasi-bidir acts as open-drain with weak internal pull.
     * External 10 kΩ pull-ups supply the HIGH level per TM1637 spec.        */
    P1M1 &= ~0x18U;
    P1M2 &= ~0x18U;

    /* ── Initial pin states ─────────────────────────────────────────────── */
    RELAY_LED = 0;            /* relay / LED OFF at boot                      */
    TM_CLK    = 1;            /* bus idle: both lines HIGH                    */
    TM_DIO    = 1;
    BTN1_PIN  = 1;            /* Enable input / pull-ups on all buttons       */
    BTN2_PIN  = 1;
    BTN3_PIN  = 1;
    BTN4_PIN  = 1;
    BTN5_PIN  = 1;
    BTN6_PIN  = 1;
    BTN7_PIN  = 1;
    BTN8_PIN  = 1;
    BTN9_PIN  = 1;

    /* Read initial button state to prevent false triggers on startup */
    g_prev_button_state = 0;
    g_prev_button_state |= (READ_BTN(BTN1_PIN) ? 1U : 0) << 0;
    g_prev_button_state |= (READ_BTN(BTN2_PIN) ? 1U : 0) << 1;
    g_prev_button_state |= (READ_BTN(BTN3_PIN) ? 1U : 0) << 2;
    g_prev_button_state |= (READ_BTN(BTN4_PIN) ? 1U : 0) << 3;
    g_prev_button_state |= (READ_BTN(BTN5_PIN) ? 1U : 0) << 4;
    g_prev_button_state |= (READ_BTN(BTN6_PIN) ? 1U : 0) << 5;
    g_prev_button_state |= (READ_BTN(BTN7_PIN) ? 1U : 0) << 6;
    g_prev_button_state |= (READ_BTN(BTN8_PIN) ? 1U : 0) << 7;
    g_prev_button_state |= (READ_BTN(BTN9_PIN) ? 1U : 0) << 8;

    /* If BTN9 is held down at boot, enter diagnostic mode */
    if (READ_BTN(BTN9_PIN))
    {
        while (1)
        {
            uint8_t d0 = (READ_BTN(BTN1_PIN) ? 0x01 : 0) | (READ_BTN(BTN2_PIN) ? 0x02 : 0) | (READ_BTN(BTN3_PIN) ? 0x04 : 0);
            uint8_t d1 = (READ_BTN(BTN4_PIN) ? 0x01 : 0) | (READ_BTN(BTN5_PIN) ? 0x02 : 0) | (READ_BTN(BTN6_PIN) ? 0x04 : 0);
            uint8_t d2 = (READ_BTN(BTN7_PIN) ? 0x01 : 0) | (READ_BTN(BTN8_PIN) ? 0x02 : 0) | (READ_BTN(BTN9_PIN) ? 0x04 : 0);
            TM_ShowDigits(d0, d1, d2);
            Delay_ms_soft(50);
        }
    }

    /* ── Timer 1: 1 ms interrupt base ───────────────────────────────────── */
    /*  Reload value = 65535 − (16 000 000 / 12 / 1000) ≈ 64202             */
    Timer1_AutoReload_Interrupt_Initial(16, 1000);
    EA = 1;                   /* global interrupt enable                      */

    /* ══════════════════════════════════════════════════════════════════════
     *  POWER-ON SEQUENCE
     *  1.  " On" splash for 2 s
     *  2.  "000" idle — wait for first key press
     * ════════════════════════════════════════════════════════════════════ */
    TM_ShowON();
    Delay_ms_soft(3000U);

    TM_ShowZeros();

    /* ══════════════════════════════════════════════════════════════════════
     *  MAIN LOOP
     * ════════════════════════════════════════════════════════════════════ */
    while (1)
    {
        uint8_t key = Scan_Buttons();

        /* Emergency Stop Logic: If Key 9 is pressed while IDLE or DONE */
        if (key == 9U)
        {
            RELAY_LED = 0;
            g_ms_countdown = 0;
            TM_ShowOFF();
            continue;
        }

        /* Valid Timer Keys: 1 to 8 mapped to Minutes */
        if (key >= 1U && key <= 8U)
        {
            uint32_t ms_on;
            uint32_t prev_min = 0xFFFFFFFFUL; /* Tracks minute changes for display */

            /* ── Mapping Table 1->8 for 5, 10, 15, 20, 25, 30, 40, 60 MINUTES ── */
            switch(key) {
                case 1: ms_on = 5UL  * 60000UL; break;
                case 2: ms_on = 10UL * 60000UL; break;
                case 3: ms_on = 15UL * 60000UL; break;
                case 4: ms_on = 20UL * 60000UL; break;
                case 5: ms_on = 25UL * 60000UL; break;
                case 6: ms_on = 30UL * 60000UL; break;
                case 7: ms_on = 40UL * 60000UL; break;
                case 8: ms_on = 60UL * 60000UL; break;
                default: ms_on = 0; break;
            }

            /* ── Load countdown (IRQ-safe) ── */
            EA = 0;
            g_ms_countdown = ms_on;
            EA = 1;

            RELAY_LED = 1;          /* Activate Relay */

            /* ── Countdown loop ── */
            while (g_ms_countdown > 0)
            {
                uint32_t ms_now;
                uint32_t min_now;
                uint8_t stop_key;

                /* CHECK FOR EMERGENCY STOP DURING COUNTDOWN */
                stop_key = Scan_Buttons();
                if (stop_key == 9U)
                {
                    EA = 0;
                    g_ms_countdown = 0;
                    EA = 1;
                    break;
                }

                EA = 0;
                ms_now = g_ms_countdown;
                EA = 1;

                /* Calculate Minutes Remaining (Ceiling)
                   Example: 300,000ms (5 mins) / 60,000 = 5. Display shows 005.
                   At 239,000ms (3.98 mins), (239000+59999)/60000 = 4. Display shows 004. */
                min_now = (ms_now + 59999UL) / 60000UL;

                if (min_now != prev_min)
                {
                    prev_min = min_now;
                    /* We reuse the TM_ShowSeconds function logic but pass minutes
                       since it just handles formatting 3-digit integers. */
                    TM_ShowSeconds(min_now);
                }
            }

            RELAY_LED = 0;          /* Deactivate Relay */
            TM_ShowOFF();
        }
    }
}
