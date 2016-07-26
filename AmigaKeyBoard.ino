#include <Mouse.h>
#include <HID.h>
#include <Arduino.h>

#if ARDUINO >= 10606
#include <Keyboard.h>
#define HID_SendReport(id,data,len) HID().SendReport(id,data,len)
#endif

/*
 * Keyboard   Color     Leonardo IO
Connector

1   KBDATA    BLACK     8
2   KBCLK     BROWN     9
3   KBRST     RED       10
4   5v        ORANGE    5v
5   NC        -----
6   GND       GREEN     GND
7   LED1      BLUE      5V
8   LED2      PURPLE    -

 */

#define BITMASK_A500SP      0b00010000    // IO 8
#define BITMASK_A500CLK     0b00100000    // IO 9
#define BITMASK_A500RES     0b01000000    // IO 10
#define BITMASK_JOY1        0b10011111    // IO 0..4,6
#define BITMASK_JOY2        0b11110011    // IO A0..A5
#define BITMASK_A500_OUT    0b10000000    //IO 13
#define SYNCH_HI        0
#define SYNCH_LO        1
#define HANDSHAKE       2
#define READ            3
#define WAIT_LO         4
#define WAIT_RES        5 //Waiting response
#define OUTPUT_AMIGA    7
#define OUTPUT_USB      8

KeyReport _keyReport;
uint32_t counter = 0;
uint8_t Joy, MemoJoy1, MemoJoy2, state, switch_to, bitn, bitA, key, help_key, amiga_keypress, keydown, ktab[0x68] = {
        // Tilde, 1-9, 0, sz, Accent, backslash, Num 0 (00 - 0F)
        0x35, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x31,    0, 0x62,
        // Q bis +, -, Num 1, Num 2, Num3 (10 - 1F)
        0x14, 0x1A, 0x08, 0x15, 0x17, 0x1C, 0x18, 0x0C, 0x12, 0x13, 0x2F, 0x30, 0   , 0x59, 0x5A, 0x5B,
        // A-#, -, Num 4, Num 5, Num 6 (20 - 2F)
        0x04, 0x16, 0x07, 0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x33, 0x34, 0x32, 0,    0x5C, 0x5D, 0x5E,
        // <>,Y- -, -, Num . , Num 7, Num 8, Num 9 (30 - 3F)
        0x64, 0x1D, 0x1B, 0x06, 0x19, 0x05, 0x11, 0x10, 0x36, 0x37, 0x38,    0, 0x63, 0x5F, 0x60, 0x61,
        // Space, BS, Tab, Enter, Return, ESC, Del, -, -, -, Num -, -, up, down, right, left (40 - 4F)
        0x2C, 0x2A, 0x2B, 0x58, 0x28, 0x29, 0x4C,    0,    0,    0, 0x56,    0, 0x52, 0x51, 0x4F, 0x50,
        // F1-F10, -, -, Num /, Num *, Num +, - (50 - 5F)
        0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43,    0,    0, 0x54, 0x55, 0x57,    0,
        // modifiers: Shift left, Shift right, -, Crtl left, Alt left, Alt right, Win (amiga) left, Ctrl (amiga)right
        0x02, 0x20, 0x00, 0x01, 0x04, 0x40, 0x08, 0x10
};

void setup() {
    // Joystick 1 (Port D)
    DDRD = ~BITMASK_JOY1; // direction INPUT
    PORTD = BITMASK_JOY1; // activate PULLUP

    // Joystick 2 (Port F)
    DDRF = ~BITMASK_JOY2; // direction INPUT
    PORTF = BITMASK_JOY2; // activate PULLUP

    DDRC |= BITMASK_A500_OUT;   // set IO direction to OUTPUT
    PORTC &= ~BITMASK_A500_OUT; // set OUTPUT to LOW

    // Keyboard (Port B)
    /*
    * 0010000    // IO 8
    * 0100000    // IO 9
    * 1000000    // IO 10
    * -------
    * 1110000 ~ 0001111 Sets Pins 8,9,10 as INPUTS(0) on the port register and 11,12,13 as OUTPUTS(1)
    */
    DDRB = ~(BITMASK_A500SP | BITMASK_A500CLK | BITMASK_A500RES);  // direction INPUT

}


void loop() {
    // Joystick 1
    Joy = ~PIND & BITMASK_JOY1;
    if (Joy != MemoJoy1) {
        HID_SendReport(3, &Joy, 1);
        MemoJoy1 = Joy;
    }

    // Joystick 2
    Joy = ~PINF & BITMASK_JOY2;
    if (Joy != MemoJoy2) {
        HID_SendReport(4, &Joy, 1);
        MemoJoy2 = Joy;
    }

    // Keyboard
    // Read if PINB & BITMASK_A500RES is LOW(0) or HIGH(1), if it was 1 then reboot otherwise perform the key press action
    if (((PINB & BITMASK_A500RES) == 0) && state != WAIT_RES) { // Reset
        reboot();
    } else {
        perform();
    }

}

/**
 * RED cable on Amiga keyboard pinout is set as HIGH when Ctrl+Amiga+Amiga key modifiers are pressed.
 * Actions: Enables interruptions, Send CTRL+ALT+KEY modifiers through the output, reset flags, set state as Wait for reset
 */
void reboot() {
    interrupts();
    keystroke(0x4C, 05);       // CTRL+ALT+DEL
    help_key = 0;     // Reset "Help" key flag
    amiga_keypress = 0; // Reset "Amiga" key flag
    state = WAIT_RES;
}

/**
*
*/
void perform() {
    // (PINB & BITMASK_A500RES) == digitalRead(10) => Reads INPUT Pin 10
    switch (state) {
        case WAIT_RES:     if ((PINB & BITMASK_A500RES) != 0) state = SYNCH_HI; break;  // Waiting for reset end
        case SYNCH_HI:     if ((PINB & BITMASK_A500CLK) == 0) state = SYNCH_LO; break;  // Sync-Pulse HI
        case SYNCH_LO:     if ((PINB & BITMASK_A500CLK) != 0) state = HANDSHAKE; break; // Sync-Pulse LOW
        case HANDSHAKE:    handShake(); break; // Handshake
        case WAIT_LO:      waitClockCycle(); break; // waiting for the next bit
        case READ:         readKey(); break;      // read key message (8 bits)
    }
}


/**
*
*/
void handShake() {
    if (counter == 0) {
        DDRB |= BITMASK_A500SP;   // set IO direction to OUTPUT
        PORTB &= ~BITMASK_A500SP; // set OUTPUT to LOW
        counter = millis();
    }
    else if (millis() - counter > 10) {
        counter = 0;
        DDRB &= ~BITMASK_A500SP;   // set IO direction to INPUT
        state = WAIT_LO;
        key = 0;
        bitn = 7;
    }
}

/**
*
*/
void waitClockCycle() {
    if ((PINB & BITMASK_A500CLK) == 0) {
        noInterrupts();
        state = READ;
    }
}


/**
*
*/
void readKey() {

    if ((PINB & BITMASK_A500CLK) != 0) {
        if (bitn--) {
            key += ((PINB & BITMASK_A500SP) == 0) << (bitn); // key code (add bits 0...6)

            if (switch_to = OUTPUT_AMIGA) {
                if ((PINB & BITMASK_A500SP)==0) {
                    PORTC &= ~BITMASK_A500_OUT; // set OUTPUT to LOW
                }else{
                    PORTC &= BITMASK_A500_OUT; // set OUTPUT to HIGH
                }
            }

            state = WAIT_LO;
        }
        else {  // read last bit (key down)
            keydown = ((PINB & BITMASK_A500SP) != 0); // true if key down
            interrupts();
            state = HANDSHAKE;
            mappingKey(key);
        }
    }
}



/**
*
*/
void mappingKey(uint8_t k) {
    if (k == 0x5F)  help_key = keydown; // "Help" key: special function on/off
    else if (k == 0x66) amiga_keypress = keydown; // "A" Key has been pressed, will used it to drive key press through either USB or straight to amiga motherboard.
    else if (k == 0x62) keystroke(0x39, 0x00); // CapsLock
    else {
        if (amiga_keypress) {
            //Plase here the actions that will happnen if amiga key is pressed
            amigaKeyPress(k);
        }else if (keydown && (switch_to==OUTPUT_USB)) {
            // keydown message received------
            if (help_key) {
                helpKeyPress(k);
            } else {
                if (k == 0x5A) keystroke(0x26, 0x20); // (
                else if (k == 0x5B) keystroke(0x27, 0x20); // )
                else if (k < 0x68) keypress(k);  // Code table
            }
        }
        else {
            // keyrelease message received
            if (k < 0x68) keyrelease(k);  // Code table
        }
    }
}


/**
*
*/
void helpKeyPress(uint8_t k) {
    // special function with "Help" key
    switch (k) {
        case 0x50: keystroke(0x44, 0); break; // F11
        case 0x51: keystroke(0x45, 0); break; // F12
        case 0x5A: keystroke(0x53, 0); break; // NumLock
        case 0x5B: keystroke(0x47, 0); break; // ScrollLock
        case 0x5D: keystroke(0x46, 0); break; // PrtSc
    }
}

/**
*
*/
void amigaKeyPress(uint8_t k) {
    if (k == 0x21) {
        switch_to = (switch_to == OUTPUT_AMIGA) ? OUTPUT_USB : OUTPUT_AMIGA;
    }
}

/*
 * User has set up keys through Amiga motherboard, pressing (Amiga + s)
 */
void sendKeyToAmigaBoard() {

    if (switch_to = OUTPUT_AMIGA) {
        if ((PINB & BITMASK_A500SP)==0) {
            PORTC &= ~BITMASK_A500_OUT; // set OUTPUT to LOW
        }else{
            PORTC &= BITMASK_A500_OUT; // set OUTPUT to HIGH
        }
    }
}

/**
*
*/
void keypress(uint8_t k) {

    if (k > 0x5f) _keyReport.modifiers |= ktab[key];  // modifier
    else {
        for (uint8_t i = 0; i < 6; i++) {
            if (_keyReport.keys[i] == 0) {
                _keyReport.keys[i] = ktab[key];
                break;
            }
        }
    }
    HID_SendReport(2, &_keyReport, 8); //void HID_SendReport(uint8_t id, const void* data, int len);
}

/**
*
*/
void keyrelease(uint8_t k) {

    if (switch_to==OUTPUT_USB) {
        keyReleaseUSB(k);
    } else {
        keyReleaseAMIGA();
    }
}

void keyReleaseUSB(uint8_t k){

    if (k > 0x5f) _keyReport.modifiers &= ~ktab[key];  // modifier
    else {
        for (uint8_t i = 0; i < 6; i++) {
            if (_keyReport.keys[i] == ktab[key]) _keyReport.keys[i] = 0;
        }
    }
    HID_SendReport(2, &_keyReport, 8); //void HID_SendReport(uint8_t id, const void* data, int len);
}

void keyReleaseAMIGA(){
    for (uint8_t i = 0; i < 6; i++) {
        PORTC &= ~BITMASK_A500_OUT; // set OUTPUT to LOW
    }
}

/**
*
*/
void keystroke(uint8_t k, uint8_t m) {

    unsigned short memomodifiers = _keyReport.modifiers; // save last modifier state
    for (uint8_t i = 0; i < 6; i++) {
        if (_keyReport.keys[i] == 0) {
            _keyReport.keys[i] = k;
            _keyReport.modifiers = m;
            HID_SendReport(2, &_keyReport, 8);
            _keyReport.keys[i] = 0;
            _keyReport.modifiers = memomodifiers; // recover modifier state
            HID_SendReport(2, &_keyReport, 8); //void  HID_SendReport(uint8_t id, const void* data, int len);
            break;
        }
    }
}
