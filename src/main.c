/*
 * File:   main.c
 * Author: Dac Nguyen
 *
 * Created on May 13, 2021, 12:51 PM
 */

#include <avr/io.h>
#include <xc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define GPIOA 0x12 //expander port
#define GPIOB 0x13 //expander port
#define INTCAPB 0x11
#define interruptExA 0b00000100
#define interruptExB 0b00001000
//Sensor Button Address ExpanderA
#define butSouthMain 0b00000001
#define butSouthTurn 0b00000010
#define butSouthTram 0b00000100
#define butNorthBus  0b00001000
#define butNorthMain 0b00010000
#define butNorthTram 0b00100000
#define butLS        0b01000000
#define butPedNorth  0b10000000
//Sensor Button Address ExpanderB
#define butPedNSouth 0b00000001
#define butPedLS     0b00000010
uint8_t buttonA[8] = {1, 2, 3, 4, 1, 3, 5, 6};
uint8_t buttonB[2] = {6, 6};
//LED Address ExpanderA
#define STW   0b00000001
#define STR   0b00000010
#define TURNG 0b00000100
#define TURNY 0b00001000
#define TURNR 0b00010000
#define SMG   0b00100000
#define SMY   0b01000000
#define SMR   0b10000000
//LED Address ExpanderB
#define PEDG  0b00000001
#define PEDR  0b00000010
#define NTW   0b00000100
#define NTR   0b00001000
#define BUS   0b00010000
#define NMG   0b00100000
#define NMY   0b01000000
#define NMR   0b10000000
#define lSG   0b00100000
#define lSY   0b01000000
#define lSR   0b10000000
//State Name
uint8_t broadWayCarGo =      0; //=>1
uint8_t broadWayTurn =       0; //=>2
uint8_t LSTurn =             0; //=>5
uint8_t broadWayCarGoTram =  0; //=>3
uint8_t SouthGoBus =         0; //=>4
uint8_t ped =                0; //=>6
//StateTime
#define broadWayCarGoTime 10
#define broadWayTurnTime 5
#define LSTurnTime 5
#define broadWayCarGoTramTime 10
#define SouthGoBusTime 4
#define pedTime 10


//uint8_t queue[1000];
uint8_t STATE = 0;
/*
 * 00000001: tram
 * 00000010:bus
 * 00000100: car go
 * 00001000: broadway turn
 * 00010000: LSturn
 * 00100000:ped
 */
int head=0;
int tail=0;
uint8_t state = 0;
volatile long ms=0;
long int day=0;
int period = 300;

void setupPort();
void setupSPI();
void SPI_Send_Command(int id, uint8_t reg, uint8_t data);
uint8_t SPI_Read_Command(int id, uint8_t reg);
void setupInterupts();
void setupSerial();
void sendSerialString(const char *s);
void sendSerialNumber(uint8_t h);
void setup_Timer0();
void hazardState();

int main(void) {
    setupPort();
    setupSPI();
    setupSerial();
    sendSerialString("Serial Begin 3 \r\n");
    setupInterupts();
    setup_Timer0();
    sei();
    while(1){sleep_mode();}
}
//-----------------------------------Timer--------------------------------------
ISR(TIMER0_COMPA_vect)//for timer 
{
    ms++;
    day += ms==0 ? 1 : 0;
    static long preTime=0;
    static uint8_t yellow=0;
    static uint8_t green=0;
    static uint8_t current_state=0;
    static uint8_t is_in_default = 0;
    //sendSerialNumber(STATE);
    if (!current_state && STATE>0){
        //sendSerialString("hello\r\n");
        uint8_t number=1;
        while(!current_state){
            if (STATE>>(number-1)&1) current_state=number;
            number++;
        }
    }
    //this is default State - if no action - then turn off clock until butotn is trigged
    if (!STATE && is_in_default==0){
        //default State
        SPI_Send_Command(1, GPIOA, 0b00110010); 
        SPI_Send_Command(2, GPIOA, 0b00101010); 
        SPI_Send_Command(2, GPIOB, 0b10000000);
        is_in_default=1;   //true
    }  else if(!STATE && is_in_default==1) TCCR0B =0; //turn off timer1
    //change to State processing
    if (STATE>0 && is_in_default==1){
        uint8_t dataA1 = SPI_Read_Command(1, GPIOA);
        uint8_t dataA2 = SPI_Read_Command(2, GPIOA);
        if (preTime==0) {
            preTime=ms;
            //not tram and bus state then change yellow
            //if it is red, then change to yellow
            if (!(STATE>>0&1) && !(STATE>>1&1) && !(STATE>>2&1) && (dataA1>>5&1))
                SPI_Send_Command(1, GPIOA, 0b01010010); 
            if (!(STATE>>0&1) && !(STATE>>2&1) && (dataA2>>5&1)) SPI_Send_Command(2, GPIOA, 0b01001010); 
            is_in_default=0;
        }
        if (ms-preTime>=3*period){
            is_in_default=0;
            preTime=0;
        }
    } else if (STATE>0) {
        //State processing
        switch (current_state){
            case 1: //broadWayCarGoTram
                if (preTime==0) preTime=ms;
                if (ms-preTime>=(broadWayCarGoTramTime+4)*period){ //out of state 6=3+bustime+1
                    STATE&=0b11111110; preTime=0; yellow=0; green=0; current_state=0;
                    if (STATE>0) is_in_default=1; //??
                } else if (ms-preTime>=(broadWayCarGoTramTime+3)*period && yellow==0){ //turn yellow
                        yellow=1;
                        broadWayCarGoTram=0;
                        SPI_Send_Command(1, GPIOA, 0b00110010); //off tram
                        SPI_Send_Command(2, GPIOA, 0b00101010); //off tram
                } else if (ms-preTime>=3*period && green==0){ //turn green
                    broadWayCarGoTram=1; green=1;
                    SPI_Send_Command(1, GPIOA, 0b00110001);
                    SPI_Send_Command(2, GPIOA, 0b00100110); //TramL on, NM G SM G ON
                }
                break;
            case 2: //SouthGoBus
                if (preTime==0) preTime=ms;
                if (ms-preTime>=(SouthGoBusTime+4)*period){ //out of state 6=3+bustime+1
                    STATE&=0b11111101; preTime=0; yellow=0; green=0; current_state=0;
                    if (STATE>0) is_in_default=1;
                } else if (ms-preTime>=(SouthGoBusTime+3)*period && yellow==0){ //turn yellow
                        yellow=1;
                        SouthGoBus=0;
                        SPI_Send_Command(2, GPIOA, 0b10001010); //busL off
                } else if (ms-preTime>=3*period && green==0){ //turn green
                    SouthGoBus=1; green=1;
                    SPI_Send_Command(2, GPIOA, 0b10011010); //busL on, NM G off
                    if ((STATE>>0&1)||(STATE>>2&1)) //if turn, ls, ped (no need to turn light on)
                                                    //else -> turn light on.
                        SPI_Send_Command(1, GPIOA, 0b00110010); // SM G on
                }
                break;
            case 3: //broadWayCarGo                
                if (preTime==0) {
                    preTime=ms;
                }
                if (ms-preTime>=(broadWayCarGoTime+3)*period){ //turn yellow     
                    STATE&=0b11111011; preTime=0; current_state=0;
                    if (STATE>0) is_in_default=1;
                    broadWayCarGo=0; 
                }else if (ms-preTime>=(3)*period){ //turn yellow     
                    SPI_Send_Command(1, GPIOA, 0b00110010); 
                    SPI_Send_Command(2, GPIOA, 0b00101010); 
                    broadWayCarGo=1;
                }
                break;
            case 4: //broadWayTurn
                if (preTime==0) preTime=ms;
                if (ms-preTime>=((broadWayTurnTime+6)*period)){ //out of turn state
                    STATE&=0b11110111; preTime=0; yellow=0; green=0; current_state=0;
                    
                    /*
                    if ((STATE>>0&1) || (STATE>>2&1)) { //or tram, or car go
                        SPI_Send_Command(1, GPIOA, 0b00110010);
                        SPI_Send_Command(2, GPIOA, 0b00101010);
                    } else if (STATE>>1&1) //if bus turn the light
                        SPI_Send_Command(1, GPIOA, 0b00110010);*/
                    //else 
                    SPI_Send_Command(1, GPIOA, 0b10010010); 
                } else if (ms-preTime>=(broadWayTurnTime+3)*period && yellow==0){ //turn yellow     
                    yellow=1;
                    broadWayTurn=0; 
                    SPI_Send_Command(1, GPIOA, 0b10001010); 
                } else if (ms-preTime>=3*period && green==0){ //turn green
                    broadWayTurn=1; green=1;
                    SPI_Send_Command(1, GPIOA, 0b10000110); 
                    SPI_Send_Command(2, GPIOA, 0b10001010);
                }
                break;
            case 5: //LSTurn
                if (preTime==0) preTime=ms;
                if (ms-preTime>=((LSTurnTime+6)*period)){ //out of turn state
                    STATE&=0b11101111; yellow=0; green=0; preTime=0;current_state=0;
                    //if (STATE>0) is_in_default=1;
                    /*
                    if ((STATE>>0&1) || (STATE>>2&1)) { //or tram, or cargo
                        SPI_Send_Command(1, GPIOA, 0b00110010);
                        SPI_Send_Command(2, GPIOA, 0b00101010);
                    } else if (STATE>>1&1) //if bus turn the light
                        SPI_Send_Command(1, GPIOA, 0b00110010);*/
                    SPI_Send_Command(2, GPIOB, 0b10000000);
                } else if (ms-preTime>=(LSTurnTime+3)*period && yellow==0){ //turn yellow     
                        yellow=1;
                        LSTurn=0; 
                        SPI_Send_Command(2, GPIOB, 0b01000000); //
                } else if (ms-preTime>=3*period && green==0){ //turn green
                    LSTurn=1; green=1;
                    SPI_Send_Command(1, GPIOA, 0b10010010); //SM R on
                    SPI_Send_Command(2, GPIOA, 0b10001010); //
                    SPI_Send_Command(2, GPIOB, 0b00100000); 
                }
                break;       
            case 6: //ped
                if (preTime==0) preTime=ms;
                if (ms-preTime>=((pedTime+4)*period)){ //out of turn state
                    SPI_Send_Command(2, GPIOA, 0b10001010); //turn red light on PED
                    STATE&=0b11011111; yellow=0; green=0; preTime=0;current_state=0;
                    //if tram State or Bus State
                    /*
                    if ((STATE>>0&1) || (STATE>>2&1)) { //or tram, or cargo
                        SPI_Send_Command(1, GPIOA, 0b00110010);
                        SPI_Send_Command(2, GPIOA, 0b00101010);
                    } else if (STATE>>1&1) */ //if bus turn the light
                    SPI_Send_Command(1, GPIOA, 0b00110010);
                } else if (ms-preTime>=(pedTime+2.5)*period && yellow==5){
                    SPI_Send_Command(2, GPIOA, 0b10001001);
                    yellow++;
                } else if (ms-preTime>=(pedTime+2)*period && yellow==4){
                    SPI_Send_Command(2, GPIOA, 0b10001000);
                    yellow++;
                }else if (ms-preTime>=(pedTime+1.5)*period && yellow==3){
                    SPI_Send_Command(2, GPIOA, 0b10001001);
                    yellow++;
                } else if (ms-preTime>=(pedTime+1)*period && yellow==2){
                    SPI_Send_Command(2, GPIOA, 0b10001000);
                    yellow++;
                } else if (ms-preTime>=(pedTime+0.5)*period && yellow==1){
                    SPI_Send_Command(2, GPIOA, 0b10001001);
                    yellow++;
                } else if (ms-preTime>=(pedTime)*period && yellow==0){ //falshing     
                        yellow++; ped=0; 
                        SPI_Send_Command(2, GPIOA, 0b10001000);
                } else if (ms-preTime>=3*period && green==0){ //turn green
                    ped=1; green=1;
                    SPI_Send_Command(1, GPIOA, 0b10010010); //SM R on
                    SPI_Send_Command(2, GPIOA, 0b10001001); //turn on Ped light
                }
                break;
        }
    }
} 
ISR(TIMER2_COMPA_vect)//for main state processing
{
}
//------------------------------State Handler-----------------------------------
void StateHandler(int id)
{
    TCCR0B = 0b00000011; //change here
    uint8_t data = SPI_Read_Command(id, GPIOB);    
    //sendSerialNumber(data);
    switch (id) {
        case 1:
            if (!(data&1)) STATE |= 0b00000100; //main
            if (!((data>>1)&1)) STATE |= 0b00001000; //broad way turn
            if (!((data>>2)&1)) STATE |= 0b00000001; //tram
            if (!((data>>3)&1)) STATE |= 0b00000010; //bus
            if (!((data>>4)&1)) STATE |= 0b00000100; //main
            if (!((data>>5)&1)) STATE |= 0b00000001; //tram
            if (!((data>>6)&1)) STATE |= 0b00010000; //ls
            if (!((data>>7)&1)) STATE |= 0b00100000; //ped
            break;
        case 2:
            if (!(data&1)) STATE |= 0b00100000;
            if(!((data>>1)&1)) STATE |= 0b00100000;
            break;
    }
    //sendSerialNumber(STATE);
}
//--------------------------------Interrupt-------------------------------------
ISR (INT0_vect){StateHandler(1);}
ISR (INT1_vect){StateHandler(2);}
//--------------------------------functions-------------------------------------
void hazardState(){
    //00110010 gpioA1;
    //00101010 gpioA2
    //readGPIOB2 or vs 1
    SPI_Send_Command(1, GPIOA, 0b00110010); 
    SPI_Send_Command(2, GPIOA, 0b00101010); 
    SPI_Send_Command(2, GPIOB, SPI_Read_Command(2, GPIOB)|0b10000000); 
}
void setup_Timer0() {
    TCCR0B = 0;             // Turn off clock whilst configuring
    TCNT0 = 0;
    OCR0A = 249;            // value to match (0 - 249) = 250 ticks
    TIFR0 = 0xff;           // clear all existing flags
    TIMSK0 = _BV(OCIE0A);   // enable interrupt on CTC match
    TCCR0A = 0b00000010;    // WGM01 = 1, WGM00 = 0
    TCCR0B = 0b00000011;    // CS02 = 0, CS01 = 1, CS00 = 1 (start counter)
}
void serialSendChar(char c){
    while (!(UCSR0A & _BV(UDRE0))) {;}
    UDR0 = c;
}
char hex(uint8_t h) {
    return "0123456789ABCDEF"[h & 0xf];
}
void sendSerialString(const char *s) {
    char c;
    while ((c = *s++)) serialSendChar(c);
}
void sendSerialNumber(uint8_t h) {
    serialSendChar(hex(h>>4));
    serialSendChar(hex(h>>0));
    sendSerialString("\r\n");
}
void setupSerial() {
    UCSR0A = 0;     //
    UCSR0B = 0x18;  // enable transmitter and receiver
    UCSR0C = 0x06;  // async, no parity, 1 stop, 8 data bits
    // Baud rate is calculated as rate = 16M/(16 * (baud - 1))
    // we want 9600 baud -> 16M/(16 * (9600 - 1))
    // baud rate = 104 [Note, in the data sheet it is listed as 103]
    UBRR0 = 104;    // 9600 baud with .17% error
}
void setupInterupts() {  
    EIMSK |= 3;
}
uint8_t SPI_transfer(uint8_t data) {
    SPDR = data;
    while ((SPSR & _BV(SPIF)) == 0) {;}
    return SPDR;
}
void SPI_Send_Command(int id, uint8_t reg, uint8_t data) {
    if (id == 2) PORTB &= ~_BV(0);    
    else PORTB &= ~_BV(2); 
    SPI_transfer(0x40);
    SPI_transfer(reg);
    SPI_transfer(data);
    if (id == 2) PORTB |= _BV(0);    
    else PORTB |= _BV(2);
}
uint8_t SPI_Read_Command(int id, uint8_t reg) {
    uint8_t data;
    if (id == 1)PORTB &= ~_BV(2);    
    else PORTB &= ~_BV(0);
    SPI_transfer(0x41);
    SPI_transfer(reg);
    data = SPI_transfer(0);
    if (id == 1) PORTB |= _BV(2);    
    else PORTB |= _BV(0);
    return data;
}
/*
 * f_OSC/4 => 4MHZ
 */
void setupSPI(){
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = 0;
    SPI_Send_Command(1, 0x00, 0x00); // register IODIRA outputs
    SPI_Send_Command(1, 0x01, 0xff); // register IODIRB inputs  
    SPI_Send_Command(1, 0x0d, 0xff); // GPPUB port B GPIO Pull ups
    SPI_Send_Command(1, 0x05, 0xff); // GPINTENB enable port B interrupt
    SPI_Send_Command(1, 0x0B, 0b01001000);

    SPI_Send_Command(2, 0x00, 0x00);   // register IODIRA outputs
    SPI_Send_Command(2, 0x01, 0x03); // register IODIRB inputs  
    SPI_Send_Command(2, 0x0d, 0x03); // GPPUB port B GPIO Pull ups
    SPI_Send_Command(2, 0x05, 0x03); // GPINTENB enable port B interrupt
    SPI_Send_Command(2, 0x0B, 0b01011000);
}
void setupPort(){
    DDRD  = 0b00000010; 
    PORTD = 0b11111110; 
    DDRB  = 0b00101111;
    PORTB = 0b00000101;
    DDRC  = 0b00001110; 
    PORTC = 0b00110000;
}