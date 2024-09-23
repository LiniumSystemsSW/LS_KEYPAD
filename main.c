//******************************************************************************
//  MSP430G2553 - QuizGame for the students of the school Sagrado Corazon de Jesus
//  The game consists of 6 arcade machine style buttons which must be pushed when
//  as soon as the game master cast a question.
//  The first button to be pushed will light until the game master reset it,
//  showing which player was the fastest answering the question.
//
//  Meanwhile, the firmware will be sending info to the PC through a serial
//  connection. This info will be received in a QT GUI application that will
//  let the teacher control the game.
//
//  Timer_A, Ultra-Low Pwr UART 9600 Echo, 32kHz ACLK
//
//  Description:

//  ACLK = TACLK = LFXT1 = 32768Hz, MCLK = SMCLK = default DCO
//  //* An external watch crystal is required on XIN XOUT for ACLK *//
//
//               MSP430G2553
//            -----------------
//        /|\|              XIN|-
//         | |                 | 32kHz
//         --|RST          XOUT|-
//           |                 |
//           |   CCI0B/TXD/P1.1|-------->
//           |                 | 9600 8N1
//           |   CCI0A/RXD/P1.2|<--------
//
//  A.Guzman
//  09/2024
// Cada vez que una tecla es cambia de nivel, se manda el estado de todas las teclas por UART. Por ejemplo, si pulsan el boton0
//   Built with Code Composer Studio Version: 8.2.0.00007
//******************************************************************************


#include <msp430.h>
#include <stdlib.h>

//------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD   0x02                     // RXD on P1.1
#define UART_RXD   0x04                     // TXD on P1.2
#define GUN0       BIT0                     // PULSADOR PISTOLA0
#define GUN0_MODE  BIT1                     // MODO PISTOLA0
#define GUN1       BIT2                     // PULSADOR PISTOLA1
#define GUN1_MODE  BIT3                     // MODO PISTOLA1
#define GUN2       BIT4                     // PULSADOR PISTOLA2
#define GUN2_MODE  BIT5                     // MODO PISTOLA2

#define ENABLE  1
#define DISABLE 0

enum GUN_MODE {
    MANUAL = 0,
    AUTO,
};

enum BUTTON_NUMBER {
    BUTTON_GUN0 = 0,
    BUTTON_GUN0_MODE,
    BUTTON_GUN1,
    BUTTON_GUN1_MODE,
    BUTTON_GUN2,
    BUTTON_GUN2_MODE,
};

typedef int BUTTON;

// COMS
#define PACKAGE_START 0x5B
#define PACKAGE_END 0x5D
char *buffer_pointer = NULL;
char uart_received[4];
unsigned char uart_semaphore = 0;
unsigned char process_package = 0;


//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void UART_init(void);
void button_led_pins_init(void);
void setup_P2_ints(unsigned char l_enable);
void TimerA_UART_tx(unsigned char byte);
void UART_print(char *string);
void UART_send(unsigned char *l_byte);
void timer_init(void);
unsigned int get_button_state(BUTTON l_gun);
void flash( int ms_cycle, int n_times );
void wait_ms( int ms_cycle);
void integer_to_string( char *str, unsigned int number );
void setup_debounce_time( unsigned char t_ms );

unsigned char button_semaphore = 0;
BUTTON button = 0;
unsigned int button_state=0;
unsigned char keypad_state = 0x00;
unsigned char debounce_time = 20;


/**
 * blink.c
 */
void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                   // Stop WDT
    if (CALBC1_1MHZ==0xFF)                      // If calibration constant erased
    {
        while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                                 // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                      // Set DCO
    DCOCTL = CALDCO_1MHZ;
    UART_init();
    button_led_pins_init();
    timer_init();

    __enable_interrupt();

    UART_print("\r\n");
    UART_print("LS-KEYPAD-CONTROLLER v1.0\r\n");

    flash( 100, 4 );

    char time[] = "0";
    for(;;)
    {
        if( button_semaphore )
        {
            setup_P2_ints( DISABLE );
            UART_send(&keypad_state);
            wait_ms(debounce_time);
            button_semaphore = 0;
            P2IFG =  0;                                 // P1.x IFG cleared
            setup_P2_ints( ENABLE );
        }

        if ( process_package )
        {
            switch (uart_received[1]) {
                case 'k':
                    UART_send(&keypad_state);
                    break;
                case 'd':
                    setup_debounce_time(uart_received[2]);
                    break;
                default:
                    break;
            }
            process_package = 0;
        }
    }

}

// Port 2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{

    if( !button_semaphore )
    {
        switch (P2IFG) {
            case GUN0:
                button = BUTTON_GUN0;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN0; // BIT0 del keypad_state a 0
                    P2IES |= GUN0; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN0;
                    P2IES &= ~GUN0; // Activo interrupcion por flanco de subida
                }
                break;
            case GUN0_MODE:
                button = BUTTON_GUN0_MODE;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN0_MODE;
                    P2IES |= GUN0_MODE; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN0_MODE;
                    P2IES &= ~GUN0_MODE; // Activo interrupcion por flanco de subida
                }
                break;
            case GUN1:
                button = BUTTON_GUN1;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN1; //
                    P2IES |= GUN1; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN1;
                    P2IES &= ~GUN1; // Activo interrupcion por flanco de subida
                }
                break;
            case GUN1_MODE:
                button = BUTTON_GUN1_MODE;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN1_MODE; //
                    P2IES |= GUN1_MODE; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN1_MODE;
                    P2IES &= ~GUN1_MODE; // Activo interrupcion por flanco de subida
                }
                break;
            case GUN2:
                button = BUTTON_GUN2;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN2; //
                    P2IES |= GUN2; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN2; //
                    P2IES &= ~GUN2; // Activo interrupcion por flanco de subida
                }
                break;
            case GUN2_MODE:
                button = BUTTON_GUN2_MODE;
                if( P1OUT & GUN0 ) // si led encendido, es que venimos de boton presionado
                {
                    P1OUT &= ~GUN0; // Apago led
                    keypad_state &= ~GUN2_MODE; //
                    P2IES |= GUN2_MODE; // activo interrupcion por flanco de bajada
                }
                else
                {
                    P1OUT |= GUN0; // enciendo led
                    keypad_state |= GUN2_MODE; //
                    P2IES &= ~GUN2_MODE; // Activo interrupcion por flanco de subida
                }
                break;
        }
    }

    button_semaphore = 1;
    P2IFG = 0;
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
  if ( UCA0RXBUF == PACKAGE_START )
  {
      uart_semaphore = 1;
      buffer_pointer = uart_received;
  }
  else if ( UCA0RXBUF == PACKAGE_END )
  {
      uart_semaphore = 0;
      *buffer_pointer = UCA0RXBUF;
      buffer_pointer = NULL;
      process_package = 1;
  }

  if ( uart_semaphore )
  {
      *buffer_pointer++ = UCA0RXBUF;
  }
}

//------------------------------------------------------------------------------
// Function configures full-duplex UART operation
//------------------------------------------------------------------------------
void UART_init(void)
{
    P1SEL = UART_TXD + UART_RXD;              // UART function for TXD/RXD pins
    P1SEL2 = UART_TXD + UART_RXD;             // UART function for TXD/RXD pins
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 8;                              // 1MHz 115200
    UCA0BR1 = 0;                              // 1MHz 115200
    UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}


void timer_init(void)
{
    // Configure TA1 to count
    TA1CTL = TASSEL_1 + ID_3 + MC_2 + TACLR;        // ACLK, upmode, clear TAR
}

//------------------------------------------------------------------------------
// Function sets up pinout
//------------------------------------------------------------------------------
void button_led_pins_init(void)
{

    /*P1.4, P1.5 y P1.6 son los leds de los botones*/
    /*P2.0..2.5 son los botones e interruptores*/

    P1DIR = BIT0 + BIT1;                        // Set all P1.x to input direction except TX and LED
    P1IE =  BIT2;                               // All P1.x interrupts disbled except RX
    P1IES = BIT2;                               // P1.3 and P1.4 Hi/lo edge
    P1REN = ~( BIT0 + BIT1 + BIT2 );            // Enable Pull Up on every pin except P1.0, P1.1 and P1.2
    P1IFG =  0;                                 // P1.x IFG cleared

    P2DIR = 0x00;                               // Set all P2.x to INPUT direction
    P2IE  = GUN0 + GUN0_MODE + GUN1 + GUN1_MODE + GUN2 + GUN2_MODE; // Ints enabled on all buttons
    P2IES = GUN0 + GUN0_MODE + GUN1 + GUN1_MODE + GUN2 + GUN2_MODE; // Ints on high-to-low transition
    P2REN = GUN0 + GUN0_MODE + GUN1 + GUN1_MODE + GUN2 + GUN2_MODE; // ENABLE PULL resistors
    P2OUT = 0x00;
    P2OUT = GUN0 + GUN0_MODE + GUN1 + GUN1_MODE + GUN2 + GUN2_MODE; // As the pull resistors are enabled, writing 1 to the reg selects the PULL-UP
}

//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void UART_print( char *string )
{
    while (*string)
    {
        while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
        UCA0TXBUF = *string++;                    // TX -> next character character
   }
}

void UART_send(unsigned char *l_byte)
{
    while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
    UCA0TXBUF = *l_byte;                      // TX -> next character character

}


void flash( int ms_cycle, int n_times )
{
    int l_var0 = n_times;
    for( l_var0 = n_times; l_var0 > 0; l_var0-- )
    {
        P1OUT ^= BIT0;
        wait_ms(ms_cycle);
        P1OUT ^= BIT0;
        wait_ms(ms_cycle);
    }

    P1OUT &= ~( BIT0 );
}

void wait_ms(int ms_cycle)
{
    int l_var0 = ms_cycle;
    for(l_var0 = ms_cycle; l_var0 > 0; l_var0-- )
    {
        __delay_cycles(1000);
    }
}

unsigned int get_button_state(BUTTON l_gun)
{
    keypad_state = ~(P2IN&0x1F);
    return ((P2IN&(BIT0<<l_gun))&&BIT0);
}

void setup_P2_ints(unsigned char l_enable)
{
    if( l_enable )
    {
        P2IE  = GUN0 + GUN0_MODE + GUN1 + GUN1_MODE + GUN2 + GUN2_MODE; // Ints enabled on all buttons
    }
    else
    {
        P2IE = 0x00;
    }
}

void setup_debounce_time( unsigned char t_ms )
{
    debounce_time = t_ms;
}
