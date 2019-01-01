/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"

// UART over USB (or bluetooth) config
const eUSCI_UART_Config uartConfig = // USING 3Mhz Master Clock, baud rate 115200 (values from the calculator at http://bit.ly/432UART)
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        1,                                     // BRDIV = 19
        10,                                       // UCxBRF = 8
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/* ADC-driving timer */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        320,                                    // period = 320/32000 = 10ms => 100Hz sampling rate
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        320                                       // match Period
};





// Debounce Timer A2 config
const Timer_A_UpModeConfig debounceConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        32,                                  // Period: 32/(32Khz) = 1ms
        TIMER_A_TAIE_INTERRUPT_ENABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};




// Port remapping: Points TA1.1 and TA1.2 to pins P3.7 and P3.6, respectively.
const uint8_t portMapping[] =

{ //Port 3 remap
        PMAP_NONE,  PMAP_NONE,  PMAP_NONE,

        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,

        PMAP_TA1CCR1A,      PMAP_TA1CCR2A

};

// ~~~~~ EDIT HERE ~~~~~~~ all config structs (timera & ccrs) for PWM generation from Prelab 5
//might needs to be modified.
//Motor Controller Timer A1 config
const Timer_A_UpModeConfig leftrightConfig =
{
 TIMER_A_CLOCKSOURCE_ACLK, // Usually DCO clock, which in this case we set to 12MHz in main()
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 1000,
 TIMER_A_TAIE_INTERRUPT_DISABLE,
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
 TIMER_A_DO_CLEAR
};

////delete this one
//const Timer_A_CompareModeConfig rightConfig={
//        TIMER_A_CAPTURECOMPARE_REGISTER_1,
//        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
//        TIMER_A_OUTPUTMODE_TOGGLE_SET,
//        500
//};



//only need one ccr
const Timer_A_CompareModeConfig leftConfig={
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_TOGGLE_SET,
        1000
};

// Circular Buffer Struct & Vars
typedef struct {
uint16_t distanceADC;
    uint16_t encoderL;
    uint16_t pwmL;
} packet;
volatile packet circBuf[100];
uint8_t index_Write;
uint8_t index_Transmit;


/* Statics */
static volatile bool ignoreButton;
static volatile bool sampling;
// Are we reading a number? (set true to call handleLongCommand once reading of number is complete)
static volatile bool readingNumber;
// most recently received nonnumeric character
static volatile char recentCmd;
// the number being read in over UART (accumulates digit by digit)
static volatile int16_t readValue;
// flag in case we are reading a negative value
static volatile bool negative;
//Distance Rover should be away from the object



// new code
static volatile uint16_t checkNeg=0;

static volatile uint32_t pwmActive; //Used in Go and kill commands
static volatile uint32_t d_cm;
static volatile uint32_t Kp = 30;
static volatile uint32_t Ki = 20;
static volatile uint32_t Kd=10;

static volatile int16_t error;
static volatile int16_t error_prev;
static volatile int16_t sumError = 0;

static volatile int16_t e_derivative;

static volatile int16_t integral;
static volatile int16_t integral_prev;

static volatile int16_t P_term; //Proportional
static volatile int16_t I_term; //Integral
static volatile int16_t D_term; //Derivative
static volatile int16_t pwm; //PID

volatile unsigned int step;
volatile unsigned int currentState;

static volatile uint32_t TimerVal0; //old time
static volatile uint32_t TimerVal1 = 0xFFFFFFFF; //new time
static volatile uint32_t dt_Encoder; //delta time
static volatile uint32_t Speed; //Speed from the encoders


//static volatile uint32_t P_term; // PID for reverse, not the right way, but sometimes effective

//assume the distance is 30 for now, need to be user's input later
static volatile uint32_t distance = 30;


int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_enableSleepOnIsrExit();

    /* Setting up clocks
     * MCLK = MCLK = 3MHz = DCO = SMCLK
     * ACLK = REFO = 32Khz */
    CS_setDCOFrequency(3000000);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializing ADC (uses MCLK to sample)  */
    MAP_ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false); //SOURCE1 = timerA0's CCR1 output per table 6-51 in 432 datasheet
    ADC14_setResolution(ADC_14BIT);

    // ~~~~~ EDIT HERE ~~~~~~~ to read iN A0 (our motor driver current report pins scaled to safe values)
    ADC14_configureSingleSampleMode(ADC_MEM0, true); //USES External voltage references!! VRef+ on P5.6, VRef- on P5.7 per http://bit.ly/432Function
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);

    // ~~~~~ EDIT HERE ~~~~~~~ to read iN A3 (our distance sensor for PID)
    ADC14_configureSingleSampleMode(ADC_MEM3, true);
    ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A3, ADC_NONDIFFERENTIAL_INPUTS);//added code

    // ~~~~~ EDIT HERE ~~~~~~~ to actually use A0 as analog input pins instead of regular GPIO pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
        GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION); // per http://bit.ly/432Function

    // ~~~~~ EDIT HERE ~~~~~~~ to actually use A0 as analog input pins instead of regular GPIO pins
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
        GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION); // per http://bit.ly/432Function

    // ~~~~~ EDIT HERE ~~~~~~~ modify the interrupts on the ADC to actually trigger for A0
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableInterrupt(ADC_INT3);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();


    sampling = 0; // Only start sampling at 100Hz when activated by UART command or P1.4 button press

    /* Starting Timer_A0 in up mode and sourced from ACLK (32khz), sampling every 10ms or 100Hz */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);


    // SETUP UART
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // per http://bit.ly/432Function
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    readingNumber = 0;
    recentCmd = 0;
    negative = 0;
    readValue = 0;

    //setup UART (bluetooth)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, // no need for clock pin, unlike USB
    GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // per http://bit.ly/432Function
    UART_initModule(EUSCI_A2_BASE, &uartConfig);
    UART_enableModule(EUSCI_A2_BASE);

    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);

    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    // Setup GPIO (button & LED)
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // ~~~~~ EDIT HERE ~~~~~~~ add in the GPIO pins for telling the motor driver forward or back
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);//left
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);//left
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);

//    //added for direction fix
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);//right
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);//right
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

    //Encoder stuff---------------------------------------------------------------------------------------------------------
    /* Set up GPIO input for encoder reading */

    //Port4 does low to high transition
    GPIO_interruptEdgeSelect (GPIO_PORT_P4,GPIO_PIN1 | GPIO_PIN2,GPIO_LOW_TO_HIGH_TRANSITION);

    //Port6 does high to low transition
    GPIO_interruptEdgeSelect (GPIO_PORT_P6,GPIO_PIN6 | GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);

    //enable interrupt
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2);
    Interrupt_enableInterrupt(INT_PORT4);

    //enable interrupt
    GPIO_clearInterruptFlag(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7);
    Interrupt_enableInterrupt(INT_PORT6);
    //end of initialization for Encode
    //-------------------------------------------------------------------------------------------------------------------


    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);  // <- Not strictly necessary, safety precaution against spurious interrupt flags on boot
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
    Interrupt_enableInterrupt(INT_PORT1);

    // ~~~~~ EDIT HERE ~~~~~~~  Don't forget to initialize the timerA1 and its CCRs for PWM
    //added code
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &leftrightConfig);
    MAP_Timer_A_initCompare(TIMER_A1_BASE, &leftConfig); //left motor
 //   MAP_Timer_A_initCompare(TIMER_A1_BASE, &rightConfig); //right motor


//    Prepare debouncing using Timer A2
    ignoreButton = 0;
    Timer_A_configureUpMode(TIMER_A2_BASE, &debounceConfig);
    Timer_A_enableInterrupt(TIMER_A2_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Interrupt_enableInterrupt(INT_TA2_0);

    // Redirect TA1.1 and TA1.2 outputs to pins P3.7 and P3.8, for ease of physical wiring
    PMAP_configurePorts(portMapping, PMAP_P3MAP, 2, PMAP_DISABLE_RECONFIGURATION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN7 | GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION); //outputs TA1.1 and TA1.2 outputs on pins 3.7 and 3.6 respectively.

    //setup Circular Buffer
    index_Write = 0;
    index_Transmit = 0;
    Interrupt_disableSleepOnIsrExit(); //allows for circBuf to be emptied in main loop
    /* Going to sleep z_z */
    while (1)
    {
    if(index_Write != index_Transmit){ //stuff left to transmit
        packet p = circBuf[index_Transmit];
        if(checkNeg==0){
            printf(EUSCI_A0_BASE,"%u\t%u\t%i\n\r", p.distanceADC, p.encoderL, p.pwmL);
        }
        else if(checkNeg==1){
            printf(EUSCI_A0_BASE,"%u\t%u\t-%u\n\r", p.distanceADC, p.encoderL, p.pwmL);
        }
        //16bit * 3 + 8bit * 4 = 80bits data/packet = 100bits transmit/packet
        index_Transmit++;
        index_Transmit %= 100; //wrap around the buffer if we hit the end
    } else { //nothing left to transmit
        PCM_gotoLPM3InterruptSafe();
    }
    }

}

// Centralized Sampling-State-Toggle code for code cleanliness
void toggleSamplingState(){
    if (sampling){
        printf(EUSCI_A0_BASE,"\n\rSampling Halted.\n\r");
        sampling = 0;
        ADC14_disableConversion();
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    } else {
        printf(EUSCI_A0_BASE,"\n\rSampling...\n\r");
        ADC14_enableConversion();
        sampling = 1;
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

// called only after a complete, multicharacter command has been received (character followed by number_
void handleLongCmd(char cmd, int16_t number){
    if(cmd == 'd'){ //double
        printf(EUSCI_A0_BASE,"\n\r%i*2 = %i\n\r",number, number*2);
    }


    if(cmd == 'r'){
        Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2, number);
        if(negative==1){
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
//            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
//            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6);
        }
        else if(negative==0){
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
//            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
//            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
        }
    }
    if(cmd == 'l'){
        distance = number;
    }

}

// UART Interrupt (receives commands)
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char readdata = EUSCI_A_SPI_receiveData(EUSCI_A0_BASE);
        //Always Echo back
        //printf(EUSCI_A0_BASE, "%c.", readdata);

        //Handle & Accumulate numeric values; will call handleLongCmd(recentCmd,readValue) once command is complete
        if(readingNumber){
            if(readdata < '0' || readdata > '9'){ //new non-numeric character!!
                if(readdata=='-' && readValue==0){ //first character is minus sign means value should be negative, keep reading number
                    negative = 1;
                }
                else { //we have a nonnumeric character; have thus hit end of command.
                    handleLongCmd(recentCmd, readValue);
                    readingNumber = 0;
                    readValue = 0;
                    negative = 0;
                }
            }
            else { //numeric character
                readValue = 10*readValue + (negative ? -1 : 1) * (readdata - '0');
            }
        }

        //change recent command if input is not a number
        if(readdata != '-' && (readdata < '0' || readdata > '9')){
            recentCmd = readdata;
        }

        //command character logic:
        if(readdata == 's' || readdata == 'S'){ //Toggle sampling mode
            toggleSamplingState();
        }
        if(readdata == '?'){ //Help message
            printf(EUSCI_A0_BASE,"\n\rCommand:\tEffect:\n\r");
            printf(EUSCI_A0_BASE,"?\tPrint this help message\n\r"); //?
            printf(EUSCI_A0_BASE,"s\tToggle Sampling Behavior\n\r"); //s
            printf(EUSCI_A0_BASE,"d#\tDouble Input Number\n\r"); //?
        }
        if(readdata == 'd'){ //double cmd
            readingNumber = true; //tells us this a long command, begins accumulating number
            // rest of logic is in handleLongCmd, called when next nonnumeric character is received.
        }
        //Additional commands here as needed
        if(readdata == 'g'){
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            pwmActive = 1;
            toggleSamplingState();
        }

        //kill
        if(readdata == 'k'){
            //set duty cycle = 0
            Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2, 0);
            pwmActive = 0;
            toggleSamplingState();
            //stop and clear the timer didn't work
            //Timer_A_clearTimer(TIMER_A1_BASE);
            //Timer_A_stopTimer(TIMER_A1_BASE);
        }

        //new setpoint
        if(readdata == 'l'){
            readingNumber=true;
        }


        // ~~~~~~~~~~ EDIT HERE ~~~~~~~~~~ to add L and R
    }
}


/* This interrupt is fired whenever a conversion is completed and placed in ADC_MEM0 */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);


   if (status && ADC_INT3) //A0 reading, on P5.5, current
   {
        //uint16_t adcVal0 = ADC14_getResult(ADC_MEM0); //for current

        //uint16_t currentVal = adcVal0*0.198-1040; //in mA
        //printf(EUSCI_A0_BASE, "Current is %u __ %u\n\r",adcVal0, currentVal);  //This call is blocking!!
         uint32_t adc_val3 = ADC14_getResult(ADC_MEM3); // for distance
         //printf(EUSCI_A1_BASE, "R%u\n\r",adc_val3);  //This call is blocking!!
        if(pwmActive==1){

                 //local variable dt
                 uint32_t dt = 0.001;
                 //convert ADC outout to cm
                 d_cm = 60000*16383/3300/adc_val3;
                 //printf(EUSCI_A0_BASE, "Distance is %u\n\r",d_cm);  //This call is blocking!!

                 error = d_cm - distance;
                 sumError = sumError + error * dt;

                 P_term = Kp * error;

                 I_term = Ki * sumError;


                 //adc sampling at 1000 Hz

                 e_derivative = (error - error_prev) / dt;
                 D_term = Kd* e_derivative;


                 pwm = P_term + I_term + D_term;

                 if (pwm >= 999)
                 {
                     pwm = 999;
                 }

                 if (pwm <=50 && pwm >=-50)
                 {
                     pwm=0;
                 }

                 //if the object is too far away
                 if (error > 0){
                     //drive forward
                     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
                     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
                     checkNeg=0;
                     Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2, pwm);

                 }
                 else
                 {
                     //Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
                     //drive backward
                     pwm = -pwm;
                     checkNeg = 1;
                     MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
                     MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
                     Timer_A_setCompareValue(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2, pwm);
                 }
        }

         // sticking into circular buffer
         volatile packet* p = &(circBuf[index_Write]);
         p->distanceADC = d_cm; //Distance Sensor reading from the ADC EDIT NEEDED
         p->encoderL = (step*18.85/333); // Left front motor encoder reading EDIT NEEDED
         p->pwmL = pwm; //Signed PWM control value EDIT NEEDED
         index_Write++;
         index_Write %= 100; //Cycle on end of buffer

  }


}

void PORT1_IRQHandler(void){ //Port 1 interrupt
    uint64_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if(status & GPIO_PIN4 & ~ignoreButton){ //button 1.4
        ignoreButton = true;
        Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

        toggleSamplingState();
    }

}

void TA2_0_IRQHandler(void){ //debounce timer interrupt
    uint64_t status = Timer_A_getEnabledInterruptStatus(TIMER_A2_BASE);

    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);

    Timer_A_stopTimer(TIMER_A2_BASE);
    ignoreButton = false;
}

// UART Interrupt (receives commands)


void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char readdata = EUSCI_A_SPI_receiveData(EUSCI_A2_BASE);
        //Always Echo back
        //printf(EUSCI_A0_BASE, "%c.", readdata);

        //Handle & Accumulate numeric values; will call handleLongCmd(recentCmd,readValue) once command is complete
        if(readingNumber){
            if(readdata < '0' || readdata > '9'){ //new non-numeric character!!
                if(readdata=='-' && readValue==0){ //first character is minus sign means value should be negative, keep reading number
                    negative = 1;
                }
                else { //we have a nonnumeric character; have thus hit end of command.
                    handleLongCmd(recentCmd, readValue);
                    readingNumber = 0;
                    readValue = 0;
                    negative = 0;
                }
            }
            else { //numeric character
                readValue = 10*readValue + (negative ? -1 : 1) * (readdata - '0');
            }
        }

        //change recent command if input is not a number
        if(readdata != '-' && (readdata < '0' || readdata > '9')){
            recentCmd = readdata;
        }

        //command character logic:
        if(readdata == 's' || readdata == 'S'){ //Toggle sampling mode
            toggleSamplingState();
        }
        if(readdata == '?'){ //Help message
            printf(EUSCI_A2_BASE,"\n\rCommand:\tEffect:\n\r");
            printf(EUSCI_A2_BASE,"?\tPrint this help message\n\r"); //?
            printf(EUSCI_A2_BASE,"s\tToggle Sampling Behavior\n\r"); //s
            printf(EUSCI_A2_BASE,"d#\tDouble Input Number\n\r"); //?
        }
        if(readdata == 'd'){ //double cmd
            readingNumber = true; //tells us this a long command, begins accumulating number
            // rest of logic is in handleLongCmd, called when next nonnumeric character is received.
        }
        //Additional commands here as needed
        if(readdata == 'l' || readdata == 'r'){
            readingNumber=true;
        }


        // ~~~~~~~~~~ EDIT HERE ~~~~~~~~~~ to add L and R
    }

}

//encoder stuff -----------------------------------------------------------------------------------------------------------------

/*
 * Use P4.1 for Channel A
 *     P4.2 for channel B
 *     Port 4 ISR only low to high
 */
extern void PORT4_IRQHandler() {
    uint32_t status;
    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);

    //if it's P4.1 is interrupting, P4.1 is high
    if(status & GPIO_PIN1)
    {
        //check P4.2
        if (GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN2))//if P4.2 is high
        {
            currentState = 2;
            step--;
        }
        else
        {
            currentState = 1;
            step++;
        }
    }

    //if it's P4.2 is interrupting, P4.2 is high, check P4.1
    else if(status & GPIO_PIN2)
    {
        //check P4.1
        if (GPIO_getInputPinValue(GPIO_PORT_P4,GPIO_PIN1)) //if P4.1 is high
        {
            currentState = 2;
            step++;
        }
        else
        {
            currentState = 3;
            step--;
        }
    }
    else
    {
        printf(EUSCI_A0_BASE,"\n\r error\n\r");
    }

}

/*
 * Use P6.6 for Channel A
 *     P7.7 for channel B
 *     Port 6 ISR only do high to low transition
 */
extern void PORT6_IRQHandler() {
    uint32_t status;
    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    GPIO_clearInterruptFlag(GPIO_PORT_P6, status);

    //if it's P6.6, P6.7 is low
    if(status & GPIO_PIN6)
    {
        //check P6.7
        if (GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN7))//if P5.7 is high
        {
            currentState = 3;
            step++;
        }
        else
        {
            currentState = 0;
            step--;
        }
    }
    //if it's P6.6, P6.7 is low
    else if(status & GPIO_PIN7)
    {
        //check P5.6
        if (GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN6))//if P5.6 is high
        {
            currentState = 1;
            step--;
        }
        else
        {
            currentState = 0;
            step++;
        }

    }
    else
    {
        printf(EUSCI_A0_BASE,"\n\r error\n\r");
    }
}


