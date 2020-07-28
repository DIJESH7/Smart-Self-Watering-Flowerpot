//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include "adc0.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
 char buffer[MAX_CHARS+1];
 uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;




// Bitband aliases

#define COMP (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define DEINT (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define MOTOR (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define SPEAKER (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))

#define COMP_MASK 128
#define DEINT_MASK 32

// PortE masks
#define AIN1_MASK 4
#define AIN2_MASK 2
#define AIN0_MASK 8
//PORT A masks
#define MOTOR_MASK 4
#define SPEAKER_MASK 8

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R2 |SYSCTL_RCGCGPIO_R5|SYSCTL_RCGCGPIO_R0;
    //CONFIGURE ADC0
    SYSCTL_RCGCADC_R = SYSCTL_RCGCADC_R0 ;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1|SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCACMP_R = SYSCTL_RCGCACMP_R0;
    SYSCTL_RCGCHIB_R = SYSCTL_RCGCHIB_R0 ;
    _delay_cycles(3);

    //CONFIGURE COMPARATOR
       GPIO_PORTC_DIR_R  &= ~COMP_MASK;
       // GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC7_M;
       GPIO_PORTC_AMSEL_R = 0x00000080;
       COMP_ACREFCTL_R = 0x0000020F;
       COMP_ACCTL0_R |= 0x0000040C;

       //CONFIGURE DEINT
       GPIO_PORTE_DIR_R |= DEINT_MASK;
       GPIO_PORTE_DEN_R |= DEINT_MASK;

       GPIO_PORTA_DIR_R|=MOTOR_MASK;
       GPIO_PORTA_DEN_R|=MOTOR_MASK;

       GPIO_PORTA_DIR_R|=SPEAKER_MASK;
       GPIO_PORTA_DEN_R|=SPEAKER_MASK;


       //CONFIGURE TIMER 1 FOR COUNT UP 25 NS
       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
       TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
       TIMER1_TAMR_R = TIMER_TAMR_TACDIR;
       TIMER1_CTL_R |=TIMER_CTL_TAEN;

       TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
       TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
       TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
       TIMER2_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
       TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
       NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 37 (TIMER1A)
      //TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time


}
void initEEPROM()
{
    SYSCTL_RCGCEEPROM_R|=SYSCTL_RCGCEEPROM_R0;
    _delay_cycles(6);
    while ((EEPROM_EEDONE_R&=EEPROM_EEDONE_WORKING));
        if ((EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY)||(EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY))
        {
            return NULL;
        }
        SYSCTL_SREEPROM_R|=SYSCTL_SREEPROM_R0;
        SYSCTL_SREEPROM_R=0;
        _delay_cycles(6);
        while (EEPROM_EEDONE_R &=EEPROM_EEDONE_WORKING);
        if ((EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY)||(EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY))
                {
                    return NULL;
                }

}

void Store_Hist( uint16_t data, uint16_t block, uint16_t offset)
{
    while( EEPROM_EEDONE_R &= EEPROM_EEDONE_WORKING)
        EEPROM_EEBLOCK_R=block;
        EEPROM_EEOFFSET_R=offset;
        EEPROM_EERDWR_R=data;


}
uint16_t Read_Hist(uint16_t block, uint16_t offset)
{
while(EEPROM_EEDONE_R &= EEPROM_EEDONE_WORKING)
    EEPROM_EEBLOCK_R=block;
    EEPROM_EEOFFSET_R=offset;
    return EEPROM_EERDWR_R;

}

void Erase_Hist(uint16_t block, uint16_t offset)
{
    while( EEPROM_EEDONE_R &= EEPROM_EEDONE_WORKING)
           EEPROM_EEBLOCK_R=block;
           EEPROM_EEOFFSET_R=offset;
           EEPROM_EERDWR_R=0;

}
void timer1Isr()
{
    SPEAKER ^= 1;
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void playBatteryLowAlert()
{

    TIMER2_CTL_R |= TIMER_CTL_TAEN;
    TIMER2_TAILR_R=19111.3235;
    waitMicrosecond(2000000);
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
       //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER2_CTL_R |= TIMER_CTL_TAEN;
       TIMER2_TAILR_R=102040.816;
           waitMicrosecond(2000000);
              TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

}
void playWaterLowAlert()
{

    TIMER2_CTL_R |= TIMER_CTL_TAEN;
      TIMER2_TAILR_R=19111.3235;
      waitMicrosecond(2000000);
      TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
         //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
      TIMER2_CTL_R |= TIMER_CTL_TAEN;
         TIMER2_TAILR_R=102040.816;
             waitMicrosecond(2000000);
                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

}


void enablePump()
{

    MOTOR=1;
    //waitMicrosecond(4000000);
    //MOTOR=0;
}
void disablePump()
{
    MOTOR=0;
}

float getLightPercentage()
{
    uint16_t raw;
    float instantLight = 0;
    GPIO_PORTE_AFSEL_R |= AIN2_MASK;
    GPIO_PORTE_DEN_R &= ~AIN2_MASK;
    GPIO_PORTE_AMSEL_R |= AIN2_MASK;
    setAdc0Ss3Mux(2);
    setAdc0Ss3Log2AverageCount(2);
     raw = readAdc0Ss3();
    instantLight = (((raw+0.5) / 4096.0 )*3.3) ;
   // char lightvoltage[100];
   // sprintf(lightvoltage,"lightvoltage : %4.1f",instantLight);
   // putsUart0(lightvoltage);
    float lightpercentage=0;
    lightpercentage=(instantLight/3.3)*100;;
    return lightpercentage;
}
float getMoisturePercentage()
{
    uint16_t raw1;
    float instantMoisture=0;
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;
    setAdc0Ss3Mux(1);
    setAdc0Ss3Log2AverageCount(2);
    // Read sensor
    raw1 = readAdc0Ss3();
    instantMoisture = (((raw1+0.5) / 4096.0 )*3.3) ;
    //char moisturevoltage[100];
    //sprintf(moisturevoltage,"moisturevoltage : %4.1f",instantMoisture);
    //putsUart0(moisturevoltage);
    float moisturepercentage=0;
    moisturepercentage= ((instantMoisture/3.3)*100);
    return moisturepercentage;
}

float getBatteryVoltage()
{
    uint16_t raw2;
    float instantVoltage=0;
    GPIO_PORTE_AFSEL_R |= AIN0_MASK;
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;
    setAdc0Ss3Mux(0);
    setAdc0Ss3Log2AverageCount(2);
    // Read sensor
    raw2 = readAdc0Ss3();
    instantVoltage = (((raw2+0.5) / 4096.0 )*3.3) ;
    return instantVoltage;
}


//-----------------------------------------------------------------------------
// isCommand
//-----------------------------------------------------------------------------

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int i=data->fieldPosition[0];
    char check[MAX_CHARS];
    int loop=1;
    int store=0;
    while(loop)
    {
      char value= data->buffer[i];
      check[store]=value;
      if (data->buffer[i]=='\0')
      {
          loop=0;
      }
      i++;
      store++;
    }
    int comp=strcmp(check,strCommand);
    if (comp==0&&data->fieldCount>=minArguments+1)
    {
        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// getFieldString
//-----------------------------------------------------------------------------
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber>data->fieldCount)
      {
          return NULL;
      }
    int i=data->fieldPosition[fieldNumber];

    char check[MAX_CHARS];
    int loop=1;
    int store=0;
    while(loop)
    {
      char value= data->buffer[i];
      check[store]=value;
      if (data->buffer[i]=='\0')
      {
          return check;
      }
      i++;
      store++;
    }
}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber>data->fieldCount)
          {
              return NULL;
          }
    int i=data->fieldPosition[fieldNumber];
    char check[MAX_CHARS];
    int loop=1;
    int store=0;
    int answer=0;
    while(loop)
    {
        char value= data->buffer[i];
        check[store]=value;
        if (data->buffer[i]=='\0')
        {
            answer=atoi(check);
            return answer;
        }
        i++;
        store++;
    }
}
//-----------------------------------------------------------------------------
// parseFields
//-----------------------------------------------------------------------------
parseFields(USER_DATA* data)
{
    int check=0;
    int count=0;
    int fieldcount=0;
    while (1)
    {
        //for set 1 2
        if (count==0&&!((data->buffer[count]>=32&&data->buffer[count]<=47)||(data->buffer[count]>=58&&data->buffer[count]<=65)||(data->buffer[count]>=123&&data->buffer[count]<=126)))
        {
            data->fieldPosition[fieldcount]=count;
            if (data->buffer[count]>=48 && data->buffer[count]<=57)
            {
                data->fieldType[fieldcount]= 'n';
                fieldcount++;
            }
            if (data->buffer[count]>=65&& data->buffer[count]<=90)
            {
                data->fieldType[fieldcount]= 'a';
                fieldcount++;
            }
            if (data->buffer[count]>=97 && data->buffer[count]<=122)
            {
                data->fieldType[fieldcount]= 'a';
                fieldcount++;
            }
            count++;
        }
        //for one delimeter
        if ((data->buffer[count]>=32&&data->buffer[count]<=47)||(data->buffer[count]>=58&&data->buffer[count]<=65)||(data->buffer[count]>=123&&data->buffer[count]<=126))
        {
            data->buffer[count]='\0';
            count++;
            while((data->buffer[count]>=32&&data->buffer[count]<=47)||(data->buffer[count]>=58&&data->buffer[count]<=65)||(data->buffer[count]>=123&&data->buffer[count]<=126))
            {
                count++;
                if (!((data->buffer[count]>=32&&data->buffer[count]<=47)||(data->buffer[count]>=58&&data->buffer[count]<=65)||(data->buffer[count]>=123&&data->buffer[count]<=126)))
                {
                  break;
                }
            }
            data->fieldPosition[fieldcount]=count;
            if (data->buffer[count]>=48 && data->buffer[count]<=57)
            {
                data->fieldType[fieldcount]= 'n';
                fieldcount++;
            }
            if (data->buffer[count]>=65&& data->buffer[count]<=90)
            {
                data->fieldType[fieldcount]= 'a';
                fieldcount++;
            }
            if (data->buffer[count]>=97 && data->buffer[count]<=122)
            {
                data->fieldType[fieldcount]= 'a';
                fieldcount++;
            }
        }
        count++;
        if (fieldcount==MAX_FIELDS)
        {
            data->fieldType[fieldcount]=0;
            data->fieldCount=fieldcount;
            return data;
        }
        if (data->buffer[count]=='\0')
        {
            data->fieldType[fieldcount]=0;
            data->fieldCount=fieldcount;
            return data;
        }
    }
}

//-----------------------------------------------------------------------------
// getsUart0
//-----------------------------------------------------------------------------
getsUart0(USER_DATA* data)
{
    int count=0;
    while(1)
    {
    char c = getcUart0();
    if (c==8||c==127)
    {
        if (count>0)
        {
            count--;
        }
        continue;
    }
    if (c==13)
    {
        data->buffer[count]=0;
        return data;
    }
    if (c>=32)
    {
        data->buffer[count]=c;
        count++;
        if (count==MAX_CHARS)
        {
            data->buffer[count]==0;
            return data;
        }
    }
    }
}

uint32_t getVolume()
{
    DEINT=1;
    waitMicrosecond(1000);
    DEINT=0;
    TIMER1_TAV_R=0;
    while(COMP_ACSTAT0_R && COMP_ACSTAT0_OVAL );
    return TIMER1_TAV_R;
}
uint32_t getCurrentSeconds()
{
    uint32_t time= HIB_RTCC_R;
    return time;

}
bool isWateringAllowed(uint32_t start,uint32_t end)
{
    uint32_t current_time=getCurrentSeconds();
if (end>current_time&&current_time>start)
{
return true;
}
else {
    return false;
}

}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;
    initHw();
    initUart0();
    initAdc0Ss3();
    initEEPROM();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    HIB_IM_R = HIB_IM_WC;
    HIB_CTL_R |= HIB_CTL_CLK32EN;
    while(!(HIB_MIS_R |= HIB_MIS_WC));
    while(!(HIB_CTL_R & 0x80000000));
    //HIB_RTCLD_R = 43200;
    HIB_CTL_R |= HIB_CTL_RTCEN;
    //uint32_t current_time=getCurrentSeconds();
    uint16_t offset=0;
    uint32_t start_time= 32400; //9 o'clock in the morning
    uint32_t end_time= 61200;   //5o'clock in the evening
    uint32_t water_level=30;
           //while(1)
                      //{


                     // waitMicrosecond(500000);
                     // }
    while(1)
    {
        //playBatteryLowAlert();
        if (kbhitUart0())
        {
    // Get the string from the user
    getsUart0(&data);
    // Echo back to the user of the TTY interface for testing
    putsUart0(data.buffer);
    // Parse fields
    parseFields(&data);
    // Echo back the parsed field information (type and fields)
    uint8_t i;
    putcUart0('\n');
    putcUart0('\r');
   /* for (i = 0; i < data.fieldCount; i++)
    {
        putcUart0(data.fieldType[i]);
        putcUart0('\n');
        putcUart0('\r');
        putsUart0(&data.buffer[data.fieldPosition[i]]);
        putcUart0('\n');
        putcUart0('\r');

    }*/
    bool valid=false;
    if (isCommand(&data, "status", 0))
    {
      char volume[100];
      uint32_t timer;
      float vol;
      timer=getVolume();
      //timer=(timer*25)/100000;
      vol= (0.5330*(timer-322));
      sprintf(volume,"Volume: %f mililiters\n\r",vol);
      putsUart0(volume);

      // For light sensor
      float lightpercentage=getLightPercentage();
      char lightpercentagec[50];
      sprintf(lightpercentagec,"lightpercentage : %4.1f\n\r",lightpercentage);
      putsUart0(lightpercentagec);

      //For moisture sensor
      float moisturepercentage=0;
      moisturepercentage=getMoisturePercentage();
      char moisturepercentagec[50];
      sprintf(moisturepercentagec,"moisturepercentage : %4.1f\n\r",moisturepercentage);
      putsUart0(moisturepercentagec);

      //For voltage sensor
      float BatteryVoltage= 0;
      BatteryVoltage=getBatteryVoltage();
      char batteryvoltage[100];
      BatteryVoltage= (BatteryVoltage/47000)*(47000+100000);
      sprintf(batteryvoltage,"batteryvoltage : %4.1f\n\r",BatteryVoltage);
      putsUart0(batteryvoltage);
      valid=true;
      Store_Hist(moisturepercentage,0,offset);
      offset++;
       Store_Hist(lightpercentage,0,offset);
       offset++;

       Store_Hist(vol,0,offset);
       offset++;
       if (offset==15)
                      {
                          offset=0;
                      }

      if (lightpercentage>10&&vol<50)
      {
          playWaterLowAlert();
      }
      if (lightpercentage>10&&BatteryVoltage<1.0)
      {
          playBatteryLowAlert();
      }
    }
    if (isCommand(&data, "Pump", 1))
    {
        char *str  = getFieldString(&data, 1);
        if (strcmp(str,"ON")==0)
        {
            enablePump();
            putsUart0("On now");
        }
        if (strcmp(str,"OFF")==0)
        {
            disablePump();
            putsUart0("Off now");
        }
        valid=true;
    }
    if (isCommand(&data, "History", 0))
    {
        int i =0;

            putsUart0("Moisture Light and Volume Respectively\n\r");


            for (i=0;i<15;i++)
                       {


                   uint16_t moisture_history = Read_Hist(0,i);
                   char historym[100];
                   sprintf(historym,"%4.2lu ",moisture_history);
                   putsUart0(historym);

                       }
            //putsUart0("No History\n\r");




            putcUart0('\n');
                   putcUart0('\r');

            valid=true;

        }
    if (isCommand(&data, "Time", 2))
        {
        uint32_t hr=getFieldInteger(&data,1);

        uint32_t min=getFieldInteger(&data,2);
        HIB_RTCLD_R=hr*3600+min*60;

                            valid=true;
        }

    if (isCommand(&data, "Erase", 0))
       {
           int i =0;


               for (i=0;i<15;i++)
               {
                   Erase_Hist(0,i);
               }
               offset=0;
               valid=true;
       }

    if (isCommand(&data, "LEVEL", 1))
    {
    uint32_t level=getFieldInteger(&data,1);
    water_level=level;
    putsUart0("Level changed\n\r");

    valid=true;
    }
    if(isCommand(&data, "water",4))
            {
        uint32_t hr1=getFieldInteger(&data,1);
        uint32_t min1=getFieldInteger(&data,2);
        uint32_t hr2=getFieldInteger(&data,3);
        uint32_t min2=getFieldInteger(&data,4);

            start_time=hr1*3600+min1*60;
            end_time=hr2*3600+min2*60;

            putsUart0("time1 changed\n\r");
            valid=true;
            }






    if (!valid)
    putsUart0("Invalid command\n");
   }
        else{
                uint16_t moisturepercentage=0;
                moisturepercentage=getMoisturePercentage();

                uint16_t lightpercentage=getLightPercentage();

                uint32_t timer;
                uint16_t vol=0;
                timer=getVolume();
                vol= (0.5330*(timer-322));

                float BatteryVoltage= 0;
                BatteryVoltage=getBatteryVoltage();

                if ((moisturepercentage<water_level )&& (isWateringAllowed(start_time,end_time)))
                {


                       // int i=0;
                        //for(i=0;i<5;i++)
                        //{
                        enablePump();
                        waitMicrosecond(5000000);
                        disablePump();
                        waitMicrosecond(30000000);
                        //waitMicrosecond(5000000);
                        //}

                }
                //playWaterLowAlert();
                if (vol<100)
                {
                    playWaterLowAlert();
                }
                if (BatteryVoltage<1.5)
                {
                    playBatteryLowAlert();
                }



            }
    }


    while (true);
    return 0;
}
