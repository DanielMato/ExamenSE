/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "string.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */


int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}


void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}

// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

void led_green_on()
{
  
  GPIOD->PCOR = (1 << 5);
}
void led_green_off()
{
  GPIOD->PSOR = (1 << 5);
}
void led_red_on()
{

  GPIOE->PCOR = (1 << 29);
}
void led_red_off()
{
  GPIOE->PSOR = (1 << 29);
  
}

int greenLed = 0; //0 = apagado normal, 1= encendido normal, 2 = parpadeo apagado, 3 = parpadeo encendido
int redLed = 0;



//Por algún motivo no funciona PORTD_Int_Handler
void PORTD_Int_Handler(void){ //Handler para la interrupcion por puerto C o D, aunque solo ponga D
	
  if (sw1_check()){ //Si pulsamos el switch derecho, encendemos el led verde, y apagamos el rojo
    if (greenLed == 0){
      greenLed = 1;
    } else if (greenLed == 2){
      greenLed = 3;
    }
    if (redLed == 1){
      redLed = 0;
    } else if (redLed ==3){
      redLed = 2;
    }
    while (sw1_check()){}
  } 
  else if (sw2_check()){ //Al revés
    if (redLed == 0){
      redLed = 1;
    } else if (redLed == 2){
      redLed = 3;
    }
    if (greenLed == 1){
      greenLed = 0;
    } else if (greenLed == 3){
      greenLed = 2;
    }
    while (sw2_check()){}

  }
  PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
  PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
	
	

}

int main(void)
{
  char ch;
  char comando[20] = "";
  NVIC_SetPriority(31, 0);
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  PRINTF("\r\nReinicio!\r\n");

  sws_ini();
  leds_ini();

  NVIC_EnableIRQ(31);
  while (1)
    {
      switch (greenLed){
        case 0 : 
        case 2 :
          led_green_off();
          break;
        case 1 : 
          led_green_on();
          break;
        case 3 :
          led_green_toggle();
          break;

      }
      switch (redLed){
        case 0 : 
        case 2 :
          led_red_off();
          break;
        case 1 : 
          led_red_on();
          break;
        case 3 :
          led_red_toggle();
          break;

      }
      ch = GETCHAR();
      PUTCHAR(ch);
      switch (ch){
        case '\n':
          if (strcmp(comando, "led1")==0){
              if (redLed == 0){
                redLed = 1;
              } else if (redLed == 2){
                redLed = 3;
              }
              if (greenLed == 1){
                greenLed = 0;
              } else if (greenLed == 3){
                greenLed = 2;
              }
          } else if (strcmp(comando, "led2")==0){
              if (greenLed == 0){
                greenLed = 1;
              } else if (greenLed == 2){
                greenLed = 3;
              }
              if (redLed == 1){
                redLed = 0;
              } else if (redLed == 3){
                redLed = 2;
              }
          }
          strcpy(comando, "");
          break;
      }
      strncat(comando, &ch, 1);
    }
}
