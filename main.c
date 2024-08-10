/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "SSD1306OLED.h"
#include <math.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x90U >> 1)
#define SSD1306_ADDR         0x3C
#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U
#define DS1631_ADDR         0x4F
/* Mode for LM75B. */
#define NORMAL_MODE 0U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
 const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
 typedef  enum{
  KEY_PRESSED,
  KEY_RELEASED
 } keyState_t;

typedef struct {
  keyState_t KeyState;
  bool KeyStateChanged;
  bool _previousValue;
} keys_t;

uint32_t cnt;
keys_t keys[4];
int16_t tempCnt;

void DebounceSwitch(){
  static uint8_t counter[4]={0};
  uint8_t i;

  for(i=0;i<4;i++){
    if(bsp_board_button_state_get(i) == keys[i]._previousValue){
      counter[i]=4;//20ms
    }
    else{
      counter[i]--;
      if(counter[i]==0){
        keys[i].KeyStateChanged=true;
        (bsp_board_button_state_get(i) == true)?(keys[i].KeyState=KEY_PRESSED):(keys[i].KeyState=KEY_RELEASED);
        keys[i]._previousValue=bsp_board_button_state_get(i);
        counter[i]=4;
      }
    }
  }
}

void SysTick_Handler(){
cnt ++;
if(cnt%5){
DebounceSwitch();
  }
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
 int8_t menuSel=0;
 int8_t opModeSel=0;
 uint16_t period=1;
 char operatingMode[3][10]={"Prag H","Prag L","Prag H2L"};
 bool temp=true;
 char mod[10]="Prag H";
 float pragH=31.0f;
 float pragL=20.0f;

 void displayOpMode(int8_t opModeSel){
        SSD1306_ClearDisplay();
        SSD1306_DrawText(3,16,"Mod operare: ",1);
        strcpy(mod,operatingMode[opModeSel]);
        SSD1306_DrawText(75,16,operatingMode[opModeSel],1);

        SSD1306_Display();
 }
 float temperaturaCurenta=0.0f;
 void mainMenu(){
        SSD1306_ClearDisplay();
        SSD1306_DrawText(3,1,"T. Curenta: ",1);
        SSD1306_DrawRoundRect(0,0,65,10,1);
        //char currTemp[20];
        //sprintf(currTemp,"%.2f",temperaturaCurenta);
        //SSD1306_DrawText(20,10,currTemp,1);
        SSD1306_DrawRoundRect(0,18,52,10,1);
        SSD1306_DrawText(70,0,"T H: ",1);
        SSD1306_DrawRoundRect(69,0,23,8,1);
        //char highTemp[3];
        //sprintf(highTemp,"%.3f",pragH);
        //SSD1306_DrawText(70,8,highTemp,1);
        SSD1306_DrawText(70,16,"T L:",1);
        //char lowTemp[3];
        //sprintf(lowTemp,"%.3f",pragL);
        SSD1306_DrawRoundRect(69,15,23,9,1);

        //SSD1306_DrawText(70,24,lowTemp,1);

        SSD1306_Display();
 }
 bool C;
 void displayUnitMenu(bool da){
        SSD1306_ClearDisplay();
        SSD1306_DrawText(3,16,"Unit. Masura: ",1);
        if(da){
        SSD1306_DrawText(83,16,"C",1);
        }else SSD1306_DrawText(83,16,"F",1);
        SSD1306_Display();
 }
 char output[15];
 void displayPeriod(uint8_t perioada){
        SSD1306_ClearDisplay();
        sprintf(output,"Perioada: %ds",perioada);
        SSD1306_DrawText(3,16,output,1);
        SSD1306_Display();
 }
 char precision[3][8]={"0.5","0.25","0.0625"};
 int8_t precisionSel=0;
 void displayPrecision(uint8_t sel){
        SSD1306_ClearDisplay();
        SSD1306_DrawText(3,16,"Precizie: ",1);
        SSD1306_DrawText(60,17,precision[precisionSel],1);
        SSD1306_Display();
 }
 void displayMenu(uint8_t menu){

    switch(menu){
        case 0:
        mainMenu();
        break;
        case 1:
        displayOpMode(opModeSel);
        break;
        case 2:
        displayUnitMenu(temp);
        break;
        case 3:
        displayPeriod(period);
        break;
        case 4:
        displayPrecision(precisionSel);
        break;
    }
    
}
void ds1631_init(void)
{
    //00(0.5),0A(0.25),0C(0.0625)
    uint8_t command[] = {0xAC, 0x00};
    
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR, command, 2, false);
    uint8_t command2[]={0x51};
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR,command2 , 1, false);

    APP_ERROR_CHECK(err_code);
}
void setPrecision(void){
  //00(0.5),0A(0.25),0C(0.0625)
    uint8_t da=0x00;
    if(strcmp(precision[precisionSel],"0.0625")==0){
        da=0x0C;
      }else if(strcmp(precision[precisionSel],"0.5")==0){
          da=0x00;
        }
        else da=0x0A;
        
    uint8_t command[] = {0xAC, da};
    
    ret_code_t err_code;
    uint8_t command2[] = {0x22};
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR, command2, 1, false);
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR, command, 2, false);
    uint8_t command3[]={0x51};
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR,command3 , 1, false);
}
#define DS1631_TEMPERATURE_REGISTER    0xAA
float ds1631_read_temperature(void)
{
    uint8_t temperature[2];
    float temp;

    ret_code_t err_code;
    uint8_t command[] = {0xAA};
    err_code = nrf_drv_twi_tx(&m_twi, DS1631_ADDR, command, 1, true);
    err_code = nrf_drv_twi_rx(&m_twi, DS1631_ADDR, temperature, sizeof(temperature));
    APP_ERROR_CHECK(err_code);

    int16_t raw_temp = (temperature[0] << 8) | temperature[1];
    temp = raw_temp / 256.0;

    return temp;
}
char rState[4];

int main(void)
{   bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);
    SysTick_Config(SystemCoreClock/1000);
    bool inv=false;
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    ds1631_init();
    //LM75B_set_mode();
    SSD1306_Begin(SSD1306_SWITCHCAPVCC,SSD1306_I2C_RealADDRESS);
    SSD1306_ClearDisplay();
    temperaturaCurenta=ds1631_read_temperature();
    displayMenu(menuSel);
    nrf_gpio_cfg_output(47);

    while (true)
    {     if(menuSel==0){
            char highTemp[3];
            sprintf(highTemp,"%.3f",pragH);
            SSD1306_DrawText(70,8,highTemp,1);
            char lowTemp[3];
            sprintf(lowTemp,"%.3f",pragL);
            SSD1306_DrawText(70,24,lowTemp,1);
            SSD1306_Display();
          }
          if(nrf_gpio_pin_out_read(47)){
              strcpy(rState,"ON ");
              if(menuSel==0){
                SSD1306_DrawText(3,20,rState,1);
                SSD1306_Display();
                }
            }
            else{
              strcpy(rState,"OFF");
                if(menuSel==0){
                  SSD1306_DrawText(3,20,rState,1);
                  SSD1306_Display();
                  }
            }
          if(cnt>1000*period){
            temperaturaCurenta=ds1631_read_temperature();
            if(strcmp(mod,"Prag H")==0&&temperaturaCurenta>=31.0f){
              nrf_gpio_pin_set(47);
              }
              else 
              {
              nrf_gpio_pin_clear(47);
              }
            if(strcmp(mod,"Prag L")==0&&temperaturaCurenta<=20.0f){
              nrf_gpio_pin_clear(47);
              }
            if(strcmp(mod,"Prag H2L")==0&&(temperaturaCurenta<=20.0f || temperaturaCurenta>=31.0f)){
              nrf_gpio_pin_set(47);
              }
            if(menuSel==0){
              char currTemp[20];
              if(temp){
              pragH=31.0;
              pragL=20.0;
              sprintf(currTemp,"%.3f",temperaturaCurenta);
              strcat(currTemp,"C");
              }else {
              temperaturaCurenta=temperaturaCurenta*1.8 + 32;
              pragH=31.0*1.8+32;
              pragL=20.0*1.8+32;

              sprintf(currTemp,"%.3f",temperaturaCurenta);
              strcat(currTemp,"F");
              }
              
              SSD1306_DrawText(10,10,currTemp,1);
              SSD1306_Display();
              }
            cnt=0;
          }
          
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
        if(keys[i].KeyStateChanged==true){
          if(keys[i].KeyState==KEY_PRESSED)
            switch(i){
                      case 0:
                      menuSel++;
                      if(menuSel>4)
                        menuSel=0;
                      displayMenu(menuSel);
                      break;
                      case 2:
                        switch(menuSel){
                            case 1:
                            opModeSel--;
                            if(opModeSel<0){
                            opModeSel=2;
                            }
                            displayOpMode(opModeSel);
                            break;
                            case 2:
                            temp^=true;
                            displayUnitMenu(temp);
                            break;
                            case 3:
                            period--;
                            if(period<1){
                              period=20;
                            }
                            displayPeriod(period);
                            break;
                            case 4:
                            precisionSel--;
                            if(precisionSel<0){
                              precisionSel=2;
                            }
                            setPrecision();
                            displayPrecision(precisionSel);
                            break;
                        }
                      break;
                      case 3:
                        switch(menuSel){
                        case 3:
                            period++;
                            if(period>20){
                              period=1;
                            }
                            displayPeriod(period);
                            break;
                        }
                    }
            keys[i].KeyStateChanged=false;         
           }    
        }
    }
}

/** @} */
