/**
  ******************************************************************************
  * @file    usbd_digital_io.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                HID Class  Description
  *          =================================================================== 
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Digital IO
  *             - Collection : Application 
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_analog_io.h"
#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

/* Global variables */
HID_ANALOG_IO_TypeDef analog_io;
HID_ANALOG_IO_TypeDef analog_io_new_state;
HID_ANALOG_IO_Trigger analog_io_trigger;
HID_ANALOG_IO_Trigger analog_io_trigger_io_change_flag;
HID_ANALOG_IO_Trigger analog_io_do_trigger = DONTCARE;
ANALOG_IO_Change_Flag analog_io_change_enable;
ANALOG_IO_Change_Flag analog_io_change_flag;
ANALOG_IO_Report_Flag analog_io_report_flag;
HID_ANALOG_IO_TRIGGER_Event analog_io_trig_events[ANALOG_IO_MAX_TRIG_NUM];

/* Functions */
extern uint16_t sinus[200];
extern uint16_t constant[200];
extern DAC_HandleTypeDef hdac;
/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @param  phost: Host handle
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_Init(HID_ANALOG_IO_TypeDef* analog_io_instance)
{
  uint8_t in_idx = 0, out_idx = 0;

  // Step over all INPUT
  for(in_idx = 0; in_idx < ANALOG_MAX_IN_NUM; in_idx++)
  {
	  // Add default INPUT values
	  analog_io_instance->in[in_idx] = 0;
  }
  // Step over all OUTPUT
  for(out_idx = 0; out_idx < ANALOG_MAX_OUT_NUM; out_idx++)
  {
	  // Add default OUTPUT values
	  analog_io_instance->in[out_idx] = 0;
	  analog_io_instance->type[out_idx] = OUT_CONST;
	  analog_io_instance->value[out_idx] = 0;
  }
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @param  phost: Host handle
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_Reset_SwitchTrig(void)
{
  // Unset trigger, change and report flag
  analog_io_trigger = DONTCARE;
  analog_io_trigger_io_change_flag = UNCHANGED;
  analog_io_trigger_io_change_flag = NO_REPORT;
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_CreateReport(uint8_t* report)
{
	uint8_t in_idx = 0, out_idx = 0, report_idx = 0;
	uint16_t value = 0;

  // Step over all INPUT
  for(in_idx = 0; in_idx < ANALOG_MAX_IN_NUM; in_idx++)
  {
	  // Add INPUT values
	  value = analog_io.in[in_idx];
	  if (value == 0) value = 1;
	  report[report_idx++]= (uint8_t)(value & 0b00000001111111);
	  report[report_idx++]= (uint8_t)((value & 0b11111110000000) >> 8);
  }

  // ADD border
  report[report_idx++]= 0;

  // Step over all OUTPUT
  for(out_idx = 0; out_idx < ANALOG_MAX_OUT_NUM; out_idx++)
  {
	  // Add OUTPUT values
	  value = analog_io.out[out_idx];
	  if (value == 0) value = 1;
	  report[report_idx++]= (uint8_t)(value & 0b00000001111111);
	  report[report_idx++]= (uint8_t)((value & 0b11111110000000) >> 8);
  }
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_Set_Changes(uint8_t* output_buff)
{
	if(output_buff[0] == OUT_SINUS)
	{
		analog_io.type[0] = OUT_SINUS;
	}
	else if (output_buff[0] == OUT_CONST)
	{
		analog_io.value[0] = output_buff[1];
		analog_io.type[0] = OUT_CONST;
	}
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_ANALOG_IO_Trigger (uint8_t* output_buff)
{
	if (output_buff[0] == 0xfe)
	{
		analog_io_trigger = TRIGGERED;
	}
	else
	{
		analog_io_trigger = DONTCARE;
	}
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_SwitchConfig(void)
{
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	if(analog_io.type == OUT_SINUS)
	{
		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sinus, 200, DAC_ALIGN_12B_R);
	}
	else if (analog_io.type == OUT_CONST)
	{
		uint8_t i = 0;
		for(i = 0; i < 200; i++)
		{
			constant[i] = analog_io.value;
		}
		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)constant, 200, DAC_ALIGN_12B_R);
	}
}


/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_Reset_Trigger_Event(HID_ANALOG_IO_TRIGGER_Event* trig_event)
{
	// Initialize trigger event values
	trig_event->enable = 0;
	trig_event->num_of_ANDs = 0;
	trig_event->params[0].adc_num = 0;
	trig_event->params[0].adc_val = 0;
	trig_event->params[0].operator = MORETHAN;
	trig_event->params[1].adc_num = 0;
	trig_event->params[1].adc_val = 0;
	trig_event->params[1].operator = MORETHAN;
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
void USBD_HID_Analog_IO_Process_Trigger_Event(uint8_t* output_buff, HID_ANALOG_IO_TRIGGER_Event* t)
{
	t->enable = output_buff[0];
	t->num_of_ANDs = output_buff[1];
	t->params[0].adc_num = output_buff[2];
	t->params[0].adc_val = output_buff[3];
	t->params[0].operator = output_buff[4];
	t->params[1].adc_num = output_buff[5];
	t->params[1].adc_val = output_buff[6];
	t->params[1].operator = output_buff[7];
}

/**
  * @brief  USBH_HID_Digital_IO_Init
  *         The function init the HID digital IO.
  * @retval USBH Status
  */
HID_ANALOG_IO_Trigger USBD_HID_Analog_IO_Check_Trigger_Event(HID_ANALOG_IO_TRIGGER_Event* t, uint8_t id)
{

	if (t->enable == 1)
	{
		switch(t->num_of_ANDs)
		{
			case 0:
				if(t->params[0].operator == MORETHAN)
				{
					if (analog_io.in[t->params[0].adc_num] >= t->params[0].adc_val)
					{
						return TRIGGERED;
					}
				}
				else
				{
					if (analog_io.in[t->params[0].adc_num] <= t->params[0].adc_val)
					{
						return TRIGGERED;
					}
				}
			break;
			case 1:
				if(t->params[0].operator == MORETHAN && t->params[1].operator == MORETHAN)
				{
					if ((analog_io.in[t->params[0].adc_num] >= t->params[0].adc_val) && (analog_io.in[t->params[1].adc_num] >= t->params[1].adc_val))
					{
						return TRIGGERED;
					}
				}
				else if(t->params[0].operator == MORETHAN && t->params[1].operator == LOWERTHAN)
				{
					if ((analog_io.in[t->params[0].adc_num] >= t->params[0].adc_val) && (analog_io.in[t->params[1].adc_num] <= t->params[1].adc_val))
					{
						return TRIGGERED;
					}
				}
				else if(t->params[0].operator == LOWERTHAN && t->params[1].operator == MORETHAN)
				{
					if ((analog_io.in[t->params[0].adc_num] <= t->params[0].adc_val) && (analog_io.in[t->params[1].adc_num] >= t->params[1].adc_val))
					{
						return TRIGGERED;
					}
				}
				else if(t->params[0].operator == LOWERTHAN && t->params[1].operator == LOWERTHAN)
				{
					if ((analog_io.in[t->params[0].adc_num] <= t->params[0].adc_val) && (analog_io.in[t->params[1].adc_num] <= t->params[1].adc_val))
					{
						return TRIGGERED;
					}
				}
			break;
		}

	}
	return DONTCARE;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
