/**
  ******************************************************************************
  * @file    usbd_digital_io.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_hid_core.c file.
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
/* Includes -------------------------------------*/
#include "usbd_customhid.h"


/* Defines -------------------------------------*/
#ifndef __USBD_ANALOG_IO_H
#define __USBD_ANALOG_IO_H

#define ANALOG_MAX_OUT_NUM		(0x01U)
#define ANALOG_MAX_IN_NUM 		(0x02U)

#define ANALOG_IO_MAX_TRIG_NUM (0x02U)

#define MASK_SHIFT(mask, nth) ((mask) << (nth))

#ifdef __cplusplus
 extern "C" {
#endif

 typedef enum {
   UNCHANGED,
   CHANGED
 } ANALOG_IO_Change_Flag;

 typedef enum {
	NO_REPORT,
	SEND_REPORT
 } ANALOG_IO_Report_Flag;


 typedef enum {
	 DONTCARE,
	 TRIGGERED,
	 DO_TRIGGER
 } HID_ANALOG_IO_Trigger;

 typedef enum {
	 LENGTH_NOTHING = 0,
	 LENGTH_TRIGGER = 2,
	 LENGTH_TRIGGER_EVENT = 8,
	 LENGTH_ANALOG_IO = 3,
 } HID_ANALOG_IO_Output;

 typedef enum {
	 OUT_CONST,
	 OUT_SINUS,
	 OUT_TRIANGLE
 } HID_ANALOG_IO_Out_Type;

 typedef enum {
	 MORETHAN,
	 LOWERTHAN
 } ANALOG_IO_Logic_Operator;

 typedef struct _HID_ANALOG_IO_Info
 {
   uint16_t     			in[ANALOG_MAX_IN_NUM];
   uint16_t					out[ANALOG_MAX_OUT_NUM];
   uint16_t					value[ANALOG_MAX_OUT_NUM];
   HID_ANALOG_IO_Out_Type 	type[ANALOG_MAX_OUT_NUM];
 } HID_ANALOG_IO_TypeDef;

 typedef struct _ANALOG_LOGICAL_Element_TypeDef
 {
	 uint8_t					adc_num;
	 ANALOG_IO_Logic_Operator	operator;
	 uint8_t  					adc_val;
 } ANALOG_LOGICAL_TypeDef;

 typedef struct _HID_ANALOG_IO_TRIGGER_Event
 {
	uint8_t 				enable;
	uint8_t					num_of_ANDs;
	ANALOG_LOGICAL_TypeDef	params[2];
 } HID_ANALOG_IO_TRIGGER_Event;


 extern HID_ANALOG_IO_TypeDef analog_io;
 extern HID_ANALOG_IO_TypeDef analog_io_new_state;
 extern HID_ANALOG_IO_Trigger analog_io_trigger;
 extern HID_ANALOG_IO_Trigger analog_io_trigger_io_change_flag;
 extern HID_ANALOG_IO_Trigger analog_io_do_trigger;;
 extern ANALOG_IO_Change_Flag analog_io_change_enable;
 extern ANALOG_IO_Change_Flag analog_io_change_flag;
 extern ANALOG_IO_Report_Flag analog_io_report_flag;
 extern HID_ANALOG_IO_TRIGGER_Event analog_io_trig_events[ANALOG_IO_MAX_TRIG_NUM];

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @param  phost: Host handle
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_Init(HID_ANALOG_IO_TypeDef* analog_io_instance);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @param  phost: Host handle
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_Reset_SwitchTrig(void);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_CreateReport(uint8_t* report);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_Set_Changes(uint8_t* output_buff);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_ANALOG_IO_Trigger (uint8_t* output_buff);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_SwitchConfig(void);


 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_Reset_Trigger_Event(HID_ANALOG_IO_TRIGGER_Event* trig_event);

 /**
   * @brief  USBH_HID_Digital_IO_Init
   *         The function init the HID digital IO.
   * @retval USBH Status
   */
 void USBD_HID_Analog_IO_Process_Trigger_Event(uint8_t* output_buff, HID_ANALOG_IO_TRIGGER_Event* t);


#ifdef __cplusplus
}
#endif

#endif  /* __USBD_DIGITAL_IO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
