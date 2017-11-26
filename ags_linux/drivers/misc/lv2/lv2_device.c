/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2017 Aligned Vision, Inc.  All rights reserved.
 *
 * Description:
 *	Aligned Vision LaserVision2 driver.  Refer to laser_api.h for commands
 *      available to control the LaserVision2 device.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/laser_api.h>
#include <linux/laser_dev.h>
#include <linux/serial_reg.h>

#define LG_VERSION 	 "0.5"

enum point_type {
  LV2_XPOINT=1,
  LV2_YPOINT,
  LV2_XYPOINT,
};

enum beam_type {
  LV2_BEAM_DARK=0,
  LV2_BEAM_DIM,
  LV2_BEAM_BRIGHT,
};

static inline void lv2_send_xy_to_dac(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off);
static void lv2_move_xydata_dark(struct lv2_xypoints *xyData);
static void lv2_move_xydata_dim(struct lv2_xypoints *xyData);
static void lv2_move_xydata_lite(struct lv2_xypoints *xyData);

static inline void lv2_get_xydata_ltcval(int16_t *output_val, int16_t input_val)
{
  if (!input_val)                                       // Value 0 is represented as 0x8000 by LTC1597
    *output_val = LTC1597_BIPOLAR_OFFSET_PLUS;
  else if (input_val < 0)                               // Negative values are 0x1->0x7FFF 
    *output_val = (input_val & LTC1597_BIPOLAR_OFFSET_NEG);
  else
    {
      if (input_val >= LTC1597_BIPOLAR_OFFSET_PLUS)    // Positive values are 0x8000->0xFFFF
	*output_val = input_val;
      else
	*output_val = input_val | LTC1597_BIPOLAR_OFFSET_PLUS;
    }
  return;
}
static inline void lv2_send_to_dac(int16_t point, uint8_t strobe_on, uint8_t strobe_off, uint8_t point_type)
{
    int16_t       dac_val;
    int8_t        hi_point;
    int8_t        lo_point;
    
    if (point_type == 0)
      {
	printk("bad data point type\n");
	return;
      }
    // Adjust data for producing correct LTC1597 output voltage to DAC
    // This function also avoids fault condition.
    lv2_get_xydata_ltcval((int16_t *)&dac_val, point);

    // Data is applied to DAC input after lo byte is written
    // so sequence is important.  hi byte then lo byte.
    hi_point = (int8_t)(dac_val >> 8) & 0xFF;
    lo_point = (int8_t)(dac_val & 0xFF);

    // write to X or Y
    if (point_type & LV2_XPOINT)
      {
	outb(hi_point, LG_IO_XH);
	outb(lo_point, LG_IO_XL);
      }
    else
      {
	outb(hi_point, LG_IO_YH);
	outb(lo_point, LG_IO_YL);
      }
    // Let data WRITE to DAC input register operation take place
    outb(strobe_on, LG_IO_CNTRL1);
    // Strobe bit 0->1 latches data,
    // Strobe bit 1->0 writes data to DAC
    outb(strobe_off, LG_IO_CNTRL1);
    return;
}
static inline void lv2_send_xy_to_dac(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    int16_t       dac_val;
    int8_t        hi_xpoint;
    int8_t        lo_xpoint;
    int8_t        hi_ypoint;
    int8_t        lo_ypoint;
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off == LV2_BEAM_BRIGHT)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else if  (beam_on_off == LV2_BEAM_DARK)
      beam_setting &= LASERDISABLE;  // Dark move, disable laser.
    else
      beam_setting = (beam_setting & DIMBEAM) | LASERENABLE;
      
    outb(beam_setting, LG_IO_CNTRL2);
    
    // Adjust data for producing correct LTC1597 output voltage to DAC
    // This function also avoids fault condition.
    // Data is applied to DAC input after lo byte is written
    // so sequence is important.  hi byte then lo byte.

    // convert xpoint
    lv2_get_xydata_ltcval((int16_t *)&dac_val, xyData->xPoint);
    hi_xpoint = (int8_t)(dac_val >> 8) & 0xFF;
    lo_xpoint = (int8_t)(dac_val & 0xFF);

    // convert ypoint
    lv2_get_xydata_ltcval((int16_t *)&dac_val, xyData->yPoint);
    hi_ypoint = (int8_t)(dac_val >> 8) & 0xFF;
    lo_ypoint = (int8_t)(dac_val & 0xFF);

    // write X & Y to DAC thru FPGA
    outb(hi_xpoint, LG_IO_XH);
    outb(lo_xpoint, LG_IO_XL);
    outb(hi_ypoint, LG_IO_YH);
    outb(lo_ypoint, LG_IO_YL);

    // Let data WRITE to DAC input register operation take place
    outb(strobe_on, LG_IO_CNTRL1);
    // Strobe bit 0->1 latches data,
    // Strobe bit 1->0 writes data to DAC
    outb(strobe_off, LG_IO_CNTRL1);
    return;
}
static void lv2_sense_point(int16_t point, uint8_t point_type, uint8_t *sense_val, uint8_t sense_delay)
{    
    // Write x-data to DAC
    lv2_send_to_dac(point, STROBE_ON_LASER_ON, STROBE_OFF_LASER_ON, point_type);

    // Read from target-find register
    // Debounce to get rid of false-positive target finds
    *sense_val = inb(TFPORTRL); 
    udelay(sense_delay);    // Wait minimum of 20 usec (minimum is set to 25) to process next x or y point.
    return;
}
static void lv_adjust_start_xy(int16_t origX, int16_t origY, int16_t *newStartX, int16_t *newStartY, struct lv2_sense_line_data *pSenseInfo)
{
    int32_t       add_maxX;
    int32_t       add_maxY;
    int32_t       sub_minX;
    int32_t       sub_minY;
  
    *newStartX = origX;
    *newStartY = origY;

    // Make sure to do 32-bit operation to account for overflow
    add_maxX = origX + (pSenseInfo->step * pSenseInfo->numPoints); 
    add_maxY = origY + (pSenseInfo->step * pSenseInfo->numPoints); 
    sub_minX = origX - (pSenseInfo->step * pSenseInfo->numPoints); 
    sub_minY = origY - (pSenseInfo->step * pSenseInfo->numPoints); 
    printk("AV-LV2: ADJUST_X: add_max = %d[%x], sub_min = %d[%x]\n", add_maxX, add_maxX, sub_minX, sub_minX);
    printk("AV-LV2: ADJUST_Y: add_max = %d[%x], sub_min = %d[%x]\n", add_maxY, add_maxY, sub_minY, sub_minY);
	
    // Check positive direction
    if ((int16_t)add_maxX > DAC_MAX_SIGNED)
      *newStartX -= (int16_t)(add_maxX - DAC_MAX_SIGNED);
    if ((int16_t)add_maxY > DAC_MAX_SIGNED)
      *newStartY -= (int16_t)(add_maxY - DAC_MAX_SIGNED);
    // Check negative direction
    if ((int16_t)sub_minX < DAC_MIN_SIGNED)
      *newStartX += (int16_t)(sub_minX - DAC_MIN_SIGNED);
    if ((int16_t)sub_minY < DAC_MIN_SIGNED)
      *newStartY += (int16_t)(sub_minY - DAC_MIN_SIGNED);

    // Check to see if it changed (debug-mode only)
    if (*newStartX != origX)
      {
	printk("AV-LV2: ADJUST_X CHANGE!  OrigX = %d[%x], NewX=%d[%x]\n", origX, origX, *newStartX, *newStartX);
      }
    if (*newStartY != origY)
      {
	printk("AV-LV2: ADJUST_Y CHANGE!  OrigY = %d[%x], NewY=%d[%x]\n", origY, origY, *newStartY, *newStartY);
      }
    return;
}

void do_line_sense_operation(struct lv2_sense_line_data *pSenseInfo, struct write_sense_cs_data *pSenseData, uint32_t point_axis, uint8_t step_direction)
{
    int16_t                    point;
    int16_t                    target_xPoint = 0;
    int16_t                    target_yPoint = 0;
    uint16_t                   i;
    uint8_t                    sense_delay;
    uint8_t                    sense_val;
    uint8_t                    target_sense_val;

    if (point_axis == LV2_XPOINT)
      point = pSenseInfo->xPoint;
    else
      point = pSenseInfo->yPoint;
    
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    target_sense_val = 0xFF;

    for (i = 0; i < pSenseInfo->numPoints; i++)
      {
	if (step_direction == LV2_ADD_STEP)
	    point += pSenseInfo->step;
	else if (step_direction == LV2_SUB_STEP)
	    point -= pSenseInfo->step;
	lv2_sense_point(point, point_axis, &sense_val, sense_delay);

#if 0
	// We know this works now, only turn on for debugging
	if (point_axis == LV2_XPOINT)
	  {
	    printk("AV-LV2: DO-LINE-SENSE X = %d [%x], Y = %d [%x], sense-val=%x, step=%d, numPts=%d\n",
		   point, point, pSenseInfo->yPoint, pSenseInfo->yPoint,
		   sense_val, pSenseInfo->step, pSenseInfo->numPoints);
	  }
	else
	  {
	    printk("AV-LV2: DO-LINE-SENSE X = %d [%x], Y = %d [%x], sense-val=%x, step=%d, numPts=%d\n",
		   pSenseInfo->xPoint, pSenseInfo->xPoint, point, point,
		   sense_val, pSenseInfo->step, pSenseInfo->numPoints);
	  }
#endif
	pSenseData[pSenseInfo->sense_buf_idx].sense_val = sense_val;

	if (point_axis == LV2_XPOINT)
	  {
	    pSenseData[pSenseInfo->sense_buf_idx].xPoint = point;
	    pSenseData[pSenseInfo->sense_buf_idx].yPoint = pSenseInfo->yPoint;
	  }
	else
	  {
	    pSenseData[pSenseInfo->sense_buf_idx].yPoint = point;
	    pSenseData[pSenseInfo->sense_buf_idx].xPoint = pSenseInfo->xPoint;
	  }

	pSenseInfo->sense_buf_idx++;

	// Look for lowest sense-val (indicates closest point to actual target)
	if (sense_val < target_sense_val)
	  {
	    if (point_axis == LV2_XPOINT)
	      {
		target_xPoint = point;
		target_yPoint = pSenseInfo->yPoint;
	      }
	    else
	      {
		target_xPoint = pSenseInfo->xPoint;
		target_yPoint = point;
	      }
	    target_sense_val = sense_val;
	  }
      }
    return;
}

void lv2_senseX_add(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_XPOINT, LV2_ADD_STEP);
    return;
}

void lv2_senseY_add(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer, so lock now.
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_YPOINT, LV2_ADD_STEP);
    return;
}
void lv2_senseX_sub(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_XPOINT, LV2_SUB_STEP);
    return;
}

void lv2_senseY_sub(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer, so lock now.
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_YPOINT, LV2_SUB_STEP);
    return;
}
static void lv2_coarse_scan_box_op(struct lv2_info *priv, struct lv2_sense_info *pSenseInfo, uint8_t laser_setting)
{      
    struct lv2_sense_line_data  new_point;
    struct write_sense_cs_data *pSenseData;
    uint8_t                     sense_delay;
    uint8_t                     beam_setting;
    uint8_t                     sense_val;
    
    if (priv == NULL)
      {
	printk("AV-CS-BOX:   Bad pointer to private data\n");
	return;
      }
    pSenseData = (struct write_sense_cs_data *)priv->pSenseBuff;
    if (pSenseData == NULL)
      {
	printk("AV-CS-BOX:  Sensor buffer not allocated properly\n");
	return;
      }
    memset((uint8_t *)pSenseData, 0, sizeof(struct write_sense_cs_data));

    // Make sure to take device lock before starting.
    spin_lock(&priv->lock);

    // Get initial val of LG_IO_CNTRL2 register
    beam_setting = inb(LG_IO_CNTRL2);

    // Set up delay
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    // These values are pushed from user & are constant
    new_point.step      = pSenseInfo->step;
    new_point.numPoints = pSenseInfo->numPoints;
    new_point.point_delay = pSenseInfo->point_delay;
    new_point.point_delay = SENSOR_MIN_DELAY;

    // Adjust start X/Y so they don't cause system fault
    // conditions
    lv_adjust_start_xy(pSenseInfo->xData, pSenseInfo->yData, &new_point.xPoint, &new_point.yPoint, &new_point);
    
    // NOTE:  Leave next lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense & debounce sense-buffer-register
    // Turn on beam, dim intensity  (sensing same point causes too-bright compared to others)
    if (laser_setting == LV2_LASER_ON)
      beam_setting = (beam_setting & DIMBEAM) | LASERENABLE;
    else
      beam_setting &= LASERDISABLE;            // dark move, disable laser
    outb(beam_setting, LG_IO_CNTRL2);
    lv2_sense_point(new_point.yPoint, LV2_YPOINT, &sense_val, sense_delay);
    sense_val = inb(TFPORTRL);
    sense_val = inb(TFPORTRL);
    sense_val = inb(TFPORTRL);
    // NOTE:  END OF special-case first point.
    
    // Turn laser on or off, depending on laser-setting flag
    if (laser_setting == LV2_LASER_ON)
      beam_setting |= LASERENABLE | BRIGHTBEAM;   // light move, enable laser.
    else
      beam_setting &= LASERDISABLE;            // dark move, disable laser
    outb(beam_setting, LG_IO_CNTRL2);
    
    // left side, UP (Y-ADD)
    new_point.sense_buf_idx = 0;
    lv2_senseY_add(pSenseData, &new_point);

    // left-to-right, across top (X-ADD)
    new_point.xPoint = pSenseInfo->xData;
    new_point.yPoint = pSenseInfo->yData + (new_point.numPoints * new_point.step);
    lv2_senseX_add(pSenseData, &new_point);

    // Advance Y to top right corner of box
    // Right side, DOWN (Y-SUB)
    new_point.xPoint = pSenseInfo->xData + (new_point.numPoints * new_point.step);
    new_point.yPoint = pSenseInfo->yData + (new_point.numPoints * new_point.step);
    lv2_senseY_sub(pSenseData, &new_point);

    // Advance X to top right corner of box
    // right-to-left, across bottom (X-SUB)
    new_point.xPoint = pSenseInfo->xData + (new_point.numPoints * new_point.step);
    new_point.yPoint = pSenseInfo->yData;
    lv2_senseX_sub(pSenseData, &new_point);
    return;	
}

//  This function is the main coarse-scan function used by AGSD
int lv2_coarse_scan_box(struct lv2_info *priv, struct lv2_sense_info *pSenseInfo)
{
    lv2_coarse_scan_box_op(priv, pSenseInfo, LV2_LASER_ON);

    // Box complete, return to caller
    return(0);
}

//  This function is only used for hardware debug purposes
int lv2_coarse_scan_box_dark(struct lv2_info *priv, struct lv2_sense_info *pSenseInfo)
{
    lv2_coarse_scan_box_op(priv, pSenseInfo, LV2_LASER_OFF);
    // Box complete, return to caller
    return(0);
}

void do_line_fssc_operation(struct lv2_sense_line_data *pSenseInfo, int16_t *endPoint, uint32_t point_axis, uint8_t step_direction, int16_t *target_point, uint8_t *target_sense_val)
{
    uint16_t                   numPoints;
    int16_t                    point;
    uint16_t                   i;
    uint8_t                    sense_delay;
    uint8_t                    sense_val;
    uint8_t                    beam_setting;

    if (point_axis == LV2_XPOINT)
      point  = pSenseInfo->xPoint;
    else
      point  = pSenseInfo->yPoint;

    // Set up delay
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    // Do the first-point 3 times to avoid false readings while LASER comes up
    // Set beam to LASER ENABLED & ON, DIM SETTING
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= (LASERENABLE | BRIGHTBEAM);   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);
    
    lv2_sense_point(point, point_axis, &sense_val, sense_delay);
    lv2_sense_point(point, point_axis, &sense_val, sense_delay);
    lv2_sense_point(point, point_axis, &sense_val, sense_delay);

    // NOTE:  numPoints is a guestimate on how big the target may be, it could be user-provided?
    numPoints = 1000;
    *target_sense_val = 0xFF;
    for (i = 0; i < numPoints; i++)
      {
	if (step_direction == LV2_ADD_STEP)
	  point+= 2;
	else if (step_direction == LV2_SUB_STEP)
	  point-= 2;

	lv2_sense_point(point, point_axis, &sense_val, sense_delay);
	printk("FSSC-DO-LINE: point=%d,sense_val=%x\n", point, sense_val);
	// Look for lowest sense-val (indicates closest point to actual target)
	if (sense_val < *target_sense_val)
	  {
	    *target_point = point;
	    *target_sense_val = sense_val;
	  }
	
	if (sense_val > FSSC_THRESHOLD)
	  {
	     *endPoint = point;
	     return;
	  }
	udelay(sense_delay);    // Wait 40 usec to process next.
      }
    if (point_axis == LV2_XPOINT)
      *endPoint  = pSenseInfo->xPoint;
    else
      *endPoint  = pSenseInfo->yPoint;
    return;
}

void lv2_FindYEndpoint_add(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo, int16_t *target_point, int8_t *target_sense_val)
{
    int16_t        endPoint;
    
    printk("FSSC-Y-ADD: TOP-END:  startXY=%d[%x],%d[%x]\n", pSenseInfo->xPoint,pSenseInfo->xPoint,pSenseInfo->yPoint,pSenseInfo->yPoint);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_YPOINT, LV2_ADD_STEP, target_point, target_sense_val);
    pSenseWriteData->TopEndpoints.xPoint = pSenseInfo->xPoint;
    pSenseWriteData->TopEndpoints.yPoint = endPoint;
    return;
}

void lv2_FindYEndpoint_sub(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo, int16_t *target_point, int8_t *target_sense_val)
{
    int16_t        endPoint;
    
    printk("FSSC-Y-SUB: BOTTOM-END: startXY=%d[%x],%d[%x]\n", pSenseInfo->xPoint,pSenseInfo->xPoint,pSenseInfo->yPoint,pSenseInfo->yPoint);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_YPOINT, LV2_SUB_STEP, target_point, target_sense_val);
    pSenseWriteData->BottomEndpoints.xPoint = pSenseInfo->xPoint;
    pSenseWriteData->BottomEndpoints.yPoint = endPoint;
    return;
}

void lv2_FindXEndpoint_add(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo, int16_t *target_point, int8_t *target_sense_val)
{
    int16_t        endPoint;
    
    printk("FSSC-X-ADD: RIGHT-ENDS:  startXY=%d[%x],%d[%x]\n", pSenseInfo->xPoint,pSenseInfo->xPoint,pSenseInfo->yPoint,pSenseInfo->yPoint);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_XPOINT, LV2_ADD_STEP, target_point, target_sense_val);
    pSenseWriteData->RightEndpoints.xPoint = endPoint;
    pSenseWriteData->RightEndpoints.yPoint = pSenseInfo->yPoint;
    return;
}

void lv2_FindXEndpoint_sub(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo, int16_t *target_point, int8_t *target_sense_val)
{
    int16_t        endPoint;
    
    printk("FSSC-X-SUB: LEFT-ENDS: startXY=%d[%x],%d[%x]\n", pSenseInfo->xPoint,pSenseInfo->xPoint,pSenseInfo->yPoint,pSenseInfo->yPoint);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_XPOINT, LV2_SUB_STEP, target_point, target_sense_val);
    pSenseWriteData->LeftEndpoints.xPoint = endPoint;
    pSenseWriteData->LeftEndpoints.yPoint = pSenseInfo->yPoint;
    return;
}

int lv2_find_ssc_center(struct lv2_xypoints *startXY, int16_t *CtrX, int16_t *CtrY)
{
    struct write_sense_cs_data    tmp_sense_data;
    struct write_sense_cs_data    low_sense_data[4];
    struct lv2_xypoints           xyData;
    int                           i;
    int16_t                       ctrX;
    int16_t                       ctrY;
    uint8_t                       start_sense_val;
    uint8_t                       low_sense_val=0xFF;
    uint8_t                       low_index=0xFF;
    
    // Do a "mini-cross" section of the point found during coarse-scan to locate center
    // point of the target
    // Move beam to x, y spot
    xyData.xPoint = startXY->xPoint;
    xyData.yPoint = startXY->yPoint;
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);

    // NOTE:  Leave next 3 lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense for first point
    lv2_sense_point(startXY->xPoint, LV2_XPOINT, (uint8_t *)&start_sense_val, FSSC_LOW_MIN_DELAY);
    lv2_sense_point(startXY->xPoint, LV2_XPOINT, (uint8_t *)&start_sense_val, FSSC_LOW_MIN_DELAY);
    lv2_sense_point(startXY->xPoint, LV2_XPOINT, (uint8_t *)&start_sense_val, FSSC_LOW_MIN_DELAY);
    // Seed the low point structs
    for (i = 0; i < 3; i++)
      {
	low_sense_data[i].sense_val = start_sense_val;
	low_sense_data[i].xPoint  = startXY->xPoint;
	low_sense_data[i].yPoint  = startXY->yPoint;
      }

    ctrX = startXY->xPoint;
    ctrY = startXY->yPoint;

    // Collect sense info with X axis
    // X-ADD
    for (i = 0; i < FSSC_LOW_COUNT; i++)
      {
	tmp_sense_data.xPoint = ctrX + (i + FSSC_LOW_STEP);
	tmp_sense_data.yPoint = ctrY;
	lv2_sense_point(tmp_sense_data.xPoint, LV2_XPOINT, (uint8_t *)&tmp_sense_data.sense_val, FSSC_LOW_MIN_DELAY);
	if (tmp_sense_data.sense_val <= low_sense_data[0].sense_val)
	  {
	    low_sense_data[0].sense_val = tmp_sense_data.sense_val;
	    low_sense_data[0].xPoint = tmp_sense_data.xPoint;
	    low_sense_data[0].yPoint = tmp_sense_data.yPoint;
	    printk("FSSC_CTR:  new low:  XY=%d[%x],%d[%x] sense_val=%x\n",
		   tmp_sense_data.xPoint, tmp_sense_data.xPoint,
		   tmp_sense_data.yPoint, tmp_sense_data.yPoint,
		   tmp_sense_data.sense_val);
	  }
      }
    // Move beam back to x, y spot
    ctrX = startXY->xPoint;
    ctrY = startXY->yPoint;
    xyData.xPoint = ctrX;
    xyData.yPoint = ctrY;
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);
    
    // X-SUB
    for (i = 0; i < FSSC_LOW_COUNT; i++)
      {
	tmp_sense_data.xPoint = ctrX + (i - FSSC_LOW_STEP);
	tmp_sense_data.yPoint = ctrY;
	lv2_sense_point(tmp_sense_data.xPoint, LV2_XPOINT, (uint8_t *)&tmp_sense_data.sense_val, FSSC_LOW_MIN_DELAY);
	if (tmp_sense_data.sense_val <= low_sense_data[1].sense_val)
	  {
	    low_sense_data[1].sense_val = tmp_sense_data.sense_val;
	    low_sense_data[1].xPoint = tmp_sense_data.xPoint;
	    low_sense_data[1].yPoint = tmp_sense_data.yPoint;
	    printk("FSSC_CTR:  new low:  XY=%d[%x],%d[%x] sense_val=%x\n",
		   tmp_sense_data.xPoint, tmp_sense_data.xPoint,
		   tmp_sense_data.yPoint, tmp_sense_data.yPoint,
		   tmp_sense_data.sense_val);
	  }
      }
    // Collect sense info with Y axis
    ctrX = startXY->xPoint;
    ctrY = startXY->yPoint;

    // Move beam to new x, y spot
    xyData.xPoint = ctrX;
    xyData.yPoint = ctrY;
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);
    
    // Y-ADD
    for (i = 0; i < FSSC_LOW_COUNT; i++)
      {
	tmp_sense_data.xPoint = ctrX;
	tmp_sense_data.yPoint = ctrY + (i + FSSC_LOW_STEP);
	lv2_sense_point(tmp_sense_data.yPoint, LV2_YPOINT, (uint8_t *)&tmp_sense_data.sense_val, FSSC_LOW_MIN_DELAY);
	if (tmp_sense_data.sense_val <= low_sense_data[2].sense_val)
	  {
	    low_sense_data[2].sense_val = tmp_sense_data.sense_val;
	    low_sense_data[2].xPoint = tmp_sense_data.xPoint;
	    low_sense_data[2].yPoint = tmp_sense_data.yPoint;
	    printk("FSSC_CTR:  new low:  XY=%d[%x],%d[%x] sense_val=%x\n",
		   tmp_sense_data.xPoint, tmp_sense_data.xPoint,
		   tmp_sense_data.yPoint, tmp_sense_data.yPoint,
		   tmp_sense_data.sense_val);
	  }
      }
    // Y-SUB
    ctrX = startXY->xPoint;
    ctrY = startXY->yPoint;

    // Move beam to new x, y spot
    xyData.xPoint = ctrX;
    xyData.yPoint = ctrY;
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);
    
    for (i = 0; i < FSSC_LOW_COUNT; i++)
      {
	tmp_sense_data.xPoint = ctrX;
	tmp_sense_data.yPoint = ctrY + (i - FSSC_LOW_STEP);
	lv2_sense_point(tmp_sense_data.yPoint, LV2_YPOINT, (uint8_t *)&tmp_sense_data.sense_val, FSSC_LOW_MIN_DELAY);
	if (tmp_sense_data.sense_val <= low_sense_data[3].sense_val)
	  {
	    low_sense_data[3].sense_val = tmp_sense_data.sense_val;
	    low_sense_data[3].xPoint = tmp_sense_data.xPoint;
	    low_sense_data[3].yPoint = tmp_sense_data.yPoint;
	    printk("FSSC_CTR:  new low:  XY=%d[%x],%d[%x] sense_val=%x\n",
		   tmp_sense_data.xPoint, tmp_sense_data.xPoint,
		   tmp_sense_data.yPoint, tmp_sense_data.yPoint,
		   tmp_sense_data.sense_val);
	  }
      }
    // Do we have a good center?
    low_sense_val = low_sense_data[0].sense_val;
    low_index = 0;
    printk("FSSC_CTR:  orig low_sense_val=%x\n", low_sense_val);
    for (i = 1; i < 3; i++)
      {
	if (low_sense_data[i].sense_val <= low_sense_val)
	  {
	    low_sense_val = low_sense_data[i].sense_val;
	    low_index = i;
	    printk("FSSC_CTR:  found new low_sense_val=%x\n", low_sense_val);
	  }
      }
    if (low_sense_val <= FSSC_LOW_THRESHOLD)
      {
	*CtrX = low_sense_data[low_index].xPoint;
	*CtrY = low_sense_data[low_index].yPoint;
	printk("FSSC_CTR:  Dead Center: XY =%x,%x, sense=%x, ORIG XY=%x,%x\n",
	       *CtrX, *CtrY, low_sense_data[low_index].sense_val,
	       startXY->xPoint, startXY->yPoint);
	return(0);
      }
    printk("FSSC_CTR:  TARGET NOT FOUND FOR STARTXY %x,%x\n", startXY->xPoint, startXY->yPoint);
    return(-1);
}

int lv2_find_ss_coords(struct lv2_info *priv, struct lv2_sense_info *pSenseInfo)
{
    struct lv2_sense_line_data     new_point;
    struct lv2_xypoints            xyData;
    struct write_sense_fssc_data   *pSenseData;
    int                            ret;
    int16_t                        target_point1;
    int16_t                        target_point2;
    uint8_t                        target_sense_val1;
    uint8_t                        target_sense_val2;
    uint8_t                        beam_setting;
    
    pSenseData = (struct write_sense_fssc_data *)priv->pSenseBuff;
    if (pSenseData == NULL)
      {
	printk("AV-FSSC-BOX:  Sensor buffer not allocated properly\n");
	return(-ENOMEM);
      }

    memset((uint8_t *)pSenseData, 0, sizeof(struct write_sense_fssc_data));
    
    // Make sure to take device lock before starting.
    spin_lock(&priv->lock);

    // These values are pushed from user & are constant
    new_point.step      = pSenseInfo->step;
    new_point.point_delay = pSenseInfo->point_delay;
    new_point.sense_buf_idx = 0;
    new_point.point_delay = SENSOR_MIN_DELAY;
    
    // Turn on beam, bright intensity
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= (LASERENABLE | BRIGHTBEAM);   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);
    udelay(100);
    
    // Start with input values for X & Y
    xyData.xPoint = pSenseInfo->xData;
    xyData.yPoint = pSenseInfo->yData;
    printk("FSSC_CTR:  START XY=%x,%x\n", xyData.xPoint, xyData.yPoint);
    ret = lv2_find_ssc_center((struct lv2_xypoints *)&xyData, (int16_t *)&xyData.xPoint, (int16_t *)&xyData.yPoint);
    if (ret < 0)
      {
	printk("FSSC: FIND-CTR FAILED, no center found around startXY=%d[%x],%d[%x]\n",
	       pSenseInfo->xData, pSenseInfo->xData,
	       pSenseInfo->yData, pSenseInfo->yData);
	return(-EINVAL);
      }
    printk("FSSC:  FIND-CTR RESULT:  ORIG=%x,%x, adjusted=%x,%x\n",
	   pSenseInfo->xData, pSenseInfo->yData,
	   xyData.xPoint, xyData.yPoint);

    // Move beam to x, y spot
    new_point.xPoint = xyData.xPoint;
    new_point.yPoint = xyData.yPoint;
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);

    // Go UP (Y-ADD), search for top endpoint
    printk("FSSC-YADD: XY=%x,%x\n", new_point.xPoint, new_point.yPoint);
    lv2_FindYEndpoint_add(pSenseData, &new_point, &target_point2, &target_sense_val2);
    printk("FSSC-YADD:  yPoint2=%x, sense-val=%d\n", target_point2, target_sense_val2);
    
    // Move back to original XY
    new_point.xPoint = xyData.xPoint;
    new_point.yPoint = xyData.yPoint;
    printk("FSSC-MOVING BACK TO ORIG %x,%x\n", xyData.xPoint, xyData.yPoint);
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);

    // Go Down (Y-SUB), search for bottom endpoint
    printk("FSSC-YSUB: XY=%x,%x\n", new_point.xPoint, new_point.yPoint);
    lv2_FindYEndpoint_sub(pSenseData, &new_point, &target_point1, &target_sense_val1);
    printk("FSSC-YSUB:  yPoint1=%x, sense-val=%x\n", target_point1, target_sense_val1);

    // Move to Y midpoint to start search left & right
    xyData.yPoint = (target_point2 + target_point1) / 2;
      
    printk("FSSC-MOVING Y to midpoint: %x,%x\n", xyData.xPoint, xyData.yPoint);
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);

    // Set XY to new start-point, Go right (X-ADD), search for right-side endpoint
    new_point.xPoint = xyData.xPoint;
    new_point.yPoint = xyData.yPoint; 
    printk("FSSC-XADD: start XY=%d,%d\n", new_point.xPoint, new_point.yPoint);
    lv2_FindXEndpoint_add(pSenseData, &new_point, &target_point1, &target_sense_val1);
    printk("FSSC-XADD: xPoint1=%x, sense-val=%x\n", target_point1, target_sense_val1);
    
    // Move back to mid-point to start search left & right
    printk("FSSC-MOVING back to point %x,%x\n", xyData.xPoint, xyData.yPoint);
    lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);

    new_point.xPoint = xyData.xPoint;
    new_point.yPoint = xyData.yPoint;

    // Go left (X-SUB), search for bottom endpoint
    printk("FSSC-XSUB: XY=%d,%d\n", new_point.xPoint, new_point.yPoint);
    lv2_FindXEndpoint_sub(pSenseData, &new_point, &target_point2, &target_sense_val2);
    printk("FSSC-XADD: xPoint2=%d, sense-val=%x\n", target_point2, target_sense_val2);
    printk("FSSC: END:  LeftXY=%d,%d; RightXY=%d,%d; TopXY=%d,%d; BottomXY=%d,%d\n",
	   pSenseData->LeftEndpoints.xPoint,
	   pSenseData->LeftEndpoints.yPoint,
	   pSenseData->RightEndpoints.xPoint,
	   pSenseData->RightEndpoints.yPoint,
	   pSenseData->TopEndpoints.xPoint,
	   pSenseData->TopEndpoints.yPoint,
	   pSenseData->BottomEndpoints.xPoint,
	   pSenseData->BottomEndpoints.yPoint);
    return(0);
}

int lv2_super_scan(struct lv2_info *priv, struct lv2_ss_sense_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData;
    struct lv2_sense_line_data     new_point;
    struct lv2_xypoints            xyData;
    uint32_t                       i;
    uint32_t                       line_count;
    int16_t                        startX;
    int16_t                        endX;
    uint8_t                        sense_delay;
    uint8_t                        sense_val;
    uint8_t                        beam_setting;
    
    pSenseData = (struct write_sense_cs_data *)priv->pSenseBuff;
    if (pSenseData == NULL)
      {
	printk("AV-FSSC-BOX:  Sensor buffer not allocated properly\n");
	return(-ENOMEM);
      }

    // Make sure to take device lock before starting.
    spin_lock(&priv->lock);

    // Clean slate with read sense buffer
    memset((uint8_t *)pSenseData, 0, pSenseInfo->numPoints * pSenseInfo->numLines * sizeof(struct write_sense_fssc_data));
        
    // Set up delay
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    // These values are pushed from user & are constant
    new_point.step        = pSenseInfo->step;
    new_point.numPoints   = pSenseInfo->numPoints;
    new_point.point_delay = sense_delay;
    new_point.sense_buf_idx = 0;
    startX = pSenseInfo->xData;
    endX = pSenseInfo->xData + (pSenseInfo->numPoints * pSenseInfo->step);
    
    // Start with input X,Y
    new_point.xPoint  = startX;
    new_point.yPoint  = pSenseInfo->yData;

    // Turn on beam, bright intensity
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= LASERENABLE | BRIGHTBEAM;   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);
    udelay(100);

    // NOTE:  Leave next 3 lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense for first 2 points
    lv2_sense_point(new_point.yPoint, LV2_YPOINT, &sense_val, sense_delay);
    lv2_sense_point(new_point.yPoint, LV2_YPOINT, &sense_val, sense_delay);
    lv2_sense_point(new_point.yPoint, LV2_YPOINT, &sense_val, sense_delay);

    line_count = 0;
    xyData.yPoint = pSenseInfo->yData;
    xyData.xPoint = startX;
    while (line_count < pSenseInfo->numLines)
      {
	// Go right (X-ADD), move xy into position
	lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);
	
	// Sense line, going right (X-ADD, ->)
	new_point.xPoint = xyData.xPoint;
	new_point.yPoint = xyData.yPoint;
	printk("SUPERSCAN Line# %d: (X-ADD) XY=%d[%x],%d[%x]\n",
	       line_count, xyData.xPoint, xyData.xPoint,
	       xyData.yPoint, xyData.yPoint);
	lv2_senseX_add(pSenseData, &new_point);

	// Next line
	line_count++;
	xyData.yPoint++;
	xyData.xPoint = endX;

	// Move xy in dim to position
	lv2_move_xydata_lite((struct lv2_xypoints *)&xyData);
	
	// Sense line, going left (X-SUB, <-)
	new_point.xPoint = xyData.xPoint;
	new_point.yPoint = xyData.yPoint;
	printk("SUPERSCAN Line# %d: (X-SUB) XY=%d[%x],%d[%x]\n",
	       line_count, xyData.xPoint, xyData.xPoint,
	       xyData.yPoint, xyData.yPoint);
	lv2_senseX_sub(pSenseData, &new_point);

	// Next line
	line_count++;
	xyData.yPoint++;
	xyData.xPoint = startX;
      }
    return(0);
}

void lv2_sense_one_ypoint(struct lv2_info *priv, struct lv2_sense_one_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData=(struct write_sense_cs_data *)priv->pSenseBuff;
    int16_t                    point;
    uint8_t                    sense_val;

    if (pSenseData == NULL)
      return;
    
    // About to write to sensor buffer, so lock now.
    point = pSenseInfo->data;
    lv2_sense_point(point, LV2_YPOINT, &sense_val, SENSOR_MIN_DELAY);
    pSenseData[pSenseInfo->index].sense_val = sense_val;
    pSenseData[pSenseInfo->index].yPoint = point;
    printk("Sense yPoint[%d]=%d; sense_read_data=%x\n",pSenseInfo->index, point, pSenseData[pSenseInfo->index].sense_val);
    return;
}
void lv2_sense_one_xpoint(struct lv2_info *priv, struct lv2_sense_one_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData=(struct write_sense_cs_data *)priv->pSenseBuff;
    int16_t    point;
    uint8_t    sense_val;
    
    if (pSenseData == NULL)
      return;
    
    // About to write to sensor buffer, so lock now.
    spin_lock(&priv->lock);
    point = pSenseInfo->data;
    lv2_sense_point(point, LV2_XPOINT, &sense_val, SENSOR_MIN_DELAY);
    pSenseData[pSenseInfo->index].sense_val = sense_val;
    pSenseData[pSenseInfo->index].xPoint = point;
    printk("Sense xPoint[%d]=%d; rtn_data=%x; buf_data=%x\n",pSenseInfo->index, point, sense_val, pSenseData[pSenseInfo->index].sense_val);
    return;
}
#if 0
// FIXME---PAH---MAY NOT NEED THESE
static void lv2_move_xdata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off == LV2_BEAM_BRIGHT)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else if  (beam_on_off == LV2_BEAM_DARK)
      beam_setting &= LASERDISABLE;  // Dark move, disable laser.
    else
      beam_setting = (beam_setting & DIMBEAM) | LASERENABLE;
    outb(beam_setting, LG_IO_CNTRL2);  // Apply CNTRL2 setting
    lv2_send_to_dac(xyData->xPoint, strobe_on, strobe_off, LV2_XPOINT);
    return;
}
static void lv2_move_ydata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off == LV2_BEAM_BRIGHT)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else if  (beam_on_off == LV2_BEAM_DARK)
      beam_setting &= LASERDISABLE;  // Dark move, disable laser.
    else
      beam_setting = (beam_setting & DIMBEAM) | LASERENABLE;
    outb(beam_setting, LG_IO_CNTRL2);  // Apply CNTRL2 settings
    lv2_send_to_dac(xyData->yPoint, strobe_on, strobe_off, LV2_YPOINT);
    return;
}
#endif
static void lv2_move_xydata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    lv2_send_xy_to_dac(xyData, strobe_on, strobe_off, beam_on_off);
    return;
}
static void lv2_move_xydata_dark(struct lv2_xypoints *xyData)
{
    printk("lv2_move_dark:  x=%x,y=%x", xyData->xPoint, xyData->yPoint);
    lv2_move_xydata(xyData, STROBE_ON_LASER_OFF, STROBE_OFF_LASER_OFF, LV2_BEAM_DARK);
    return;
}
static void lv2_move_xydata_dim(struct lv2_xypoints *xyData)
{
  //    printk("lv2_move_dim:  x=%x,y=%x", xyData->xPoint, xyData->yPoint);
    lv2_move_xydata(xyData, STROBE_ON_LASER_ON, STROBE_OFF_LASER_ON, LV2_BEAM_DIM);
    return;
}
static void lv2_move_xydata_lite(struct lv2_xypoints *xyData)
{
  //    printk("lv2_move_lite:  x=%x,y=%x", xyData->xPoint, xyData->yPoint);
    lv2_move_xydata(xyData, STROBE_ON_LASER_ON, STROBE_OFF_LASER_ON, LV2_BEAM_BRIGHT);
    return;
}      
static int lv2_proc_cmd(struct cmd_rw *p_cmd_data, struct lv2_info *priv)
{
    uint8_t       beam_setting;

    if (!priv)
      return(-ENODEV);

    switch(p_cmd_data->base.hdr.cmd)
      {
      case CMDW_STOP:
	beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
	beam_setting &= LASERDISABLE;      // Dark move, disable laser.
	outb(beam_setting, LG_IO_CNTRL2);
	break;
      case CMDW_LV2_SENSE_XPOINT:
	lv2_sense_one_xpoint(priv, (struct lv2_sense_one_info *)&p_cmd_data->base.cmd_data.senseData);
	break;
      case CMDW_LV2_SENSE_YPOINT:
	lv2_sense_one_ypoint(priv, (struct lv2_sense_one_info *)&p_cmd_data->base.cmd_data.senseData);
	break;
      case CMDW_LV2_COARSE_SCAN_BOX:
	return(lv2_coarse_scan_box(priv, (struct lv2_sense_info *)&p_cmd_data->base.cmd_data.senseData));
	break;
      case CMDW_LV2_MVXYDARK:
	lv2_move_xydata_dark((struct lv2_xypoints *)&p_cmd_data->base.cmd_data.xyPoints);
	break;
      case CMDW_LV2_MVXYLITE:
	lv2_move_xydata_lite((struct lv2_xypoints *)&p_cmd_data->base.cmd_data.xyPoints);
	break;
      case CMDW_LV2_FIND_SS_COORDS:
	return(lv2_find_ss_coords(priv, (struct lv2_sense_info *)&p_cmd_data->base.cmd_data.senseData));
	break;
      case CMDW_LV2_COARSE_SCAN_BOX_DARK:
	return(lv2_coarse_scan_box_dark(priv, (struct lv2_sense_info *)&p_cmd_data->base.cmd_data.senseData));
	break;
      case CMDW_LV2_SUPER_SCAN:
	return(lv2_super_scan(priv, (struct lv2_ss_sense_info *)&p_cmd_data->base.cmd_data.ss_senseData));
	break;
      default:
	printk(KERN_ERR "AV-LV2:  CMDW %d option not found\n", p_cmd_data->base.hdr.cmd);
	break;
      }
    return(0);
};
/******************************************************************
*                                                                 *
* lv2_read()                                                      *
* Description:   This function is used to process read commands.  *
*                The read buffer contains results for target      *
*                find phases.                                     *
*                                                                 *
******************************************************************/
ssize_t lv2_read(struct file *file, char __user *buffer, size_t count, loff_t *f_pos)
{
     struct lg_dev         *lg_devp = container_of(file->private_data, struct lg_dev, lgLV2);
     struct lv2_info       *priv= (struct lv2_info *)&lg_devp->lv2_data;
     //   struct lg_dev       *lg_devp=(struct lg_dev *)file->private_data;
     //    struct lv2_info     *priv;

    // The buffer is filled during write-sense commands invoked
    // by user.  Sense data from target-find register is collected 
    // for each write-sense operation.  The device is locked during
    // write-sense commands so that the integrity of this buffer
    // can be ensured.
    // Only unlock once buffer is read (or if an error is encountered).
    if (!priv)
      return(-EBADF);

    if ((count<=0) || (count > priv->sense_buff_size))
      {
 	spin_unlock(&priv->lock);
	printk(KERN_ERR "AV-LV2: READ bad count=%d to sensor buffer\n", (int)count);
	return(-EINVAL);
      }
    if (!priv->pSenseBuff)
      {
 	spin_unlock(&priv->lock);
	printk(KERN_ERR "AV-LV2: READ bad pointer to sensor buffer\n");
	return(-EBADF);
      }
    /* sensor scan has been done */
    if (copy_to_user(buffer, priv->pSenseBuff, count))
      {
	spin_unlock(&priv->lock);
	return(-EFAULT);
      }
    // All done with sensing, prep for next one.
    spin_unlock(&priv->lock);
    return(count);
}

ssize_t lv2_write(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
     struct lg_dev         *lg_devp = container_of(file->private_data, struct lg_dev, lgLV2);
     struct lv2_info       *priv= (struct lv2_info *)&lg_devp->lv2_data;
     //   struct lg_dev       *lg_devp=(struct lg_dev *)file->private_data;
     //    struct lv2_info     *priv;
    struct cmd_rw       *cmd_data;
    struct cmd_hdr      *pHdr;
    int                 rc;

    if (!priv)
      return(-EBADF);
  
    // Get command data
    cmd_data = kzalloc(sizeof(struct cmd_rw), GFP_KERNEL);
    if (!cmd_data)
      {
	pr_err("kzalloc() failed for laser\n");
	return(-ENOMEM);
      }

    if ((count <= 0)|| (count > sizeof(struct cmd_rw)))
      {
	kfree(cmd_data);
	return -EINVAL;
      }
    if(copy_from_user((char *)cmd_data, buffer, count))
      {
	kfree(cmd_data);
	return -EFAULT;
      }
    pHdr = (struct cmd_hdr *)cmd_data;
    // Validate command type
    if (!pHdr->cmd || (pHdr->cmd > CMD_LAST))
      {
	printk(KERN_ERR "AV-LV2: lg_write unknown command %d\n", pHdr->cmd);
	kfree(cmd_data);
	return(-EINVAL);
      }
    rc = lv2_proc_cmd(cmd_data, priv);
    kfree(cmd_data);
    return(count);
} 

int lv2_dev_init(struct lv2_info *lv2_data)
{
    if (!lv2_data)
      return(-EINVAL);
  
    spin_lock_init(&lv2_data->lock);
    INIT_LIST_HEAD(&lv2_data->free);
    INIT_LIST_HEAD(&lv2_data->used);
    init_waitqueue_head(&lv2_data->wait);

    // Initialize buffers to 0
    lv2_data->sense_buff_size = MAX_TF_BUFFER;
    if (!lv2_data->sense_buff_size)
      {
	printk("AV-LV2:  Bad buffer size, %d\n", lv2_data->sense_buff_size);
	return(-ENOMEM);
      }
    printk("AV-LV2: sensor buffer allocated, size = %d\n", lv2_data->sense_buff_size);
    lv2_data->pSenseBuff = kzalloc(lv2_data->sense_buff_size, GFP_KERNEL);
    if (lv2_data->pSenseBuff == NULL)
      {
	printk("AV-LV2:  Unable to malloc sensor buffer\n");
	return(-ENOMEM);
      }

    printk("AV-LV2:  lv2_dev_init() succeeded\n");
    return(0);
}
int lv2_open(struct inode *inode, struct file *file)
{
  return 0;
}
int lv2_release(struct inode *_inode, struct file *f)
{
  return 0;
}

const struct file_operations lv2_fops = {
  .owner	    = THIS_MODULE,
  .llseek           = no_llseek,
  .read             =  lv2_read,        /* lg_read */
  .write            = lv2_write,       /* lg_write */
  .open             = lv2_open,        /* lg_open */
  .release          = lv2_release,     /* lg_release */
};
MODULE_AUTHOR("Patricia A. Holden for Aligned Vision");
MODULE_DESCRIPTION("Driver for Laser Vision 2");
MODULE_LICENSE("GPL");
