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

#define LG_VERSION 	 "0.4"
#define DEV_NAME_LV2   	 "lv2"

enum point_type {
  LV2_XPOINT=1,
  LV2_YPOINT,
  LV2_XYPOINT,
};

struct lv2_dev {
  // dev stuff first
  struct miscdevice       miscdev;
  struct mutex            lg_mutex;
  spinlock_t              lock;
  struct kref             ref;
  wait_queue_head_t       wait;
  struct list_head        free;
  struct list_head        used;
  struct completion       completion;
  struct device           *dev;
  void __iomem	          *iobase;
  struct dentry           *dbg_entry;
  // Start of actual lv2 private data
  struct timeval          last_ts;
  uint8_t                 *pSenseBuff;
  uint32_t                sense_buff_size;
  uint8_t                 pad[3];
};

static void lv2_move_xydata_dark(struct lv2_xypoints *xyData);

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
static inline void lv2_send_xy_to_dac(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t point_type, uint8_t beam_on_off)
{
    int16_t       dac_val;
    int8_t        hi_xpoint;
    int8_t        lo_xpoint;
    int8_t        hi_ypoint;
    int8_t        lo_ypoint;
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else
      beam_setting &= LASERDISABLE;  // Dark move, disable laser.

    outb(beam_setting, LG_IO_CNTRL2);
    
    if (point_type == 0)
      {
	printk("bad data point type\n");
	return;
      }
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

    // Wait x usec (user-defined)
    printk("AV-LV2: write-wait is %d usec\n", sense_delay);
    udelay(sense_delay);

    // Read from target-find register
    // Debounce to get rid of false-positive target finds
    *sense_val = inb(TFPORTRL);
    printk("AV-LV2: sense-byte is %d\n", *sense_val);
    return;
}
void do_line_sense_operation(struct lv2_sense_line_data *pSenseInfo, struct write_sense_cs_data *pSenseData, uint32_t point_axis, uint8_t step_direction)
{
    uint32_t                   pt_idx;
    int16_t                    point;
    uint16_t                   i;
    uint8_t                    sense_delay;
    uint8_t                    sense_val;

    point  = pSenseInfo->point;
    pt_idx = pSenseInfo->sense_buf_idx;
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    for (i = 0; i < pSenseInfo->numPoints; i++)
      {
	if (step_direction == LV2_ADD_STEP)
	  point += pSenseInfo->step;
	else if (step_direction == LV2_SUB_STEP)
	  point -= pSenseInfo->step;
	  
	lv2_sense_point(point, point_axis, &sense_val, pSenseInfo->sense_delay);
	pSenseData[pt_idx].sense_val = ~sense_val;
	pSenseData[pt_idx].point = point;
	printk("AV-LV2:  LINE_WS_OP sense point=%x; sense_read_data[%d]=%x\n",point, pt_idx, pSenseData[pt_idx].sense_val);
	pt_idx++;
	udelay(sense_delay);    // Wait minimum of 20 usec (minimum is set to 25) to process next x or y point.
      }
    return;
}
void lv2_senseX_add(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer
    printk("TVX-ADD: startX=%x, step=%d, count=%d\n", pSenseInfo->point, pSenseInfo->step, pSenseInfo->numPoints);
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_XPOINT, LV2_ADD_STEP);
    return;
}

void lv2_senseY_add(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer, so lock now.
    printk("TVY-ADD: startY=%x, step=%d, count=%d\n", pSenseInfo->point, pSenseInfo->step, pSenseInfo->numPoints);
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_YPOINT, LV2_ADD_STEP);
    return;
}
void lv2_senseX_sub(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer
    printk("TVX-SUB: startX=%x, step=%d, numPoints=%d\n", pSenseInfo->point, pSenseInfo->step, pSenseInfo->numPoints);
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_XPOINT, LV2_SUB_STEP);
    return;
}

void lv2_senseY_sub(struct write_sense_cs_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    // About to write to sensor buffer, so lock now.
    printk("TVY-SUB: startY=%x, step=%d, numPoints=%d\n", pSenseInfo->point, pSenseInfo->step, pSenseInfo->numPoints);
    do_line_sense_operation(pSenseInfo, pSenseWriteData, LV2_YPOINT, LV2_SUB_STEP);
    return;
}
static void lv2_coarse_scan_box_op(struct lv2_dev *priv, struct lv2_sense_info *pSenseInfo, uint8_t laser_setting)
{      
    struct lv2_sense_line_data  new_point;
    struct write_sense_cs_data *pSenseData;
    uint8_t                     beam_setting;
    uint8_t                     sense_val;

    pSenseData = (struct write_sense_cs_data *)priv->pSenseBuff;
    if (pSenseData == NULL)
      {
	printk("AV-CS-BOX:  Sensor buffer not allocated properly\n");
	return;
      }
    memset((uint8_t *)pSenseData, 0, sizeof(struct write_sense_cs_data));

    // Turn laser on or off, depending on laser-setting flag
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    if (laser_setting == LV2_LASER_ON)
      beam_setting |= LASERENABLE | BRIGHTBEAM;   // light move, enable laser.
    else
      beam_setting &= LASERDISABLE;            // dark move, disable laser
    outb(beam_setting, LG_IO_CNTRL2);
    
    // Make sure to take device lock before starting.
    spin_lock(&priv->lock);

    // These values are pushed from user & are constant
    new_point.step      = pSenseInfo->step;
    new_point.numPoints = pSenseInfo->numPoints;
    new_point.point_delay = pSenseInfo->point_delay;
    new_point.sense_buf_idx = 0;
    new_point.point_delay = SENSOR_MIN_DELAY;
    
    // Start with input X,Y
    new_point.point  = pSenseInfo->yData;
    
    // NOTE:  Leave next lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense & debounce sense-buffer-register
    // Turn on beam, dim intensity  (sensing same point causes too-bright compared to others)
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    sense_val = inb(TFPORTRL);
    sense_val = inb(TFPORTRL);
    sense_val = inb(TFPORTRL);
    
    // left side, UP (Y-ADD)
    lv2_senseY_add(pSenseData, &new_point);

    // left-to-right, across top (X-ADD)
    new_point.point = pSenseInfo->xData;
    new_point.sense_buf_idx += new_point.numPoints;
    lv2_senseX_add(pSenseData, &new_point);

    // Advance Y to top right corner of box
    // Right side, DOWN (Y-SUB)
    new_point.point = pSenseInfo->yData + (new_point.numPoints * new_point.step);
    new_point.sense_buf_idx += new_point.numPoints;
    lv2_senseY_sub(pSenseData, &new_point);

    // Advance X to top right corner of box
    // right-to-left, across bottom (X-SUB)
    new_point.point = pSenseInfo->xData + (new_point.numPoints * new_point.step);
    new_point.sense_buf_idx += new_point.numPoints;
    lv2_senseX_sub(pSenseData, &new_point);
}

int lv2_coarse_scan_box(struct lv2_dev *priv, struct lv2_sense_info *pSenseInfo)
{
    lv2_coarse_scan_box_op(priv, pSenseInfo, LV2_LASER_ON);

    // Box complete, return to caller
    return(0);
}

int lv2_coarse_scan_box_dark(struct lv2_dev *priv, struct lv2_sense_info *pSenseInfo)
{
    lv2_coarse_scan_box_op(priv, pSenseInfo, LV2_LASER_OFF);
    // Box complete, return to caller
    return(0);
}

void do_line_fssc_operation(struct lv2_sense_line_data *pSenseInfo, int16_t *endPoint, uint32_t point_axis, uint8_t step_direction)
{
    uint32_t                   pt_idx;
    uint16_t                   numPoints;
    int16_t                    point;
    uint16_t                   i;
    uint8_t                    sense_delay;
    uint8_t                    sense_val;

    point  = pSenseInfo->point;
    pt_idx = pSenseInfo->sense_buf_idx;
    if (pSenseInfo->point_delay < SENSOR_MIN_DELAY)
      sense_delay = SENSOR_MIN_DELAY;
    else
      sense_delay = pSenseInfo->point_delay;

    numPoints = LTC1597_BIPOLAR_MAX - point;
    for (i = 0; i < numPoints; i++)
      {
	if (step_direction == LV2_ADD_STEP)
	  point++;
	else if (step_direction == LV2_SUB_STEP)
	  point--;
	  
	lv2_sense_point(point, point_axis, &sense_val, pSenseInfo->sense_delay);
	if (sense_val > FSSC_THRESHOLD)
	  {
	     *endPoint = point;
	     return;
	  }
	udelay(sense_delay);    // Wait 40 usec to process next.
      }
    *endPoint = LTC1597_BIPOLAR_OFFSET_MAX;
    return;
}

void lv2_FindYEndpoint_add(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    int16_t        endPoint;
    
    printk("FSSC-Y-ADD: startY=%x\n", pSenseInfo->point);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_YPOINT, LV2_ADD_STEP);
    pSenseWriteData->TopEndpoints.yPoint = endPoint;
    return;
}

void lv2_FindYEndpoint_sub(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    int16_t        endPoint;
    
    printk("FSSC-Y-SUB: startY=%x\n", pSenseInfo->point);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_YPOINT, LV2_SUB_STEP);
    pSenseWriteData->BottomEndpoints.yPoint = endPoint;
    return;
}

void lv2_FindXEndpoint_add(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    int16_t        endPoint;
    
    printk("FSSC-X-ADD: startX=%x\n", pSenseInfo->point);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_XPOINT, LV2_ADD_STEP);
    pSenseWriteData->RightEndpoints.xPoint = endPoint;
    return;
}

void lv2_FindXEndpoint_sub(struct write_sense_fssc_data *pSenseWriteData, struct lv2_sense_line_data *pSenseInfo)
{
    int16_t        endPoint;
    
    printk("FSSC-X-SUB: startX=%x\n", pSenseInfo->point);
    do_line_fssc_operation(pSenseInfo, (int16_t *)&endPoint, LV2_XPOINT, LV2_SUB_STEP);
    pSenseWriteData->LeftEndpoints.xPoint = endPoint;
    return;
}

int lv2_find_ss_coords(struct lv2_dev *priv, struct lv2_sense_info *pSenseInfo)
{
    struct write_sense_fssc_data   *pSenseData;
    struct lv2_sense_line_data     new_point;
    struct lv2_xypoints            xyData;
    uint8_t                        sense_val;
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
    
    // Start with input X,Y
    new_point.point  = pSenseInfo->yData;
    
    // NOTE:  Leave next lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense for first 2 points
    // Turn on beam, dim intensity  (sensing same point causes too-bright compared to others)
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= LASERENABLE | DIMBEAM;   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);

    // Turn on beam, bright intensity
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= LASERENABLE | BRIGHTBEAM;   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);

    // Go UP (Y-ADD), search for top endpoint
    lv2_FindYEndpoint_add(pSenseData, &new_point);
    pSenseData->TopEndpoints.xPoint = pSenseInfo->xData;
    
    // Go Down (Y-SUB), search for bottom endpoint
    new_point.point  = pSenseInfo->yData;
    lv2_FindYEndpoint_sub(pSenseData, &new_point);
    pSenseData->BottomEndpoints.xPoint = pSenseInfo->xData;

    xyData.xPoint  = pSenseInfo->xData;
    xyData.yPoint = (pSenseData->TopEndpoints.yPoint + pSenseData->BottomEndpoints.yPoint) / 2;
    lv2_move_xydata_dark((struct lv2_xypoints *)&xyData);

    // Go right (X-ADD), search for right-side endpoint
    new_point.point  = pSenseInfo->xData;
    lv2_FindXEndpoint_add(pSenseData, &new_point);
    pSenseData->RightEndpoints.yPoint = xyData.yPoint;
    
    // Go left (X-SUB), search for bottom endpoint
    new_point.point  = pSenseInfo->xData;
    lv2_FindXEndpoint_sub(pSenseData, &new_point);
    pSenseData->LeftEndpoints.yPoint = xyData.yPoint;
    return(0);
}

int lv2_super_scan(struct lv2_dev *priv, struct lv2_ss_sense_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData;
    struct lv2_sense_line_data     new_point;
    struct lv2_xypoints            xyData;
    uint32_t                       i;
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
        
    // These values are pushed from user & are constant
    new_point.step        = pSenseInfo->step;
    new_point.numPoints   = pSenseInfo->numPoints;
    new_point.point_delay = pSenseInfo->point_delay;
    new_point.sense_buf_idx = 0;
    new_point.point_delay = SENSOR_MIN_DELAY;
    
    // Start with input X,Y
    new_point.point  = pSenseInfo->yData;
    
    // NOTE:  Leave next lines as-is.
    // Sometimes there's a spike when beam is turned on,
    // so do per-point write-sense for first 2 points
    // Turn on beam, dim intensity  (sensing same point causes too-bright compared to others)
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= LASERENABLE | DIMBEAM;   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    lv2_sense_point(new_point.point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);

    // Turn on beam, bright intensity
    beam_setting = inb(LG_IO_CNTRL2);           // Get initial val of LG_IO_CNTRL2 register
    beam_setting |= LASERENABLE | BRIGHTBEAM;   // light move, enable laser.
    outb(beam_setting, LG_IO_CNTRL2);

    for (i = 0; i < pSenseInfo->numLines; i++)
      {
	// Go right (X-ADD)
	// Move xy in dark to position
	xyData.xPoint = pSenseInfo->xData;
	xyData.yPoint = pSenseInfo->yData + (i * pSenseInfo->numLines);
	lv2_move_xydata_dark((struct lv2_xypoints *)&xyData);

	// Sense left across line
	new_point.point  = pSenseInfo->xData;
	lv2_senseX_add(pSenseData, &new_point);
	new_point.sense_buf_idx += pSenseInfo->numPoints;
	
	// Go left (X-SUB)
	// Move xy in dark to position
	xyData.xPoint = pSenseInfo->xData + (new_point.numPoints * new_point.step);
	xyData.yPoint = pSenseInfo->yData + (i * pSenseInfo->numLines);
	lv2_move_xydata_dark((struct lv2_xypoints *)&xyData);

	// Sense right across line
	new_point.point = pSenseInfo->xData + (new_point.numPoints * new_point.step);
	lv2_senseX_sub(pSenseData, &new_point);
	new_point.sense_buf_idx += pSenseInfo->numPoints;
      }
    return(0);
}

void lv2_sense_one_ypoint(struct lv2_dev *priv, struct lv2_sense_one_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData=(struct write_sense_cs_data *)priv->pSenseBuff;
    int16_t                    point;
    uint8_t                    sense_val;

    if (pSenseData == NULL)
      return;
    
    // About to write to sensor buffer, so lock now.
    point = pSenseInfo->data;
    lv2_sense_point(point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    pSenseData[pSenseInfo->index].sense_val = ~sense_val;
    pSenseData[pSenseInfo->index].point = point;
    printk("Sense yPoint[%d]=%x; sense_read_data=%x\n",pSenseInfo->index, point, pSenseData[pSenseInfo->index].sense_val);
    return;
}
void lv2_sense_one_xpoint(struct lv2_dev *priv, struct lv2_sense_one_info *pSenseInfo)
{
    struct write_sense_cs_data    *pSenseData=(struct write_sense_cs_data *)priv->pSenseBuff;
    int16_t    point;
    uint8_t    sense_val;
    
    if (pSenseData == NULL)
      return;
    
    // About to write to sensor buffer, so lock now.
    spin_lock(&priv->lock);
    point = pSenseInfo->data;
    lv2_sense_point(point, LV2_YPOINT, &sense_val, pSenseInfo->sense_delay);
    pSenseData[pSenseInfo->index].sense_val = ~sense_val;
    pSenseData[pSenseInfo->index].point = point;
    printk("Sense xPoint[%d]=%x; rtn_data=%x; buf_data=%x\n",pSenseInfo->index, point, sense_val, pSenseData[pSenseInfo->index].sense_val);
    return;
}
static void lv2_move_xdata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else
      beam_setting &= LASERDISABLE;            // dark move, disable laser
    outb(beam_setting, LG_IO_CNTRL2);  // Apply CNTRL2 setting
    lv2_send_to_dac(xyData->xPoint, strobe_on, strobe_off, LV2_XPOINT);
    return;
}
static void lv2_move_ydata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    uint8_t       beam_setting;

    beam_setting = inb(LG_IO_CNTRL2);  // Turn off beam
    if (beam_on_off)
      beam_setting |= LASERENABLE | BRIGHTBEAM;  // light move, enable laser.
    else
      beam_setting &= LASERDISABLE;    // dark move, disable laser.
    outb(beam_setting, LG_IO_CNTRL2);  // Apply CNTRL2 settings
    lv2_send_to_dac(xyData->yPoint, strobe_on, strobe_off, LV2_YPOINT);
    return;
}
static void lv2_move_xydata(struct lv2_xypoints *xyData, uint8_t strobe_on, uint8_t strobe_off, uint8_t beam_on_off)
{
    lv2_move_xdata(xyData, strobe_on, strobe_off, beam_on_off);
    lv2_move_ydata(xyData, strobe_on, strobe_off, beam_on_off);
    return;
}
static void lv2_move_xydata_dark(struct lv2_xypoints *xyData)
{
    lv2_move_xydata(xyData, STROBE_ON_LASER_OFF, STROBE_OFF_LASER_OFF, 0);
    return;
}
static void lv2_move_xydata_lite(struct lv2_xypoints *xyData)
{
  lv2_move_xydata(xyData, STROBE_ON_LASER_ON, STROBE_OFF_LASER_ON, 1);
    return;
}      
static int lv2_proc_cmd(struct cmd_rw *p_cmd_data, struct lv2_dev *priv)
{
    if (!priv)
      return(-ENODEV);

    switch(p_cmd_data->base.hdr.cmd)
      {
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
	printk(KERN_ERR "\nAV-LV2:  CMDW %d option not found", p_cmd_data->base.hdr.cmd);
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
    struct lv2_dev *priv = file->private_data;

    // The buffer is filled during write-sense commands invoked
    // by user.  Sense data from target-find register is collected 
    // for each write-sense operation.  The device is locked during
    // write-sense commands so that the integrity of this buffer
    // can be ensured.
    // Only unlock once buffer is read (or if an error is encountered).
    if (!priv)
      {
	spin_unlock(&priv->lock);
	return(-EBADF);
      }    
    if ((count<=0) || (count > priv->sense_buff_size))
      {
 	spin_unlock(&priv->lock);
	printk(KERN_ERR "\nAV-LV2: READ bad count=%d to sensor buffer", (int)count);
	return(-EINVAL);
      }
    if (!priv->pSenseBuff)
      {
 	spin_unlock(&priv->lock);
	printk(KERN_ERR "\nAV-LV2: READ bad pointer to sensor buffer");
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
    struct lv2_dev           *priv;
    struct cmd_rw            *cmd_data;
    struct cmd_hdr           *pHdr;
    int                       rc;

    priv = (struct lv2_dev *)file->private_data;
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
	printk(KERN_ERR "\nAV-LV2: lg_write unknown command %d", pHdr->cmd);
	kfree(cmd_data);
	return(-EINVAL);
      }
    rc = lv2_proc_cmd(cmd_data, priv);
    kfree(cmd_data);
    return(count);
} 

static int lv2_dev_init(struct lv2_dev *lv2_devp)
{
    struct   lv2_xypoints   xyData;

    if (!lv2_devp)
      return(-EINVAL);
  
    do_gettimeofday(&lv2_devp->last_ts);

    // Initialize buffers to 0
    lv2_devp->sense_buff_size = MAX_TF_BUFFER;
    if (!lv2_devp->sense_buff_size)
      {
	printk("AV-LV2:  Bad buffer size, %d\n", lv2_devp->sense_buff_size);
	return(-ENOMEM);
      }
    printk("AV-LV2: sensor buffer allocated, size = %d", lv2_devp->sense_buff_size);
    lv2_devp->pSenseBuff = kzalloc(lv2_devp->sense_buff_size, GFP_KERNEL);
    if (lv2_devp->pSenseBuff == NULL)
      {
	printk("AV-LV2:  Unable to malloc sensor buffer\n");
	return(-ENOMEM);
      }

    /* move to 0,0 position, laser disabled, beam off */
    memset((uint8_t *)&xyData, 0, sizeof(struct lv2_xypoints));
    lv2_move_xydata_dark((struct lv2_xypoints *)&xyData);
      
    // Start with READY-LED ON TO INDICATE NOT READY TO RUN.
    outb(RDYLEDON, LG_IO_CNTRL2);
    printk("AV-LV2:  lv2_dev_init() succeeded\n");
    return(0);
}
static int lv2_open(struct inode *inode, struct file *file)
{
  return 0;
}
int lv2_release(struct inode *_inode, struct file *f)
{
  return 0;
}

static const struct file_operations lv2_fops = {
  .owner	    = THIS_MODULE,
  .llseek           = no_llseek,
  .read             =  lv2_read,        /* lg_read */
  .write            = lv2_write,       /* lg_write */
  .open             = lv2_open,        /* lg_open */
  .release          = lv2_release,     /* lg_release */
};
struct miscdevice lv2_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEV_NAME_LV2,
  .fops = &lv2_fops,
};

static int lv2_dev_probe(struct platform_device *plat_dev)
{
    struct lv2_dev *lv2_dev;
    int            rc;

    // allocate mem for struct device will work with
    lv2_dev = kzalloc(sizeof(struct lv2_dev), GFP_KERNEL);
    if (!lv2_dev)
      {
	pr_err("kzalloc() failed for laser device struct\n");
	return(-ENOMEM);
      }

    memset((char *)lv2_dev,0,sizeof(struct lv2_dev));
    platform_set_drvdata(plat_dev, lv2_dev);
    lv2_dev->dev = &plat_dev->dev;

    // We don't use kref or mutex locks yet.
    kref_init(&lv2_dev->ref);
    mutex_init(&lv2_dev->lg_mutex);

    dev_set_drvdata(lv2_dev->dev, lv2_dev);
    spin_lock_init(&lv2_dev->lock);
    INIT_LIST_HEAD(&lv2_dev->free);
    INIT_LIST_HEAD(&lv2_dev->used);
    init_waitqueue_head(&lv2_dev->wait);

    // Setup misc device
    lv2_dev->miscdev.minor = lv2_device.minor;
    lv2_dev->miscdev.name = DEV_NAME_LV2;
    lv2_dev->miscdev.fops = lv2_device.fops;
    rc = misc_register(&lv2_dev->miscdev);
    if (rc)
      {
	printk(KERN_ERR "AV-LV2:  Failed to register Laser misc_device, err %d \n", rc);
	kfree(lv2_dev);
	return(rc);
      }

    printk(KERN_INFO "\nAV-LV2:laser misc-device created\n");

    // Obtain IO space for device
    if (!request_region(LG_BASE, LASER_REGION, DEV_NAME_LV2))
      {
	kfree(lv2_dev);
	misc_deregister(&lv2_dev->miscdev);
	printk(KERN_CRIT "\nAV-LV2:  Unable to get IO regs");
	return(-EBUSY);
      }

    rc = lv2_dev_init(lv2_dev);
    if (rc)
      {
	printk(KERN_ERR "AV-LV2:  Failed to initialize Laser device, err %d \n", rc);
	kfree(lv2_dev);
	return(rc);
      }
      
    // All initialization done, so enable timer
    printk(KERN_INFO "\nAV-LV2: LV2 Device installed and initialized.\n");
    return(0);
}
static int lv2_pdev_remove(struct platform_device *pdev)
{
  struct lv2_dev *lv2_dev = platform_get_drvdata(pdev);
  struct device *this_device;
  
  if (!lv2_dev)
    return(-EBADF);

  kfree(lv2_dev->pSenseBuff);
  this_device = lv2_dev->miscdev.this_device;
  release_region(LG_BASE, LASER_REGION);
  misc_deregister(&lv2_dev->miscdev);
  kfree(lv2_dev);
  return(0);
}

static struct platform_device lv2_dev = {
  .name   = DEV_NAME_LV2,
  .id     = 0,
  .num_resources = 0,
};
static struct platform_driver lv2_platform_driver = {
  .probe   = lv2_dev_probe,
  .remove  = lv2_pdev_remove,
  .driver  = {
    .name  = DEV_NAME_LV2,
  },
};
static int __init lv2_init(void)
{
  int rc;

  rc = platform_driver_register(&lv2_platform_driver);
  if (rc)
    {
      printk(KERN_ERR "AV-LV2:  Unable to register platform driver lv2, ret %d\n", rc);
      return(rc);
    }
  rc = platform_device_register(&lv2_dev);
  if (rc)
    {
      printk(KERN_ERR "AV-LV2:  Unable to register platform device lv2, ret %d\n", rc);
      return(rc);
    }
  printk(KERN_INFO "\nAV-LV2:LaserVision2 platform device driver installed.\n");
  return(rc);
}

static void __exit lv2_exit(void)
{
  platform_device_unregister(&lv2_dev);
  platform_driver_unregister(&lv2_platform_driver);
  return;
}
module_init(lv2_init);
module_exit(lv2_exit);

MODULE_AUTHOR("Patricia A. Holden for Aligned Vision");
MODULE_DESCRIPTION("Driver for Laser Vision 2");
MODULE_LICENSE("GPL");
