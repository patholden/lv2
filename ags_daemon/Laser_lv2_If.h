#ifndef _LASER_IF_H
#define _LASER_IF_H

void lv_setpoints_lite(struct lg_master *pLgMaster, struct lv2_xypoints *xyData);
void lv_setpoints_dark(struct lg_master *pLgMaster, struct lv2_xypoints *xyData);
void lv_senseX_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData);
void lv_senseY_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData);
void lv_sense_oneX_cmd(struct lg_master *pLgMaster, struct lv2_sense_one_info *pSenseData);
void lv_sense_oneY_cmd(struct lg_master *pLgMaster, struct lv2_sense_one_info *pSenseData);
void lv_box_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData);
void lv_find_ss_coords_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData);


#endif
