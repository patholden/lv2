/*
static char rcsid[] = "$Id$";

*/
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/laser_api.h>
#include "BoardComm.h"
#include "AppCommon.h"
#include "TargetFind.h"
#include "Laser_lv2_If.h"

static int doLv2WriteCmd(struct lg_master *pLgMaster, void *pData, uint32_t data_size, uint32_t command);
static void lv_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData, uint32_t sense_cmd);

static int doLv2WriteCmd(struct lg_master *pLgMaster, void *pData, uint32_t data_size, uint32_t command)
{
    int     rc=0;
    struct  cmd_rw_base   *pCmdData;

    if (!pData || !pLgMaster)
      return(-1);

    pCmdData = (struct cmd_rw_base *)malloc(sizeof(struct cmd_rw_base));
    if (!pCmdData)
      return(-1);

    pCmdData->hdr.cmd = command;
    pCmdData->hdr.length = data_size;
    memcpy((uint8_t *)&pCmdData->cmd_data, (uint8_t *)pData, data_size);
    rc = write(pLgMaster->fd_lv2, (char *)pCmdData, sizeof(struct cmd_rw_base));
    if (rc < 0)
      syslog(LOG_ERR,"\nCMDW-MOVEDATA: ERROR cmd %d, rc %d, errno %d\n", command, rc, errno);
    free(pCmdData);
    return(rc);
}

void lv_setpoints_dark(struct lg_master *pLgMaster, struct lv2_xypoints *xyData)
{
    syslog(LOG_NOTICE,"LV_SETDARK:  x=%x,y=%x", xyData->xPoint, xyData->yPoint);
    doLv2WriteCmd(pLgMaster, (void *)xyData, sizeof(struct lv2_xypoints), CMDW_LV2_MVXYDARK);
    return;
}
void lv_setpoints_lite(struct lg_master *pLgMaster, struct lv2_xypoints *xyData)
{
    syslog(LOG_NOTICE,"LV_SETLITE:  x=%x,y=%x", xyData->xPoint, xyData->yPoint);
    doLv2WriteCmd(pLgMaster, (void *)xyData, sizeof(struct lv2_xypoints), CMDW_LV2_MVXYLITE);
    return;
}
void lv_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData, uint32_t sense_cmd)
{
    doLv2WriteCmd(pLgMaster, (void *)pSenseData, sizeof(struct lv2_sense_info), sense_cmd);
    return;
}
void lv_senseX_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData)
{
    lv_sense_cmd(pLgMaster, pSenseData, CMDW_LV2_TRAVERSEX);
    return;
}
void lv_senseY_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData)
{
    lv_sense_cmd(pLgMaster, pSenseData, CMDW_LV2_TRAVERSEY);
    return;
}
void lv_box_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData)
{
    lv_sense_cmd(pLgMaster, pSenseData, CMDW_LV2_COARSE_SCAN_BOX);
    return;
}
void lv_box_dark_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData)
{
    lv_sense_cmd(pLgMaster, pSenseData, CMDW_LV2_COARSE_SCAN_BOX_DARK);
    return;
}
void lv_find_ss_coords_sense_cmd(struct lg_master *pLgMaster, struct lv2_sense_info *pSenseData)
{
    lv_sense_cmd(pLgMaster, pSenseData, CMDW_LV2_FIND_SS_COORDS);
    return;
}
static void lv_sense_one_cmd(struct lg_master *pLgMaster, struct lv2_sense_one_info *pSenseData, uint32_t sense_cmd)
{
    doLv2WriteCmd(pLgMaster, (void *)pSenseData, sizeof(struct lv2_sense_one_info), sense_cmd);
    return;
}
void lv_sense_oneX_cmd(struct lg_master *pLgMaster, struct lv2_sense_one_info *pSenseData)
{
    lv_sense_one_cmd(pLgMaster, pSenseData, CMDW_LV2_SENSE_XPOINT);
    return;
}
void lv_sense_oneY_cmd(struct lg_master *pLgMaster, struct lv2_sense_one_info *pSenseData)
{
    lv_sense_one_cmd(pLgMaster, pSenseData, CMDW_LV2_SENSE_YPOINT);
    return;
}
void lv_super_scan_sense_cmd(struct lg_master *pLgMaster, struct lv2_ss_sense_info *pSenseData)
{
    doLv2WriteCmd(pLgMaster, (void *)pSenseData, sizeof(struct lv2_ss_sense_info), CMDW_LV2_SUPER_SCAN);
    return;
}
