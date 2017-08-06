/*
 * $Id: RightOnCert.h,v 1.1 1999/11/22 01:58:11 ags-sw Exp $
 */
#ifndef RTONCRT_H
#define RTONCRT_H

#ifdef AGS_DEBUG
void LogRightOnCertCommand(struct parse_rtoncert_parms *param, struct lg_master *pLgMaster);

void LogRightOnCertResponse(struct parse_rtoncert_resp *pRespBuf, uint32_t respLen);
#endif

void RightOnCert(struct lg_master *pLgMaster,
		 struct parse_rtoncert_parms *param,
		 uint32_t respondToWhom);

#endif  // RTONCRT_H
