/*
 * $Id: RightOnCert.h,v 1.1 1999/11/22 01:58:11 ags-sw Exp $
 */
#ifndef FRACCRT_H
#define FRACCRT_H

#ifdef AGS_DEBUG
void LogFracCertCommand(struct parse_fraccert_parms *param, struct lg_master *pLgMaster);

void LogFracCertResponse(struct parse_fraccert_resp *pRespBuf, uint32_t respLen);
#endif

extern
void FracCert(struct lg_master *pLgMaster,
		 struct parse_fraccert_parms *param,
		 uint32_t respondToWhom);

#endif  // RTONCRT_H
