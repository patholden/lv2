#ifndef CALCTRNSFM_H
#define CALCTRNSFM_H

void LogCalculateTransformCommand(struct parse_clcxfrm_parms *param, struct lg_master *pLgMaster);

void LogCalculateTransformResponse(struct parse_clcxfrm_resp *pRespBuf, uint32_t respLen);

void CalculateTransform(struct lg_master *pLgMaster,struct parse_clcxfrm_parms * pInp,
			uint32_t respondToWhom);

#endif  // CALCTRNSFM_H
