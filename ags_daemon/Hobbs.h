#ifndef HOBBS_H
#define HOBBS_H

#define HOBBS_MAX_BUFFLEN  4096

void StartHobbs(struct lg_master *pLgMaster);
void EndHobbs(struct lg_master *pLgMaster);
int WriteHobbs(struct lg_master *pLgMaster);
void DoHobbsSet(struct lg_master *pLgMaster, struct parse_hobbsset_parms *pInp, uint32_t respondToWhom);
void DoHobbsGet(struct lg_master *pLgMaster, struct parse_hobbsget_parms *pInp, uint32_t respondToWhom);
int HobbsCountersInit(struct lg_master *pLgMaster);
#endif
