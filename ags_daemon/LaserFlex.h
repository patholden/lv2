#ifndef LASERFLEX_H
#define LASERFLEX_H
void DoThresholdQuickCheck(struct lg_master *pLgMaster, struct parse_thqkchk_parms *pInp, uint32_t respondToWhom);
void DoFlexQuickCheck(struct lg_master *pLgMaster, struct parse_flexqkchk_parms* data, uint32_t respondToWhom);
void DoFlexDisplayChunks(struct lg_master *pLgMaster,
			 struct parse_chunkflex_parms *pInp,
			 uint32_t respondToWhom);
void DoFlexDisplay (struct lg_master *pLgMaster, uint32_t dataLength,
		    struct parse_flexdisp_parms * pInp, char * patternData);
#endif // LASERFLEX_H

