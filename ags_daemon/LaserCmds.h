/*   $Id: LaserCmds.h,v 1.8 2001/01/03 18:01:36 ags-sw Exp pickle $  */
#ifndef LASERCMDS_H
#define LASERCMDS_H

void ResetPlyCounter(struct lg_master *pLgMaster);
void DoStopCmd( struct lg_master *pLgMaster, uint32_t respondToWhom );
int  IfStopThenStopAndNeg1Else0 (struct lg_master *pLgMaster);
void DoGoAngle(struct lg_master *pLgMaster,struct parse_goangle_parms *pInp, uint32_t respondToWhom);
void DoEtherAngle(struct lg_master *pLgMaster, struct parse_ethangle_parms *pInp, uint32_t respondToWhom);
void DarkAngle(struct lg_master *pLgMaster, struct parse_dkangle_parms *pInp, uint32_t respondToWhom);
void DoSearchForASensor(double x, double y, uint32_t respondToWhom);
void DoDisplay(struct lg_master *pLgMaster, uint32_t dataLength, char *otherParameters, char *patternData);
void DoNewDisplay(uint32_t dataLength, char *otherParameters, char *patternData,
		  uint32_t respondToWhom);
void DoDisplayNoQuickCheck(uint32_t dataLength, char *transformation,
			   char *patternData, uint32_t respondToWhom);
void DoDisplayChunksStart(struct lg_master *pLgMaster,
			  struct parse_chunkstart_parms *pInp,
			  uint32_t respondToWhom );
void DoDisplayChunks(struct lg_master *pLgMaster,
		     struct parse_chunksdo_parms *pInp,
		     uint32_t respondToWhom );

void DoDisplayKitVideo (struct lg_master *pLgMaster, uint32_t dataLength,
			unsigned char * otherParameters, char * patternData,
			uint32_t respondToWhom);
void AddDisplayChunksData(struct lg_master *pLgMaster, uint32_t dataLength,
			  uint32_t dataOffset, char *patternData, uint32_t respondToWhom);

void DoQuickCheck(struct lg_master *pLgMaster, struct parse_qkcheck_parms *angles, uint32_t respondToWhom);
void SetDisplaySeveral(struct lg_master *pLgMaster, uint32_t number,
		       uint32_t respondToWhom);
void DoDisplayVideoCheck(uint32_t dataLength, char *otherParameters,
                        char *patternData, uint32_t respondToWhom);
void DoSegmentDisplay(double x, double y, uint32_t respondToWhom);
void DoDisplayChunksRaw(int32_t respondToWhom);
void DimAngle (struct lg_master *pLgMaster, struct parse_dimangle_parms *pInp, uint32_t respondToWhom);
#endif
