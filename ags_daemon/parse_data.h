#include <stdint.h>
#ifndef PARSE_DATA_H
#define PARSE_DATA_H

int parse_data(struct lg_master *pLgMaster, unsigned char *data, uint32_t data_len, uint32_t *rawindex);
// Command input structures
// NOTE---first 4 bytes for input parms are stripped off by parse_data()
//        before calling a function to perform command/request.  CRC is
//        checked when parse_data() is first called, can be ignored by
//        specific functions.  Therefore, all input structs exclude the
//        k_header and CRC.
#define MAX_DATA         4096
#define MAX_INPUT_DATA   8192
#define MAX_NEW_TRANSFORM_ITEMS  12
#define MAX_TGTS_PER_PROJ_OLD    18   // kNumberOfRegPoints * 3
#define OLDTRANSFORMLEN  144   // 12 * kSizeOldLongDouble
#define NEWTRANSFORMLEN  MAX_NEW_TRANSFORM_ITEMS * (sizeof(double))   // 12 * 8
#define OLDANGLEPAIRSLEN 48     // kNumberOfFlexPoints * 2 * sizeof(uint32_t)
#define ANGLEPAIRSLENFLEX  192  // kNumberOfFlexPoints * 2 * sizeof(uint32_t)
#define NUMDISPFLEXPOINTS 576      // kNumberOfFlexPoints * 3 * sizeof ( double )
#define RESPFLEXFAIL 0xE7  // aka kFlexFail
#define RESPFAIL 0xE8      // aka kFail
#define RESPFLEXFAIL 0xE7      // aka kFlexFail
#define RESPGOOD 0xE0      //  aka kOK
#define RESPSTOPOK 0xE2    // aka kStopOK
#define RESPQCPLYFAIL       0xEB    // AKA kQuickCheckPlyFail
#define RESPDATATOOLARGE  0x6
#define RESPTOOMANYPLIES  0x8
#define RESPOTHERERROR    0xFFFF
#define RESPE1APTERROR    0x4
#define RESPE1BOARDERROR  0x5
#define RESPE1INANGLEOUTOFRANGE 0x2
#define RESPE1PATANGLEOUTOFRANGE 0x3
#define BUFF_SIZE         8192     // refer to AGS_SIZE in Files.h 
#define AUTOFOCUS_SIZE    32
#define LCNAME_SIZE       32
#define MAX_TARGETSOLD 6
#define MAX_TGTANGLEOLD 6
#define QCPLYCOUNTMASK      0xFFFFFF
#define QCPLYCOUNT23MASK    0xFFFF

struct k_targetinfo
{
  uint32_t  tgt_number;
  double    xdev;
  double    ydev;
} __attribute__ ((packed));

struct k_xy_anglepair
{
  int32_t   xangle;
  int32_t   yangle;
} __attribute__ ((packed));

struct k_xyz_double
{
  double   Xtgt;
  double   Ytgt;
  double   Ztgt;
} __attribute__ ((packed));

struct k_xy_double
{
  double   Xangle;
  double   Yangle;
} __attribute__ ((packed));

struct show_anglepair
{
  double   Xangle;
  double   Yangle;
  uint32_t flag;
} __attribute__ ((packed));

/*********************************************************/
/*                                                       */
/*   PARM INPUT Structures                               */
/*   NOTE:  MUST EXCLUDE kheader & CRC                   */
/*                                                       */
/*********************************************************/
struct parse_dispsev_parms {
  uint32_t         num_seq;
} __attribute__ ((packed));

struct parse_hobbsget_parms {
  uint32_t         inp_hobbsid;
} __attribute__ ((packed));

struct parse_hobbsset_parms {
  uint32_t         inp_hobbsid;
  uint32_t         inp_hobbsval;
} __attribute__ ((packed));

struct parse_chunkstart_parms {
  uint32_t         apt_len;
} __attribute__ ((packed));

struct parse_chunkdata_parms {
  uint32_t        chunk_len;
  uint32_t        chunk_offset;
  unsigned char   chunk_apt_data[BUFF_SIZE];
} __attribute__ ((packed));

struct parse_chunksdo_parms {
  unsigned char   chunk_anglepairs[OLDANGLEPAIRSLEN];
  unsigned char   chunk_transform[OLDTRANSFORMLEN];
} __attribute__ ((packed));

struct parse_getdata_parms {
  char       inp_filename[LCNAME_SIZE];
  uint32_t   inp_offset;
  uint32_t   inp_numbytes;
} __attribute__ ((packed));

struct parse_getstart_parms {
  char       inp_filename[LCNAME_SIZE];
} __attribute__ ((packed));

struct parse_putstart_parms {
  char       inp_filename[LCNAME_SIZE];
  uint32_t   inp_filelen;
} __attribute__ ((packed));

struct parse_putdata_parms {
  char        inp_filename[LCNAME_SIZE];
  uint32_t    inp_offset;
  uint32_t    inp_buf_numbytes;
  uint32_t    inp_write_numbytes;
  char        inp_buffer[BUFF_SIZE];
} __attribute__ ((packed));

struct parse_putdone_parms {
  char        inp_filename[LCNAME_SIZE];
} __attribute__ ((packed));

struct parse_autofocus_parms {
  char        inp_data[AUTOFOCUS_SIZE];
} __attribute__ ((packed));


struct parse_rtoncert_parms {
  uint32_t         inp_angle_flag;
  unsigned char    inp_transform[OLDTRANSFORMLEN];
  double           inp_Xin;
  double           inp_Yin;
  double           inp_Zin;
  int32_t          inp_XrawAngle;
  int32_t          inp_YrawAngle;
  double           inp_XgeomAngle;
  double           inp_YgeomAngle;
} __attribute__ ((packed));

struct parse_fraccert_parms {
  uint32_t         inp_angle_flag;
  unsigned char    inp_transform[OLDTRANSFORMLEN];
  double           inp_Xin;
  double           inp_Yin;
  double           inp_Zin;
  double           inp_XrawAngle;
  double           inp_YrawAngle;
  double           inp_XgeomAngle;
  double           inp_YgeomAngle;
} __attribute__ ((packed));

struct parse_chunkflex_parms
{
  uint32_t         inp_numTargets;
  unsigned char    inp_anglepairs[ANGLEPAIRSLENFLEX];
  unsigned char    inp_transform[OLDTRANSFORMLEN];
} __attribute__ ((packed));
struct parse_flexqkchk_parms
{
  uint32_t         inp_numTargets;
  unsigned char    inp_anglepairs[ANGLEPAIRSLENFLEX];
} __attribute__ ((packed));

struct parse_flexdisp_parms
{
  uint32_t      inp_dataLength;
  uint32_t      inp_numTargets;
  unsigned char inp_anglepairs[NUMDISPFLEXPOINTS];
  unsigned char inp_transform[OLDTRANSFORMLEN];
} __attribute__ ((packed));

struct parse_disp_parms
{
  uint32_t      inp_dataLength;
  unsigned char inp_dispdata[MAX_DATA];
} __attribute__ ((packed));

struct parse_quickcheckcount_parms
{
  uint32_t      inp_qccount;
} __attribute__ ((packed));

struct parse_quickchecktime_parms
{
  uint32_t      inp_qctime;
} __attribute__ ((packed));

struct parse_dispkitvid_parms {
  uint32_t         inp_aptlen;
  unsigned char    inp_transform[OLDTRANSFORMLEN];
  uint32_t         inp_vid_predwell;
  uint32_t         inp_vid_count;
  double           inp_vidX;
  double           inp_vidY;
  double           inp_vidZ;
  unsigned char    inp_vidB1;
  unsigned char    inp_vidB2;
  unsigned char    inp_vid_aptdata[BUFF_SIZE];
} __attribute__ ((packed));

struct parse_findonetgt_parms {
  double            steerX;
  double            steerY;
} __attribute__ ((packed));

struct parse_takepic_parms {
  unsigned char     transform[NEWTRANSFORMLEN];
  double            visionX;
  double            visionY;
  double            visionZ;
} __attribute__ ((packed));

struct parse_clcxfrm_parms {
  struct k_xyz_double target[MAX_TARGETSOLD];
  struct k_xy_double  tgtAngle[MAX_TARGETSOLD];
} __attribute__ ((packed));

struct parse_flexcalxfrm_parms {
  uint32_t            num_targets;
  struct k_xyz_double target[MAX_TARGETSFLEX];
  struct k_xy_double  tgtAngle[MAX_TARGETSFLEX];
} __attribute__ ((packed));

struct parse_flexcalxfdbk_parms
{
  uint32_t            num_targets;
  struct k_xyz_double target[MAX_TARGETSFLEX];
  struct k_xy_double  tgtAngle[MAX_TARGETSFLEX];
} __attribute__ ((packed));

struct parse_rightondofullreg_parms {
  uint32_t              angleflag;
  struct k_xyz_double   target[MAX_TARGETSOLD];
  struct k_xy_double    target_geo_angle[MAX_TARGETSOLD];
  struct k_xy_anglepair target_raw_angle[MAX_TARGETSOLD];
} __attribute__ ((packed));

struct parse_rightondofullregwithfeedback_parms {
  uint32_t              angleflag;
  uint32_t              num_targets;
  struct k_xyz_double   target[MAX_TARGETSOLD + 2];
  struct k_xy_double    target_geo_angle[MAX_TARGETSOLD + 2];
  struct k_xy_anglepair target_raw_angle[MAX_TARGETSOLD + 2];
} __attribute__ ((packed));

struct parse_flexrightondofullregwithfeedback_parms {
  uint32_t              angleflag;
  uint32_t              num_targets;
  struct k_xyz_double   target[MAX_TARGETSFLEX];
  struct k_xy_double    target_geo_angle[MAX_TARGETSFLEX];
  struct k_xy_anglepair target_raw_angle[MAX_TARGETSFLEX];
} __attribute__ ((packed));

struct parse_chngdisp_parms {
  uint32_t    displayPeriod;
} __attribute__ ((packed));

struct parse_ethangle_parms {
  union {
    double   xData;
    uint8_t  xchar[8];
  }x;
  union {
    double   yData;
    uint8_t  ychar[8];
  }y;
} __attribute__ ((packed));

struct parse_goangle_parms {
  union {
    double   xData;
    int8_t   xchar[8];
  }x;
  union {
    double   yData;
    int8_t   ychar[8];
  }y;
} __attribute__ ((packed));

struct parse_setbit_parms {
  uint32_t  bit_id;
  uint32_t  bit_value;
};

struct parse_showtgt_parms
{
  uint32_t         inp_numpairs;
  struct show_anglepair inp_targetpairs[MAX_TARGETSFLEX];
};

// Send Confirm
struct parse_dkangle_parms {
  union {
    double   xData;
    uint8_t xchar[8];
  }x;
  union {
    double   yData;
    uint8_t  ychar[8];
  }y;
} __attribute__ ((packed));

struct parse_dimangle_parms {
  double   xData;
  double   yData;
  uint32_t pulseoff;
  uint32_t pulseon;
} __attribute__ ((packed));

struct parse_chngxfrmtol_parms
{
  double         new_tolerance;
} __attribute__ ((packed));

struct parse_calibxy_parms
{
  double         steerX;
  double         steerY;
  double         inp_Z;
} __attribute__ ((packed));
struct parse_qkcheck_parms
{
  unsigned char   anglepairs[OLDANGLEPAIRSLEN];
} __attribute__ ((packed));
struct parse_thqkchk_parms
{
  uint32_t        threshold;
  uint32_t        num_targets;
  unsigned char   anglepairs[ANGLEPAIRSLENFLEX];
} __attribute__ ((packed));

/*********************************************************/
/*                                                       */
/*              CONFIRMATION structure                   */
/*                                                       */
/*********************************************************/

struct send_cnfm {
  unsigned char   cmd;
  unsigned char   flags;
  uint16_t        seq_num;
  uint16_t        crc;
} __attribute__ ((packed));

/*********************************************************/
/*                                                       */
/*   Response output structures                          */
/*   NOTE:  MUST include kheader & CRC                   */
/*                                                       */
/*********************************************************/

struct parse_basic_resp {
  struct   k_header  hdr;
  uint16_t   resp_crc;
} __attribute__ ((packed));

struct parse_getstart_resp {
  struct   k_header  hdr;
  uint32_t   resp_filelen;
  uint16_t   resp_crc;
} __attribute__ ((packed));

struct parse_getdata_resp {
  struct k_header  hdr;
  char             lcName[LCNAME_SIZE];
  uint32_t         resp_offset;
  uint32_t         resp_numbytes;
  char             resp_buffer[BUFF_SIZE];
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_putstart_resp {
  struct k_header  hdr;
  unsigned char    filename[LCNAME_SIZE];
  uint32_t         resp_filelen;
  uint16_t   resp_crc;
} __attribute__ ((packed));

struct parse_putdata_resp {
  struct k_header  hdr;
  uint16_t   resp_crc;
} __attribute__ ((packed));

struct parse_putdone_resp {
  struct k_header  hdr;
  unsigned char    filename[LCNAME_SIZE];
  uint32_t         filelen;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_autofocus_resp {
  struct k_header  hdr;
  char       resp_data[AUTOFOCUS_SIZE];
  uint16_t   resp_crc;
} __attribute__ ((packed));

struct parse_rtoncert_resp {
  struct k_header  hdr;
  double           Xtr;
  double           Ytr;
  double           Ztr;
  double           Xfound;
  double           Yfound;
  double           Xexpect;
  double           Yexpect;
  uint32_t         bXf;
  uint32_t         bYf;
  double           Xexternal;
  double           Yexternal;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_fraccert_resp {
  struct k_header  hdr;
  double           Xtr;
  double           Ytr;
  double           Ztr;
  double           Xfound;
  double           Yfound;
  double           Xexpect;
  double           Yexpect;
  double           dbXf;
  double           dbYf;
  double           Xexternal;
  double           Yexternal;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_hobbsget_resp {
  struct k_header  hdr;
  uint32_t         resp_hobbscount;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_findonetgt_resp {
  struct k_header   hdr;
  uint32_t          rawX;
  uint32_t          rawY;
  double            geoX;
  double            geoY;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_tgtsused_resp {
  struct k_header  hdr;
  uint32_t         tgtCount;
  unsigned char    tgtNumber[MAX_TARGETSUSED];
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_clcxfrm_resp {
  struct k_header  hdr;
  unsigned char    transform[OLDTRANSFORMLEN];
  unsigned char    anglepairs[OLDANGLEPAIRSLEN];
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_flexcalxfrm_resp {
  struct k_header  hdr;
  unsigned char    transform[OLDTRANSFORMLEN];
  unsigned char    anglepairs[ANGLEPAIRSLENFLEX];
  uint32_t         num_xfrms;
  uint32_t         num_tgts;
  uint32_t         colineartgt;
  uint32_t         coplanartgts;
  uint16_t         resp_crc;
} __attribute__ ((packed));
 struct parse_getdacs_resp {
   struct k_header  hdr;
   uint32_t     x_dac;
   uint32_t     y_dac;
   uint16_t     resp_crc;
} __attribute__ ((packed));

struct parse_flexcalxfdbk_resp {
  struct k_header  hdr;
  unsigned char    transform[OLDTRANSFORMLEN];
  unsigned char    anglepairs[ANGLEPAIRSLENFLEX];
  uint32_t         num_xfrms;
  uint32_t         num_tgts;
  uint32_t         colineartgt;
  uint32_t         coplanartgts;
  uint32_t         tgt_status[MAX_TARGETSFLEX];
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_flexcalxfdbkexp_resp {
  struct k_header  hdr;
  unsigned char    transform[OLDTRANSFORMLEN];
  unsigned char    anglepairs[ANGLEPAIRSLENFLEX];
  uint32_t         num_xfrms;
  uint32_t         num_tgts;
  uint32_t         colineartgt;
  uint32_t         coplanartgts;
  uint32_t         tgt_status[MAX_TARGETSFLEX];
  double           bestTolerance;
  double           worstTolOfAnyCalcXfrm;
  double           worstTolOfAnyInTolXfrm;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_chngdisp_resp {
  struct k_header  hdr;
  uint32_t     displayPeriod;
  uint16_t     resp_crc;
} __attribute__ ((packed));

struct parse_rfrshrt_resp {
  struct k_header  hdr;
  uint32_t     num_points;
  uint32_t     laser_period;
  uint16_t     resp_crc;
} __attribute__ ((packed));

struct parse_rightondofullreg_resp {
  struct k_header    hdr;
  unsigned char      transform[OLDTRANSFORMLEN];
  unsigned char      anglepairs[OLDANGLEPAIRSLEN];
  struct k_xy_double target_geo_angle[MAX_TARGETSOLD];
  uint16_t           resp_crc;
} __attribute__ ((packed));

struct parse_rightondofullregwithfeedback_resp {
  struct k_header    hdr;
  unsigned char      transform[NEWTRANSFORMLEN];
  double             besttolerance;
  double             worsttoleranceofanycalculatedtransform;
  double             worsttoleranceofanyintolerancetransform;
  uint32_t           numberoftransforms;
  uint32_t           numberoftargets;
  unsigned char      anglepairs[OLDANGLEPAIRSLEN];
  unsigned char      anglepairsextra[16];
  struct k_xy_double target_geo_angle[MAX_TARGETSOLD + 2];
  uint32_t           target_status[MAX_TARGETSOLD + 2];
  unsigned char      fill[2];
  uint16_t           resp_crc;
} __attribute__ ((packed));

struct parse_flexrightondofullregwithfeedback_resp {
  struct k_header    hdr;
  unsigned char      transform[NEWTRANSFORMLEN];
  double             besttolerance;
  double             worsttoleranceofanycalculatedtransform;
  double             worsttoleranceofanyintolerancetransform;
  uint32_t           numberoftransforms;
  uint32_t           numberoftargets;
  unsigned char      anglepairs[ANGLEPAIRSLENFLEX];
  struct k_xy_double target_geo_angle[MAX_TARGETSFLEX];
  uint32_t           target_status[MAX_TARGETSFLEX];
  uint32_t           colineartargets;
  uint32_t           coplanartargets;
  uint16_t           resp_crc;
} __attribute__ ((packed));

struct parse_flexfail_resp {
  struct k_header    hdr;
  uint32_t           colineartargets;
  uint32_t           coplanartargets;
  uint32_t           fill[4];
  uint16_t           resp_crc;
} __attribute__ ((packed));

struct parse_chngxfrmtol_resp
{
  struct k_header  hdr;
  double           new_tolerance;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_superfom_resp
{
  struct k_header      hdr;
  uint32_t             num_targets;
  double               chi_square;
  struct k_targetinfo  tgt_info[MAX_TARGETSFLEX];
  uint16_t             resp_crc;
} __attribute__ ((packed));

struct parse_calibxy_resp
{
  struct k_header  hdr;
  double           foundX;
  double           foundY;
  uint16_t         resp_crc;
} __attribute__ ((packed));

struct parse_qcmgr_resp
{
  struct k_header  hdr;
  uint16_t         currQCSet;
  uint16_t         QCPlybyte23;
  uint16_t         resp_crc;
} __attribute__ ((packed));

#endif
