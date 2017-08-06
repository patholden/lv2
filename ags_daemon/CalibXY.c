#include <stdint.h>
#include <math.h>
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
#include "comm_loop.h"
#include "parse_data.h"
#include "LaserInterface.h"
#include "CalibXY.h"
#include "Protocol.h"

void CalibXY(struct lg_master *pLgMaster, struct parse_calibxy_parms* pInp, uint32_t respondToWhom)
{
    struct parse_calibxy_resp *pResp=(struct parse_calibxy_resp *)pLgMaster->theResponseBuffer;
    double xgotoang,  ygotoang;
    int16_t xbinary, ybinary;
    double aXf, aYf;
    double Ztr, Xf, Yf;
    uint32_t theError;

    memset ((char *)pResp, 0, sizeof(struct parse_calibxy_resp));

    xgotoang = pInp->steerX;
    ygotoang = pInp->steerY;
    Ztr      = pInp->inp_Z;

    theError  = ConvertExternalAnglesToBinary(pLgMaster, xgotoang, ygotoang, &xbinary, &ybinary);

    if (theError)
      {
	pResp->hdr.status = RESPFAIL;
	pResp->hdr.errtype1 = RESPE1INANGLEOUTOFRANGE;
	HandleResponse(pLgMaster, (sizeof(struct parse_basic_resp)-kCRCSize), kRespondExtern);
	return;
      }

    ConvertBinaryToMirrorAngles(xbinary, ybinary, &aXf, &aYf);

    ConvertMirrorToGeometricAngles( &aXf, &aYf );

    XYFromGeometricAnglesAndZ(pLgMaster, aXf, aYf, Ztr, &Xf, &Yf );

    pResp->hdr.status = RESPGOOD;
    pResp->foundX = Xf;
    pResp->foundY = Yf;
    HandleResponse(pLgMaster, (sizeof(struct parse_calibxy_resp)-kCRCSize), kRespondExtern);

    return;
}
