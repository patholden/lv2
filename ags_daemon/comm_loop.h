#include <stdint.h>
#ifndef ETHER_SERVER_LOOP_H
#define ETHER_SERVER_LOOP_H

int CommInit(struct lg_master *pLgMaster);
int CommConfigSockfd(struct lg_master *pLgMaster);
int DoProcEnetPackets(struct lg_master *pLgMaster);
int DoProcSerial(struct lg_master *pLgMaster);
void SendConfirmation(struct lg_master *pLgMaster, unsigned char theCommand);
void SendA3(struct lg_master *pLgMaster);
void HandleResponse (struct lg_master *pLgMaster, int32_t lengthOfResponse, uint32_t gRespondToWhom );
int CheckInput(struct lg_master *pLgMaster);

int TestEtherOrSerial (struct lg_master *pLgMaster);

#define kRespondExtern          0U
#define kDoNotRespond           5U
#define TEST_TARGET_FIND 1
#define COMM_CONFIRM_LEN 6
#define COMM_A3_LEN      1
#define COMM_RESP_ERROR_LEN1  (sizeof(uint32_t) + 2)
#define COMM_RESP_MIN_LEN  (sizeof(uint32_t) + 2)
#define COMM_ENET_PKT_SIZE    1500
#define COMM_RESP_MAX_LEN     4096
#define COMM_RECV_MIN_LEN  6
#define COMM_ETHER       2
#define COMM_SERIAL      1
#define COMM_MAX_ENET_RETRIES  4
#define COMM_RECV_MAX_SIZE    0x20000  // Set to 128k, Max buffer=8192 but need
                                      // space for header & CRC.  Assume every byte > 0x80
                                      // so payload ends up being doubled in size
                                      // plus extended blocks for more data.  Display function
                                      // can hold a lot of blocks.
#define COMM_RECV_CHUNK_SIZE  0x4000   // Set to 8k+enough for header, used by comm-loop to read in chunks.

enum
{
    COMMLOOP_SUCCESS=0,
    COMMLOOP_ERROR,
};
#endif
