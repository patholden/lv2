#include <stdint.h>
/*
 *  $Id$
 */

#ifndef SHOWTARGETS_H
#define SHOWTARGETS_H  1

enum
{
    SHOWTARGETNUM1=1,
    SHOWTARGETNUM2,
    SHOWTARGETNUM3,
    SHOWTARGETNUM4,
    SHOWTARGETNUM5,
    SHOWTARGETNUM6,
    SHOWTARGETNUM7,
    SHOWTARGETNUM8,
    SHOWTARGETNUM9,
    SHOWTARGETNUM10,
    SHOWTARGETNUM11,
    SHOWTARGETNUM12,
    SHOWTARGETNUM13,
    SHOWTARGETNUM14,
    SHOWTARGETNUM15,
    SHOWTARGETNUM16,
    SHOWTARGETNUM17,
    SHOWTARGETNUM18,
    SHOWTARGETNUM19,
    SHOWTARGETNUM20,
    SHOWTARGETNUM21,
    SHOWTARGETNUM22,
    SHOWTARGETNUM23,
    SHOWTARGETNUM24,
    SHOWTARGETCROSS=31,
    SHOWTARGETBOX=32
};

void DoShowTargets(struct lg_master *pLgMaster, struct parse_showtgt_parms *Parameters, uint32_t respondToWhom );
#endif
