#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <math.h>

#define   LG_BASE            0x380
#define   TFPORTRL           0x390          // Read TF IO port address
#define   TFPORTRH           0x392          // Read upper byte of TF IO ;

int
main( int argc, char ** argv )
{
   int i,j;
   unsigned char rl_val, rh_val;

   if (ioperm(LG_BASE, 32, 1)) {perror("ioperm"); exit(EXIT_FAILURE);}

   while (1)
     {
       rl_val = ~(inb( TFPORTRL ));
       rh_val = ~(inb( TFPORTRH ));
       printf( " IO Addr %x = %2x, %x = %2x\n",TFPORTRL, rl_val &0xFF, TFPORTRH, rh_val &0xFF);
       sleep(10);
     }
   exit(EXIT_SUCCESS);
}
