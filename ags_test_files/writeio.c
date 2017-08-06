#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

int main(int argc, char *argv[])
{
  int Address, Data;

  if(argc<3)
  {
    printf("Syntax: writeio <address> <data>\r\n\r\n");
    return 1;
  }

  Address=atoi(argv[1]);
  Data=atoi(argv[2]);

  if(ioperm(Address,8,1))
  {
    printf("ioperm error\r\n");
    return 2;
  }

  printf("Writing %02X to address %04X\r\n",Data,Address);

  outb(Data, Address);
}
