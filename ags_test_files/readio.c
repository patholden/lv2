#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>

#define WRAP_COUNT  16

int main(int argc, char *argv[])
{
  int Start, End;
  int Wrap=0, Data;

  if(argc<2)
  {
    printf("Syntax: readio <start> {<end>}\r\n\r\n");
    return 1;
  }

  Start=atoi(argv[1]);

  if(argc>2)
    End=atoi(argv[2]);
  else
    End=Start;

  if(End<Start)
  {
    printf("ERROR - Invalid address range\r\n\r\n");
    return 2;
  }

  if(ioperm(Start, 8*(End-Start+1),1))
  {
    printf("ioperm error\r\n");
    return 3;
  }

  printf("Start=%04X, End=%04X\r\n",Start,End);

  for(;Start<=End;Start++)
  {
    Data=inb(Start);
    printf("%02X ",Data);
    Wrap++;
    if(Wrap>=WRAP_COUNT)
    {
      printf("\r\n");
      Wrap=0;
    }
  }
  printf("\r\n");
}
