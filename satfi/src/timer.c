#include <stdio.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>

void seconds_sleep(unsigned seconds)
{
  struct timeval tv = {seconds, 0};
  int err;
  do
  {
    err = select(0,NULL,NULL,NULL,&tv);
  } while (err<0 || errno == EINTR);
}

void milliseconds_sleep(unsigned long milliseconds)
{
  struct timeval tv = {milliseconds/1000, (milliseconds%1000)*1000};
  int err;
  do
  {
    err = select(0,NULL,NULL,NULL,&tv);
  } while (err<0 || errno == EINTR);
}

void microseconds_sleep(unsigned long microseconds)
{
  struct timeval tv = {microseconds/1000000, microseconds%1000000};
  int err;
  do
  {
    err = select(0,NULL,NULL,NULL,&tv);
  } while (err<0 || errno == EINTR);
}
