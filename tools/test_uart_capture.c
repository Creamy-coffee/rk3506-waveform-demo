#define _DEFAULT_SOURCE
#include "v32cap3.h"
#include "wave_source.h"
#include <assert.h>
#include <pty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
/* Parser updates only this metadata; no GUI or synthetic input is used. */
wave_source_t wave_source_v32={.sample_rate=65000000};
int main(void)
{
    int master,slave;char dev[128],path[128];
    assert(openpty(&master,&slave,dev,NULL,NULL)==0);close(slave);
    snprintf(path,sizeof path,"/tmp/v32_uart_capture_%ld.bin",(long)getpid());
    assert(setenv("WAVE_RAW_CAPTURE",path,1)==0);
    assert(v32_init_uart(dev,2000000)==0);
    enum {N=512*1024};unsigned char *data=malloc(N),*got=malloc(N);assert(data && got);
    for(int i=0;i<N;i++)data[i]=(unsigned char)(i*37+13);
    size_t at=0;while(at<N){ssize_t n=write(master,data+at,N-at);assert(n>0);at+=(size_t)n;}
    struct stat st={0};int ready=0;
    for(int i=0;i<500;i++){if(!stat(path,&st) && st.st_size==N){ready=1;break;}usleep(10000);}
    v32_shutdown();assert(ready);close(master);
    FILE *f=fopen(path,"rb");assert(f);assert(fread(got,1,N,f)==N && fgetc(f)==EOF);fclose(f);
    assert(!memcmp(data,got,N));free(data);free(got);unlink(path);unsetenv("WAVE_RAW_CAPTURE");
    puts("UART_CAPTURE_PASS single_reader_exact_512KiB_shutdown_drain");return 0;
}
