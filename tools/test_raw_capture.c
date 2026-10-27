#define _DEFAULT_SOURCE
#include "raw_capture.h"
#include <assert.h>

int main(void)
{
    char path[128];snprintf(path,sizeof path,"/tmp/v32_rawcap_%ld.bin",(long)getpid());
    raw_capture_t c={0};unsigned char data[4096],copy[4096];
    for(size_t i=0;i<sizeof data;i++)data[i]=(unsigned char)(i*37u);
    assert(raw_capture_start(&c,NULL,8192)==0);raw_capture_feed(&c,data,sizeof data);raw_capture_finish(&c);
    assert(raw_capture_start(&c,path,128u*1024u)==1);
    for(int i=0;i<24;i++)raw_capture_feed(&c,data,sizeof data);
    raw_capture_finish(&c);assert(!c.error && !c.overflow && c.written==24u*sizeof data);
    FILE *f=fopen(path,"rb");assert(f);
    for(int i=0;i<24;i++){assert(fread(copy,1,sizeof copy,f)==sizeof copy);assert(!memcmp(data,copy,sizeof data));}
    assert(fgetc(f)==EOF);fclose(f);
    assert(raw_capture_start(&c,path,1024)<0 && errno==EEXIST);unlink(path);
    assert(raw_capture_start(&c,path,512)==1);raw_capture_feed(&c,data,sizeof data);raw_capture_finish(&c);
    assert(!c.error && !c.overflow && c.limited && c.written==512);
    f=fopen(path,"rb");assert(f);assert(fread(copy,1,sizeof copy,f)==512 && !memcmp(copy,data,512));fclose(f);unlink(path);
    assert(raw_capture_start(&c,path,RAW_CAPTURE_QUEUE*2u)==1);
    unsigned char *large=malloc(RAW_CAPTURE_QUEUE+1u);assert(large);
    raw_capture_feed(&c,large,RAW_CAPTURE_QUEUE+1u);free(large);raw_capture_finish(&c);
    assert(c.overflow && !c.error && !c.written);unlink(path);
    /* A closed sink causes an explicit writer failure, never a fake complete file. */
    assert(raw_capture_start(&c,path,8192)==1);close(c.fd);
    raw_capture_feed(&c,data,sizeof data);raw_capture_finish(&c);assert(c.error==EBADF);unlink(path);
    puts("RAW_CAPTURE_PASS disabled_exact_bytes_no_overwrite_limit_overflow_writer_error");
    return 0;
}
