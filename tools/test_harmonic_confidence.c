#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "harmonic_confidence.h"
int main(int argc,char **argv){
 assert(argc==2);FILE *f=fopen(argv[1],"rb");assert(f);fseek(f,0,SEEK_END);long n=ftell(f);rewind(f);
 unsigned char *b=malloc(n);assert(b);assert(fread(b,1,n,f)==(size_t)n);fclose(f);
 harmonic_confidence s={0};v32_harmonic_measurement m;unsigned packets=0,candidates=0,qualified=0;
 for(long at=0;at+216<=n;at++)if(v32_harmonic_decode(b+at,216,&m)){
  packets++;unsigned mask=harmonic_confirm(&s,&m,1),detected=m.valid_mask&~m.noise_mask&30u;
  if(m.thd_valid){candidates++;if(detected && (mask&(detected|1u))==(detected|1u))qualified++;}
 }
 assert(packets>=60 && candidates>0 && qualified==0);
 memset(&s,0,sizeof s);memset(&m,0,sizeof m);m.valid_mask=31;m.signal_x6[0]=100000;m.signal_x6[2]=1000;m.noise_mask=26;
 for(unsigned id=1;id<=3;id++){m.result_id=id;unsigned mask=harmonic_confirm(&s,&m,1);assert(!!(mask&4)==(id==3));
  assert(harmonic_confirm(&s,&m,1)==mask);}
 m.cfg_id++;assert(!harmonic_confirm(&s,&m,1));
 harmonic_confirm(&s,&m,0);assert(!s.active);
 printf("CONFIDENCE_PASS packets=%u candidates=%u qualified=%u persistent_H3_reset_duplicate_PASS\n",packets,candidates,qualified);
 free(b);return 0;
}
