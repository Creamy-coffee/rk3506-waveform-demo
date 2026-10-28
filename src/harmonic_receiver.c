#include "harmonic_receiver.h"
#include <math.h>
#include <string.h>
#define VALID UINT32_C(0x80000000)
#define HOST UINT32_C(512)
#define INVALID (1u|2u|4u|8u|32u|128u)
static uint32_t rd32(const uint8_t *p){
 return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static uint64_t rd64(const uint8_t *p){return rd32(p)|((uint64_t)rd32(p+4)<<32);}
static uint32_t crc32(const uint8_t *p,size_t n){
 uint32_t c=UINT32_MAX;
 for(size_t j=0;j<n;j++){c^=p[j];for(unsigned k=0;k<8;k++)c=(c>>1)^((c&1)?UINT32_C(0xedb88320):0);}
 return ~c;
}
/* floor(sqrt(x)), exact uint64 arithmetic, usable on 32-bit ARM. */
static uint32_t root64(uint64_t x){
 uint64_t bit=UINT64_C(1)<<62,r=0;
 while(bit>x)bit>>=2;
 while(bit){if(x>=r+bit){x-=r+bit;r=(r>>1)+bit;}else r>>=1;bit>>=2;}
 return (uint32_t)r;
}
/* floor((sum << 48) / base) without compiler-specific 128-bit integers.
 * sum is <=42 bits, base <=40 bits; remainder < base, so uint64 is exact. */
static int thd_exact(uint64_t sum,uint64_t base,uint32_t *q24){
 uint64_t rem=0,q=0;
 if(!base)return 0;
 for(int k=89;k>=0;k--){
  unsigned next=k>=48?(unsigned)((sum>>(k-48))&1u):0;
  rem=(rem<<1)|next;
  if(rem>=base){rem-=base;if(k>=64)return 0;q|=UINT64_C(1)<<k;}
 }
 *q24=root64(q);return 1;
}
int v32_harmonic_decode(const uint8_t *p,size_t size,v32_harmonic_measurement *out){
 if(!p||!out||size!=V32_HARMONIC_BYTES||memcmp(p,"V32HRM1!",8))return 0;
 uint32_t version=rd32(p+8);
 if((version!=1&&version!=2)||rd32(p+12)!=V32_HARMONIC_BYTES||crc32(p,212)!=rd32(p+212))return 0;
 v32_harmonic_measurement m;memset(&m,0,sizeof m);
 m.version=version;m.result_id=rd32(p+16);m.cfg_id=rd32(p+20);
 m.adc_fs=rd32(p+24);m.nfft=rd32(p+28);m.reference_fcw=rd32(p+32);
 m.decimation=rd32(p+36);m.age_ticks=rd32(p+40);m.flags=m.wire_flags=rd32(p+44);
 m.channel=rd32(p+48);m.window=rd32(p+52);m.valid_mask=rd32(p+56);m.noise_mask=rd32(p+60);
 m.thd_q24=m.fpga_thd_q24=rd32(p+64);m.dropped_frames=rd32(p+68);m.computed_on_host=version==2;
 if(!!(m.flags&HOST)!=(version==2)||(version==2&&m.fpga_thd_q24))return 0;
 if(!m.adc_fs||m.nfft<256||m.nfft>4096||(m.nfft&(m.nfft-1)))return 0;
 if(m.decimation!=1&&m.decimation!=16&&m.decimation!=256&&m.decimation!=4096&&m.decimation!=16384)return 0;
 if(m.channel>1||m.window>2||(m.valid_mask&~31u)||(m.noise_mask&~m.valid_mask)||(m.noise_mask&~31u))return 0;
 uint64_t sum=0;
 for(unsigned i=0;i<5;i++){
  m.band[i]=rd64(p+72+8*i);m.noise[i]=rd64(p+112+8*i);m.signal_x6[i]=rd64(p+152+8*i);m.peak_bin[i]=rd32(p+192+4*i);
  if((m.band[i]>>40)||(m.noise[i]>>40)||(m.signal_x6[i]>>40)||m.peak_bin[i]>=m.nfft/2)return 0;
  uint64_t b=6*m.band[i],n=7*m.noise[i];
  if(m.signal_x6[i]!=(b>n?b-n:0))return 0;
  if(i)sum+=m.signal_x6[i];
 }
 m.thd_valid=!!(m.flags&VALID);
 if(m.thd_valid){
  if(m.valid_mask!=31||(m.flags&INVALID)||!m.signal_x6[0]||m.window!=0)return 0;
  uint32_t q=0;int fits=thd_exact(sum,m.signal_x6[0],&q);
  if(version==1){if(!fits||q!=m.thd_q24)return 0;}
  else if(!fits){m.thd_valid=0;m.flags=(m.flags&~VALID)|32u;}
  else m.thd_q24=q;
 }
 m.thd_below_noise=!!(m.wire_flags&VALID)&&((m.noise_mask&30u)==30u);
 if(m.thd_below_noise){m.thd_valid=0;m.flags=(m.flags&~VALID)|16u;}
 *out=m;return 1;
}
int v32_harmonic_dbc(const v32_harmonic_measurement *m,unsigned order,double *dbc){
 if(!m||!dbc||order<1||order>5)return 0;
 if(!(m->valid_mask&1u)||(m->noise_mask&1u)||(m->flags&(1u|2u|8u|128u)))return 0;
 unsigned i=order-1;
 if(!(m->valid_mask&(1u<<i))||(m->noise_mask&(1u<<i))||!m->signal_x6[i]||!m->signal_x6[0])return 0;
 *dbc=10.0*log10((double)m->signal_x6[i]/(double)m->signal_x6[0]);return 1;
}
