/* Reused windowed Mth-power coarse FFT from firmware/linux/iq_measure.c. */
static int bpsk_coarse(const int16_t *i,const int16_t *q,size_t n,double fs,
                  double rate,unsigned order,double *step) {
 enum { L=4096 };double re[L]={0},im[L]={0},best=0,delta=0;size_t k,j,len,peak=0;
 if(!i||!q||!step||n!=1024||!isfinite(fs)||fs<=0||!isfinite(rate)||rate<=0||
    (order!=2&&order!=4))return -1;
 for(k=0;k<n;k++) {
  double a=i[k]/2048.0,b=q[k]/2048.0,x=a*a-b*b,y=2*a*b,w=.5-.5*cos(2*BPSK_PI*k/(n-1));
  if(order==4){double t=x*x-y*y;y=2*x*y;x=t;}
  re[k]=x*w;im[k]=y*w;
 }
 for(k=1,j=0;k<L;k++) {
  size_t bit=L>>1;while(j&bit){j^=bit;bit>>=1;}j^=bit;
  if(k<j){double t=re[k];re[k]=re[j];re[j]=t;t=im[k];im[k]=im[j];im[j]=t;}
 }
 for(len=2;len<=L;len<<=1) {
  double cs=cos(-2*BPSK_PI/len),sn=sin(-2*BPSK_PI/len);
  for(k=0;k<L;k+=len) {
   double wr=1,wi=0;
   for(j=0;j<len/2;j++) {
    size_t u=k+j,v=u+len/2;double tr=wr*re[v]-wi*im[v],ti=wr*im[v]+wi*re[v];
    double next=wr*cs-wi*sn;re[v]=re[u]-tr;im[v]=im[u]-ti;
    re[u]+=tr;im[u]+=ti;wi=wi*cs+wr*sn;wr=next;
   }
  }
 }
 for(k=0;k<L;k++) {
  double f=(k<=L/2?(double)k:(double)k-L)*fs/L,p=re[k]*re[k]+im[k]*im[k];
  if(fabs(f)<=fmin(fs*.45,rate*2) && p>best){best=p;peak=k;}
 }
 if(best<=0)return -1;
 {size_t left=(peak+L-1)%L,right=(peak+1)%L;
  double x=log(fmax(re[left]*re[left]+im[left]*im[left],1e-100));
  double y=log(best),z=log(fmax(re[right]*re[right]+im[right]*im[right],1e-100));
  if(fabs(x-2*y+z)>1e-12)delta=.5*(x-z)/(x-2*y+z);
  if(delta>.5)delta=.5;if(delta<-.5)delta=-.5;
 }
 *step=((peak<=L/2?(double)peak:(double)peak-L)+delta)*2*BPSK_PI/(L*order);return 0;
}
