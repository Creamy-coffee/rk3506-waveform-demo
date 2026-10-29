#include "image_transport.h"
#include <assert.h>
#include <stdio.h>
int main(int argc,char **argv){
 assert(argc==3);
 for(int orientation=1;orientation<=2;orientation++){
  FILE *f=fopen(argv[orientation],"rb");assert(f);unsigned char packets[128][68];
  assert(fread(packets,68,128,f)==128);fclose(f);
  image_receiver_t r;image_receiver_clear(&r);
  for(int i=127;i>=0;i--){image_accept_packet(&r,packets[i]);image_accept_packet(&r,packets[i]);
   assert(r.received==(unsigned)(128-i));if(i>0)assert(!r.complete);}
  assert(r.complete && r.chunks==128 && r.total_bytes==4096 && !r.crc_errors && !r.invalid_packets);
  assert(r.width==(orientation==1?128:256) && r.height==(orientation==1?256:128));
  for(unsigned y=0;y<r.height;y++)for(unsigned x=0;x<r.width;x++){
   unsigned at=y*(r.width/8)+x/8;assert(image_receiver_pixel(&r,x,y)==((r.pixels[at]>>(7-x%8))&1));}
  packets[127][40]^=1;image_accept_packet(&r,packets[127]);assert(r.crc_errors==1 && r.complete);
 }
 puts("LARGE_IMAGE_PASS both_aspects 128_chunks reverse_duplicates_pixels_CRC");return 0;
}
