#ifndef V32_IMAGE_TRANSPORT_H
#define V32_IMAGE_TRANSPORT_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* V32IMG1! v1: 32-byte header, 32-byte payload, CRC32; MSB-first Manchester.
 * This consumes recovered symbols, never generator-side image files. The two
 * pair phases and sync polarity handle arbitrary entry/BPSK sign ambiguity.
 * Only CRC-valid packets become visible; complete requires whole-image CRC.
 */
#define IMG_MAX_BYTES 2048u
#define IMG_PACKET_BYTES 68u
#define IMG_CHUNK_BYTES 32u
typedef struct {
    uint64_t sync;unsigned sync_bits,collect,at,byte_bits,invert;
    uint8_t byte,packet[IMG_PACKET_BYTES];
    int first,have_first;
} image_bit_path_t;
typedef struct {
    image_bit_path_t path[2];
    uint64_t symbols,received_mask,packets,duplicates,crc_errors,invalid_packets,gaps,images;
    uint32_t image_id,image_crc,total_bytes,revision;
    unsigned width,height,chunks,received,complete,have_image;
    uint8_t pixels[IMG_MAX_BYTES];
} image_receiver_t;

static inline uint16_t image_le16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static inline uint32_t image_le32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static inline uint32_t image_crc32(const uint8_t *p,size_t n){
    uint32_t c=~0u;
    for(size_t i=0;i<n;i++){c^=p[i];for(int b=0;b<8;b++)c=(c>>1)^((c&1)?0xedb88320u:0);}
    return ~c;
}
static inline void image_receiver_clear(image_receiver_t *r){memset(r,0,sizeof *r);r->revision=1;}
static inline void image_receiver_gap(image_receiver_t *r){memset(r->path,0,sizeof r->path);r->gaps++;}
static inline void image_accept_packet(image_receiver_t *r,const uint8_t *p){
    if(memcmp(p,"V32IMG1!",8) || image_crc32(p,64)!=image_le32(p+64)){r->crc_errors++;return;}
    unsigned w=image_le16(p+10),h=image_le16(p+12),chunk=image_le16(p+14);
    uint32_t id=image_le32(p+16),crc=image_le32(p+20),total=image_le32(p+24);
    unsigned index=image_le16(p+28),len=image_le16(p+30);
    unsigned size=((w+7)/8)*h,chunks=(size+31)/32;
    if(p[8]!=1 || p[9]!=1 || !w || !h || w>128 || h>128 || total!=size || total>IMG_MAX_BYTES ||
       chunk!=32 || index>=chunks || len!=(size-index*32<32?size-index*32:32)){
        r->invalid_packets++;return;
    }
    if(!r->have_image || id!=r->image_id){
        r->have_image=1;r->image_id=id;r->image_crc=crc;r->total_bytes=total;
        r->width=w;r->height=h;r->chunks=chunks;r->received=0;r->received_mask=0;r->complete=0;
        memset(r->pixels,0,sizeof r->pixels);r->revision++;
    }else if(r->width!=w || r->height!=h || r->image_crc!=crc || r->total_bytes!=total){
        r->invalid_packets++;return;
    }
    r->packets++;
    uint64_t mask=UINT64_C(1)<<index;
    if(r->received_mask&mask){
        r->duplicates++;
        if(!memcmp(r->pixels+index*32,p+32,len))return;
        /* A different CRC-valid replacement may repair a previous bad chunk.
         * It still cannot complete until the independent image CRC passes. */
        r->complete=0;
    }else {r->received_mask|=mask;r->received++;}
    memcpy(r->pixels+index*32,p+32,len);r->revision++;
    if(r->received==r->chunks){
        if(image_crc32(r->pixels,r->total_bytes)==r->image_crc){
            if(!r->complete)r->images++;
            r->complete=1;
        }else{
            r->crc_errors++;r->complete=0;r->received_mask=0;r->received=0;r->revision++;
        }
    }
}
static inline void image_bit_push(image_receiver_t *r,image_bit_path_t *p,int bit){
    const uint64_t magic=UINT64_C(0x563332494d473121);
    if(!p->collect){
        p->sync=(p->sync<<1)|(unsigned)bit;if(p->sync_bits<64)p->sync_bits++;
        if(p->sync_bits==64 && (p->sync==magic || p->sync==~magic)){
            p->collect=1;p->at=8;p->byte=p->byte_bits=0;p->invert=p->sync!=magic;
            memcpy(p->packet,"V32IMG1!",8);
        }
        return;
    }
    p->byte=(uint8_t)((p->byte<<1)|(unsigned)(bit^(int)p->invert));
    if(++p->byte_bits==8){
        p->packet[p->at++]=p->byte;p->byte_bits=0;p->byte=0;
        if(p->at==IMG_PACKET_BYTES){image_accept_packet(r,p->packet);p->collect=0;p->sync=0;p->sync_bits=0;}
    }
}
static inline void image_receiver_symbol(void *context,int bit){
    image_receiver_t *r=(image_receiver_t*)context;
    if(bit<0 || bit>1){image_receiver_gap(r);return;}
    unsigned parity=(unsigned)(r->symbols++&1u);
    for(unsigned i=0;i<2;i++){
        image_bit_path_t *p=&r->path[i];
        if(parity==i){p->first=bit;p->have_first=1;}
        else if(p->have_first){
            p->have_first=0;
            if(p->first==bit){p->collect=0;p->sync=p->sync_bits=0;}
            else image_bit_push(r,p,p->first);
        }
    }
}
static inline int image_receiver_pixel(const image_receiver_t *r,unsigned x,unsigned y){
    if(!r->have_image || x>=r->width || y>=r->height)return -1;
    unsigned at=y*((r->width+7)/8)+x/8;
    if(!(r->received_mask&(UINT64_C(1)<<(at/32))))return -1;
    return (r->pixels[at]>>(7-x%8))&1u;
}
#endif
