#define _DEFAULT_SOURCE
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include "../src/ui.c"
static uint32_t pixels[1024*600];
/* Deterministic freshness: rendering/file I/O under ASan must not age replay frames. */
static unsigned long long test_ms=1000;
int __real_clock_gettime(clockid_t,struct timespec *);
int __wrap_clock_gettime(clockid_t id,struct timespec *ts){if(id==CLOCK_MONOTONIC){ts->tv_sec=test_ms/1000;ts->tv_nsec=(test_ms%1000)*1000000;return 0;}return __real_clock_gettime(id,ts);}
static int record_config,config_calls,last_mode,last_channel,last_rate,last_focus;
static uint32_t last_fcw;
int __real_v32_cmd_apply(int,int,int,uint32_t,int);
int __wrap_v32_cmd_apply(int mode,int channel,int rate,uint32_t fcw,int focus){
    if(!record_config)return __real_v32_cmd_apply(mode,channel,rate,fcw,focus);
    config_calls++;last_mode=mode;last_channel=channel;last_rate=rate;last_fcw=fcw;last_focus=focus;return 0;
}
static char fixture[128];
static unsigned raw_id;
static uint32_t fixture_fcw=66076420;
static unsigned flush_count;
static int fake_touch;static int32_t fake_touch_x,fake_touch_y;
void __real_port_touch_last_point(int32_t*,int32_t*);
void __wrap_port_touch_last_point(int32_t *x,int32_t *y){if(fake_touch){*x=fake_touch_x;*y=fake_touch_y;}else __real_port_touch_last_point(x,y);}
static void flush_cb(lv_display_t *d,const lv_area_t *a,uint8_t *p){(void)a;(void)p;flush_count++;lv_display_flush_ready(d);}
static void put(uint8_t *p,uint32_t v){for(int i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static uint32_t take32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static uint32_t crc(const uint8_t *p,size_t n){uint32_t c=~0u;for(size_t i=0;i<n;i++){c^=p[i];for(int k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
static void fixture_write(unsigned fid,unsigned cfg,unsigned mode,unsigned channel,unsigned flags,int holes,unsigned dec,int raw)
{
    FILE *f=fopen(fixture,"wb");assert(f);
    if(raw){
        uint8_t b[4140]={0};memcpy(b,"V32CAP3!",8);put(b+8,3);put(b+12,++raw_id);put(b+16,65000000);put(b+20,dec);put(b+24,1024);put(b+32,raw_id*1000000u);
        for(int i=0;i<1024;i++){
            double t=(double)i*dec/65000000;int x=(int)(200*(1+.5*sin(2*M_PI*1000*t))*cos(2*M_PI*1000000*t));
            int16_t a=x,q=x/2;memcpy(b+40+4*i,&a,2);memcpy(b+42+4*i,&q,2);
        }
        put(b+4136,crc(b,4136));assert(fwrite(b,1,sizeof b,f)==sizeof b);
    }
    uint8_t b[1076]={0};memcpy(b,"V32ANA1!",8);put(b+8,1);put(b+12,fid);put(b+16,65000000);put(b+20,4096);put(b+24,256);put(b+28,flags);put(b+32,cfg);
    unsigned kind=(mode==4||mode==6)?0:(mode==1||mode==5)?1:(mode==2||mode==3)?2:3;
    put(b+36,kind|(channel<<8)|(mode<<16));put(b+40,fixture_fcw);
    for(int i=0;i<256;i++){
        double p=2*M_PI*1000*(i+(fid-1)*256)/(65000000.0/4096);uint32_t x;
        if(mode==4)x=(uint16_t)(int16_t)(((i/16)&1)?150:-150)|((uint32_t)(uint16_t)(int16_t)(((i/32)&1)?150:-150)<<16);
        else if(mode==6)x=(uint16_t)(int16_t)(((i/16)&1)?200:-200);
        else if(mode==5)x=((i/16)&1)?25600:0;
        else if(mode==3)x=(uint32_t)(int32_t)(((i/16)&1)?1000:-1000)&0xffffffu;
        else if(!kind)x=(uint16_t)(int16_t)(200*cos(p))|((uint32_t)(uint16_t)(int16_t)(100*sin(p))<<16);
        else if(kind==1)x=(unsigned)(256*(100+50*sin(p)));
        else x=(uint32_t)(int32_t)(1000*sin(p))&0xffffffu;
        if(kind && holes && i%19==0)x|=0x80000000u;
        put(b+48+4*i,x);
    }
    put(b+1072,crc(b,1072));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void append_meas(unsigned hz0,unsigned hz1,unsigned valid){
    FILE *f=fopen(fixture,"ab");assert(f);uint8_t b[128]={0};memcpy(b,"V32MEAS!",8);
    put(b+8,V32_MEAS_VERSION);put(b+12,128);put(b+16,raw_id);put(b+24,raw_id*1000000u);put(b+32,65000000);
    put(b+84,hz0);put(b+88,hz1);b[92]=(uint8_t)valid;put(b+124,crc(b,124));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void append_spec(void){
    FILE *f=fopen(fixture,"ab");assert(f);uint8_t b[2096]={0};memcpy(b,"V32SPEC!",8);
    put(b+8,1);put(b+12,sizeof b);put(b+16,raw_id);put(b+24,4096);put(b+28,2048);put(b+32,1000000);put(b+36,0);put(b+40,120);
    memset(b+44,64,2048);b[44+64]=240;put(b+2092,crc(b,2092));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void append_harmonic(unsigned id,unsigned channel,unsigned noise_mask,unsigned flags){
    FILE *f=fopen(fixture,"ab");assert(f);uint8_t b[216]={0};memcpy(b,"V32HRM1!",8);
    put(b+8,2);put(b+12,216);put(b+16,id);put(b+20,7);put(b+24,65000000);put(b+28,4096);
    put(b+32,66076420);put(b+36,1);put(b+44,0x80000200u|flags);put(b+48,channel);put(b+56,31);put(b+60,noise_mask);
    for(int i=0;i<5;i++){unsigned band=i?1000:1000000;put(b+72+i*8,band);put(b+112+i*8,0);put(b+152+i*8,band*6);put(b+192+i*4,63*(i+1));}
    put(b+212,crc(b,212));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void append_lock_meas(unsigned qual,unsigned flags){
    FILE *f=fopen(fixture,"ab");assert(f);uint8_t b[128]={0};memcpy(b,"V32MEAS!",8);
    put(b+8,V32_MEAS_VERSION);put(b+12,128);put(b+16,raw_id);put(b+24,raw_id*1000000u);put(b+32,65000000);
    put(b+40,v32_fcw_from_hz(1000000));put(b+44,409600);put(b+48,409600);put(b+52,65536);
    b[60]=0;b[61]=32;b[62]=qual;put(b+64,V32_ST_REQUIRED);b[68]=flags;
    put(b+84,1000000);put(b+88,1000000);b[92]=3;put(b+124,crc(b,124));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void append_iq(void){
    FILE *f=fopen(fixture,"ab");assert(f);uint8_t b[1068]={0};memcpy(b,"V32IQ01!",8);
    put(b+8,1);put(b+12,1);put(b+16,65000000);put(b+20,4096);put(b+24,256);put(b+32,3);
    for(int i=0;i<256;i++){b[40+4*i]=100;b[42+4*i]=100;}put(b+1064,crc(b,1064));assert(fwrite(b,1,sizeof b,f)==sizeof b);fclose(f);
}
static void draw(lv_display_t *d){test_ms+=40;lv_tick_inc(40);ui_tick();lv_obj_update_layout(lv_screen_active());lv_obj_invalidate(lv_screen_active());lv_refr_now(d);}
static void save(const char *dir,const char *name){char p[1024];snprintf(p,sizeof p,"%s/%s.ppm",dir,name);FILE *f=fopen(p,"wb");assert(f);fprintf(f,"P6\n1024 600\n255\n");for(int i=0;i<1024*600;i++){uint8_t b[3]={pixels[i]>>16,pixels[i]>>8,pixels[i]};fwrite(b,1,3,f);}fclose(f);}
static unsigned next_cp(const unsigned char **p){unsigned c=*(*p)++;if(c<128)return c;unsigned n=(c&0xe0)==0xc0?1:(c&0xf0)==0xe0?2:3;c&=(1u<<(6-n))-1;while(n--)c=(c<<6)|(*(*p)++&63);return c;}
static void glyphs(void){
    const char *text="波形原始包络窗口等待有效数据分析历史相对频偏峰值码中心载波参考实测○● BPSK PN7 待定判决比对中匹配检出误码有无效位对频频率无效/↺⊞图片接收中图已复位示波器操作调整时间两路公共量程回到恢复默认冻结再次运行继续显示串口采集不停止软件帧内对齐不是硬件单次开启后拖动读取左右缩放上下增益展开长时基跨帧存在死区实际到帧率刷新两者不同采样点数";
    const lv_font_t *fonts[]={&lv_font_v32_12,&lv_font_v32_16,&lv_font_v32_18,&lv_font_v32_20,&lv_font_v32_12_bold,&lv_font_v32_16_bold,&lv_font_v32_18_bold,&lv_font_v32_20_bold};
    for(unsigned k=0;k<sizeof fonts/sizeof fonts[0];k++){
        const unsigned char *p=(const unsigned char *)text;
        while(*p){unsigned cp=next_cp(&p);lv_font_glyph_dsc_t g;int present=fonts[k]->get_glyph_dsc(fonts[k],&g,cp,0);if(!present)fprintf(stderr,"MISSING_GLYPH font=%u U+%04X\n",k,cp);assert(present);}
    }
}

int main(int argc,char **argv)
{
    /* Physical regression: 1MHz at dec64 aliases to 15.625kHz. A trusted,
       independent full-band measurement must recover dec1, not lock to alias. */
    assert(scope_decimation_target(65000000,1024,64,1000000,1,20,0,0)==1);
    assert(scope_decimation_target(65000000,1024,64,15625,0,20,0,0)==64);
    assert(scope_decimation_target(65000000,1024,64,1000000,1,20,1,0)==16);
    assert(scope_decimation_target(65000000,1024,1,1000,1,20,0,0)==128);
    assert(scope_decimation_target(65000000,1024,1,0,0,0,0,1)==1);
    assert(scope_decimation_target(65000000,1024,1,15869,0,0,0,1)==1);
    float range_lo,range_scale;scope_auto_range(-4,8,&range_lo,&range_scale);assert(fabsf(1.f/range_scale-32)<1e-5f);
    assert(range_lo<=-4 && range_lo+1.f/range_scale>=8);
    scope_auto_range(-100,100,&range_lo,&range_scale);assert(fabsf(1.f/range_scale-220)<1e-4f);
    assert(argc>=2 && argc<=4);snprintf(fixture,sizeof fixture,"/tmp/v32_modulation_view_%ld.bin",(long)getpid());float lo,hi,out[1024],v[3]={10,10,10};assert(mod_view_range(v,3,1,&lo,&hi)&&lo==0&&hi>10);
    v[1]=NAN;mod_view_resample(v,3,out,9,0,20);assert(isnan(out[4]) && isfinite(out[0]) && isfinite(out[8]));
    float amp[5]={0,.5f,1,NAN,-1},up[5],dn[5];
    mod_view_bounds(amp,up,dn,5);
    assert(up[0]==.5f && dn[0]==.5f && up[1]==.75f && dn[1]==.25f);
    assert(up[2]==1 && dn[2]==0 && isnan(up[3]) && isnan(dn[3]) && isnan(up[4]));
    assert(amp[1]==.5f && isnan(amp[3]));
    assert(mod_view_count(256,65000000.0/4096,4)==64);
    lv_init();glyphs();lv_display_t *d=lv_display_create(1024,600);lv_display_set_color_format(d,LV_COLOR_FORMAT_XRGB8888);lv_display_set_buffers(d,pixels,NULL,sizeof pixels,LV_DISPLAY_RENDER_MODE_DIRECT);lv_display_set_flush_cb(d,flush_cb);
    fixture_write(1,3,1,0,0,0,1,1);char spec[128];snprintf(spec,sizeof spec,"file:%s",fixture);wave_source_v32_set_spec(spec);wave_source_select(&wave_source_v32);assert(wave_source_v32.init()==0);setenv("WAVE_SRC","real",1);
    ui_create();ui_set_page(UI_PAGE_MOD);draw(d);
    if(argc>=3){
        const unsigned capture_mode=argc==4 && !strcmp(argv[3],"bpsk")?6:argc==4 && !strcmp(argv[3],"fsk")?3:5;
        FILE *capture=fopen(argv[2],"rb");assert(capture);fseek(capture,0,SEEK_END);long size=ftell(capture);rewind(capture);assert(size>0 && size<64000000);
        uint8_t *data=malloc((size_t)size);assert(data && fread(data,1,(size_t)size,capture)==(size_t)size);fclose(capture);
        unsigned frames=0;s_an_type=(int)capture_mode;s_an_ch=0;s_an_rate=1;s_an_send_pending=0;s_an_tx_failed=0;s_an_have_target=1;s_an_target_fcw=66076420;s_ook_test=true;ook_reset(&s_ook);an_chips_refresh();
        for(long i=0;i+1076<=size;i++)if(!memcmp(data+i,"V32ANA1!",8) && crc(data+i,1072)==take32(data+i+1072) && ((take32(data+i+36)>>16)&255)==capture_mode){
            if(!frames)s_an_target_fcw=take32(data+i+40);
            FILE *f=fopen(fixture,"wb");assert(f && fwrite(data+i,1,1076,f)==1076);fclose(f);assert(v32_init_file(fixture)==0);
            test_ms+=20;lv_tick_inc(20);ui_tick();frames++;i+=1075;
        }
        if(capture_mode==6){
            draw(d);printf("BPSK_GUI_CAPTURE frames=%u n=%llu errors=%llu checks=%llu erasures=%llu lock=%d pn=%d evm=%f\n",frames,s_bpsk.bits,s_bpsk.pn_errors,s_bpsk.pn_checks,s_bpsk.erasures,s_bpsk.locked,s_bpsk.pn_locked,s_bpsk.evm);fflush(stdout);
            assert(frames>300 && s_bpsk.bits>5700 && s_bpsk.pn_errors==0 && s_bpsk.pn_checks>5000 && s_bpsk.erasures==0 && s_bpsk.locked && s_bpsk.pn_locked);
            assert(lv_obj_get_height(s_lbl_ook_bits)<=80);save(argv[1],"bpsk_real_capture");bpsk_reset(&s_bpsk);free(data);unlink(fixture);return 0;
        }
        draw(d);printf("%s_GUI_CAPTURE frames=%u n=%llu errors=%llu checks=%llu erasures=%llu lock=%d\n",capture_mode==3?"FSK":"OOK",frames,s_ook.bits,s_ook.alternating_errors,s_ook.alternating_checks,s_ook.erasures,s_ook.locked);fflush(stdout);
        assert(frames>300 && s_ook.bits>5900 && s_ook.alternating_errors==0 && s_ook.erasures==0 && s_ook.locked);save(argv[1],capture_mode==3?"fsk_real_capture":"ook_real_capture");free(data);unlink(fixture);return 0;
    }
    for(unsigned i=2;i<=16;i++){fixture_write(i,3,1,0,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);}
    assert(lv_obj_get_width(s_plot_modraw)>600 && lv_obj_get_width(s_plot_modana)>600);
    assert(strstr(lv_label_get_text(s_lbl_rawnote),"ANA history") && strstr(lv_label_get_text(s_lbl_rawnote),"4.03 ms"));
    assert(strstr(lv_label_get_text(s_lbl_modana_note),"16.13 ms"));
    assert(strstr(lv_label_get_text(s_lbl_modana_units),"A(t)"));save(argv[1],"envelope");
    assert(strstr(lv_label_get_text(s_lbl_modraw_units),"bounds, NOT RAW"));
    assert(strstr(lv_label_get_text(s_lbl_modcarrier),"RAW") && strstr(lv_label_get_text(s_lbl_modcarrier),"1.000"));
    uint32_t fcw=s_an_target_fcw;int pending=s_an_send_pending;
    cb_mod_view(NULL);draw(d);assert(s_mod_raw_view && s_an_target_fcw==fcw && s_an_send_pending==pending);
    assert(strstr(lv_label_get_text(s_lbl_rawnote),"us") && !strstr(lv_label_get_text(s_lbl_rawnote),"516"));save(argv[1],"raw");
    lv_area_t a,b;lv_obj_get_coords(s_chip_mv,&a);lv_obj_get_coords(s_chip_mw,&b);assert(a.x1>=0 && b.x2<1024);
    s_real_t_last=lv_tick_get()-20000u;
    fixture_write(17,3,1,0,0,0,1,0);assert(v32_init_file(fixture)==0);
    draw(d);assert(strstr(lv_label_get_text(s_lbl_rawnote),"stale"));save(argv[1],"stale");
    assert(!strstr(lv_label_get_text(s_lbl_modcarrier),"RAW") && strstr(lv_label_get_text(s_lbl_modcarrier),"DDC"));
    fixture_write(18,3,1,0,0,0,1,0);assert(v32_init_file(fixture)==0);
    cb_mod_view(NULL);cb_mod_window(NULL);draw(d);assert(!s_mod_raw_view && strstr(lv_label_get_text(s_lbl_rawnote),"16.01 ms"));
    // Valid zero samples and invalid holes must remain distinguishable.
    fixture_write(19,3,1,0,0,1,1,1);assert(v32_init_file(fixture)==0);draw(d);save(argv[1],"gaps");
    fixture_write(20,3,1,0,1,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(!strcmp(lv_label_get_text(s_lbl_m5),"--"));assert(!strcmp(lv_label_get_text(s_lbl_modana_note),"ANA | --"));
    s_an_type=4;s_an_ch=1;an_chips_refresh();fixture_write(1,4,4,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(strstr(lv_label_get_text(s_lbl_modana_units),"not decoded"));save(argv[1],"iq");
    assert(strstr(lv_label_get_text(s_lbl_modraw_title),"时间") && !strcmp(lv_label_get_text(s_lbl_m5),"未解码"));
    v32_ana_view_t digital;assert(v32_get_ana(&digital));assert(digital.nvalid==256 && digital.q[0]<0);
    s_an_type=6;s_bpsk_symbols=false;an_chips_refresh();fixture_write(1,6,6,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(v32_get_ana(&digital));assert(digital.mode==6 && digital.nvalid==256);assert(!strcmp(lv_label_get_text(s_lbl_m5),"等待判决"));save(argv[1],"bpsk");
    /* Wrong-rate lock must never imply verified bits, even with low EVM. */
    bpsk_decoder_t saved_bpsk=s_bpsk;int saved_test=s_ook_test;
    s_bpsk.locked=1;s_bpsk.pn_locked=0;s_bpsk.pn_checks=0;s_bpsk.pn_errors=0;s_bpsk.erasures=0;
    s_ook_test=0;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"待定判决"));
    s_ook_test=1;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"PN7比对中"));
    s_bpsk.pn_locked=1;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"PN7比对中"));
    s_bpsk.pn_checks=100;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"PN7匹配"));
    s_bpsk.erasures=3;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"匹配/无效位"));
    s_bpsk.pn_errors=1;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"PN7有误"));
    s_bpsk.locked=0;bpsk_status_refresh();assert(!strcmp(lv_label_get_text(s_lbl_m5),"等待判决"));
    s_bpsk=saved_bpsk;s_ook_test=saved_test;bpsk_status_refresh();
    cb_bpsk_view(NULL);draw(d);assert(s_bpsk_symbols && strstr(lv_label_get_text(s_lbl_modana_units),"symbols"));save(argv[1],"bpsk_wait");cb_bpsk_view(NULL);
    s_an_type=5;an_chips_refresh();fixture_write(1,7,5,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(v32_get_ana(&digital));assert(digital.y[0]==0 && digital.nvalid==256);assert(s_ook.locked && s_ook.bits>5);assert(strstr(lv_label_get_text(s_lbl_modraw_title),"OOK"));save(argv[1],"ook");
    uint32_t ook_fcw=s_an_target_fcw;int ook_pending=s_an_send_pending;
    cb_ook_test(NULL);fixture_write(2,7,5,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(s_ook.alternating_checks>5 && s_ook.alternating_errors==0);save(argv[1],"ook_bits");
    /* About 62 ANA/s arrive between 100 ms UI ticks. A seven-frame burst
       must be consumed in one fast tick, rather than limited to four (40/s). */
    {
        fixture_write(3,7,5,1,0,0,1,0);
        uint8_t frame[1076];FILE *burst=fopen(fixture,"rb");assert(burst);
        assert(fread(frame,1,sizeof frame,burst)==sizeof frame);fclose(burst);
        burst=fopen(fixture,"wb");assert(burst);
        for(unsigned fid=3;fid<=9;fid++){
            put(frame+12,fid);put(frame+1072,crc(frame,1072));
            assert(fwrite(frame,1,sizeof frame,burst)==sizeof frame);
        }
        fclose(burst);assert(v32_init_file(fixture)==0);draw(d);
        assert(s_ook.fid==9);v32_ana_view_t remaining;assert(!v32_pop_ana(&remaining));
    }
    lv_area_t oc;lv_obj_get_coords(s_chip_ook_test,&oc);assert(oc.x2<1024);
    ook_decoder_t saved_ook=s_ook;
    for(int k=0;k<64;k++)ook_emit(&s_ook,s_ook.last_bit^1,1);
    draw(d);assert(lv_obj_get_height(s_lbl_ook_bits)<=80);save(argv[1],"ook_bits_long");s_ook=saved_ook;
    cb_ook_on(NULL);draw(d);assert(!s_ook.bits && !strcmp(lv_label_get_text(s_lbl_m5),"未解码"));cb_ook_on(NULL);
    for(int k=0;k<5;k++)cb_ook_baud(NULL);assert(s_ook_rate==3 && s_an_target_fcw==ook_fcw && s_an_send_pending==ook_pending);
    fixture_write(3,7,5,1,0,1,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(v32_get_ana(&digital));assert(digital.nvalid<256 && !digital.yvalid[0] && digital.yvalid[1] && digital.y[1]==0);save(argv[1],"ook_gaps");
    s_an_type=3;an_chips_refresh();fixture_write(1,8,3,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(v32_get_ana(&digital));assert(digital.y[0]<0 && digital.y[16]>0);assert(s_ook.locked && s_ook.bits>5 && !s_ook.alternating_errors);assert(!strcmp(lv_label_get_text(s_lbl_m5),"同步"));assert(strstr(lv_label_get_text(s_lbl_modana_units),"Hz"));assert(!lv_obj_has_flag(s_chip_ook_on,LV_OBJ_FLAG_HIDDEN));save(argv[1],"fsk");
    saved_ook=s_ook;for(int k=0;k<64;k++)ook_emit(&s_ook,s_ook.last_bit^1,1);draw(d);assert(lv_obj_get_height(s_lbl_ook_bits)<=80);save(argv[1],"fsk_bits_long");s_ook=saved_ook;
    cb_mod_view(NULL);draw(d);assert(s_mod_raw_view);assert(strstr(lv_label_get_text(s_lbl_modraw_title),"原始"));cb_mod_view(NULL);draw(d);assert(!s_mod_raw_view);
    fixture_write(2,8,5,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(!strcmp(lv_label_get_text(s_lbl_m5),"--"));
    s_an_type=2;fixture_write(1,5,2,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(strstr(lv_label_get_text(s_lbl_modana_units),"Hz"));save(argv[1],"fm");
    S.running=false;const int previous=s_aroll_len;fixture_write(2,5,2,1,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(s_aroll_len==previous);S.running=true;draw(d);assert(s_aroll_len>previous);
    fixture_write(3,5,2,1,0,0,32768,1);assert(v32_init_file(fixture)==0);draw(d);assert(fabs(wave_source_v32_frame_rate_hz()-65000000.0/32768)<1e-6);
    /* Explicit retune selects only a fresh measurement of the selected channel. */
    record_config=1;config_calls=0;s_an_type=2;s_an_ch=1;s_an_rate=1;s_an_send_pending=0;
    s_an_have_target=1;s_an_target_fcw=66076420;s_an_tx_failed=0;
    fixture_write(10,5,2,1,0,0,1,1);append_meas(2000000,3000000,3);assert(v32_init_file(fixture)==0);
    v32_meas_view_t measured;assert(v32_get_meas(&measured) && measured.fft_hz1==3000000);
    ui_set_page(UI_PAGE_MOD);assert(strstr(lv_label_get_text(lv_obj_get_child(s_chip_auto,0)),"对频"));
    cb_auto(NULL);assert(s_an_send_pending && s_an_override && s_an_override_fcw==v32_fcw_from_hz(3000000));
    analysis_send_now();assert(config_calls==1 && last_mode==2 && last_channel==1 && last_rate==1 && last_focus==2 && last_fcw==v32_fcw_from_hz(3000000));
    assert(s_an_target_fcw==last_fcw && !s_an_tx_failed);
    s_ook.bits=123;cb_auto(NULL);assert(config_calls==1 && !s_an_send_pending && s_ook.bits==123);
    test_ms+=2000;cb_auto(NULL);assert(s_an_retune_error && config_calls==1 && !s_an_send_pending && s_an_target_fcw==last_fcw);
    assert(strstr(lv_label_get_text(s_lbl_mst),"频率无效"));save(argv[1],"retune_stale");
    fixture_write(11,5,2,1,0,0,1,1);append_meas(2000000,3000000,1);assert(v32_init_file(fixture)==0);
    cb_auto(NULL);assert(s_an_retune_error && config_calls==1); /* CH0 cannot substitute for invalid CH1. */
    fixture_write(12,5,2,1,0,0,1,1);append_meas(2000000,4000000,3);assert(v32_init_file(fixture)==0);
    cb_auto(NULL);assert(s_an_override_fcw==v32_fcw_from_hz(4000000) && !s_an_retune_error);
    analysis_apply(1,0,1);assert(!s_an_override); /* A different user config cancels the snapshot. */
    /* Only a fresh, error-free known reference may hold small FFT jitter. */
    s_an_type=6;s_an_ch=0;s_an_rate=1;s_an_send_pending=0;s_an_tx_failed=0;
    s_an_target_fcw=fixture_fcw=v32_fcw_from_hz(2000000);s_an_have_target=1;s_ook_on=1;s_ook_test=1;s_ook_rate=3;
    fixture_write(20,9,6,0,0,0,1,1);append_meas(2000024,3000000,3);assert(v32_init_file(fixture)==0);
    bpsk_reset(&s_bpsk);s_bpsk.have_frame=1;s_bpsk.cfg=9;s_bpsk.fcw=fixture_fcw;s_bpsk.channel=0;s_bpsk.decim=4096;s_bpsk.baud=1000;
    s_bpsk.locked=s_bpsk.pn_locked=1;s_bpsk.pn_checks=10000;s_bpsk.bits=10287;
    int before_commands=config_calls;cb_auto(NULL);
    assert(config_calls==before_commands && !s_an_send_pending && s_bpsk.bits==10287 && s_bpsk.pn_checks==10000);
    s_bpsk.pn_errors=1;cb_auto(NULL);assert(s_an_send_pending && !s_bpsk.bits);analysis_send_now();assert(config_calls==before_commands+1);
    /* A large new carrier must retune even when the last old frame was good. */
    s_an_target_fcw=fixture_fcw=v32_fcw_from_hz(2000000);s_an_tx_failed=0;
    fixture_write(21,10,6,0,0,0,1,1);append_meas(3000000,3000000,3);assert(v32_init_file(fixture)==0);
    bpsk_reset(&s_bpsk);s_bpsk.have_frame=1;s_bpsk.cfg=10;s_bpsk.fcw=fixture_fcw;s_bpsk.channel=0;s_bpsk.decim=4096;s_bpsk.baud=1000;
    s_bpsk.locked=s_bpsk.pn_locked=1;s_bpsk.pn_checks=10000;s_bpsk.bits=10287;
    cb_auto(NULL);assert(s_an_send_pending && s_an_override_fcw==v32_fcw_from_hz(3000000));
    fixture_fcw=66076420;
    s_an_send_pending=0;s_an_type=0;an_chips_refresh();assert(lv_obj_has_state(s_chip_auto,LV_STATE_DISABLED));
    ui_set_page(UI_PAGE_SCOPE);assert(!lv_obj_has_state(s_chip_auto,LV_STATE_DISABLED));
    assert(strstr(lv_label_get_text(lv_obj_get_child(s_chip_auto,0)),"自动"));record_config=0;
    image_receiver_clear(&s_image);s_image_drawn_revision=UINT32_MAX;
    ui_set_page(UI_PAGE_IMAGE);assert(ui_page_from_name("image")==UI_PAGE_IMAGE);
    assert(!s_image.have_image && strstr(lv_label_get_text(s_image_status),"等待数据"));draw(d);save(argv[1],"image_waiting");
    const char *image_fixture=getenv("V32_TEST_IMAGE_PACKETS");assert(image_fixture);
    FILE *image_file=fopen(image_fixture,"rb");assert(image_file);uint8_t image_packets[272];
    assert(fread(image_packets,1,sizeof image_packets,image_file)==sizeof image_packets);fclose(image_file);
    image_accept_packet(&s_image,image_packets);image_refresh();
    assert(s_image.received==1 && !s_image.complete && image_receiver_pixel(&s_image,0,31)==-1);
    assert(strstr(lv_label_get_text(s_image_status),"接收中"));draw(d);save(argv[1],"image_partial");
    for(int i=1;i<4;i++)image_accept_packet(&s_image,image_packets+68*i);
    image_refresh();assert(s_image.complete && s_image.received==4);
    assert(strstr(lv_label_get_text(s_image_status),"CRC OK"));draw(d);save(argv[1],"image_complete");
    s_image.width=32;s_image.height=16;s_image.revision++;image_refresh();draw(d);save(argv[1],"image_landscape");
    assert(s_image_pixels[8*IMAGE_CANVAS_WIDTH]==0x1082);
    assert(s_image_pixels[16*IMAGE_CANVAS_WIDTH]!=0x1082);
    for(int orientation=0;orientation<2;orientation++){
        unsigned w=orientation?256:128,h=orientation?128:256;uint8_t data[4096],packet[68];
        for(unsigned k=0;k<sizeof data;k++)data[k]=(uint8_t)(k*31+7);
        uint32_t crc=image_crc32(data,sizeof data);cb_image_clear(NULL);
        for(unsigned i=0;i<128;i++){
            memset(packet,0,sizeof packet);memcpy(packet,"V32IMG1!",8);packet[8]=packet[9]=1;
            packet[10]=w;packet[11]=w>>8;packet[12]=h;packet[13]=h>>8;packet[14]=32;
            put(packet+16,123+orientation);put(packet+20,crc);put(packet+24,4096);packet[28]=i;packet[30]=32;
            memcpy(packet+32,data+32*i,32);put(packet+64,image_crc32(packet,64));image_accept_packet(&s_image,packet);
        }
        image_refresh();assert(s_image.complete && s_image.received==128);
        assert(strstr(lv_label_get_text(s_image_info),orientation?"256 x 128":"128 x 256"));draw(d);
        save(argv[1],orientation?"image_256x128":"image_128x256");
    }
    lv_tick_inc(4000);image_refresh();assert(strstr(lv_label_get_text(s_image_info),"NO NEW PACKETS"));draw(d);save(argv[1],"image_stale");
    cb_image_clear(NULL);assert(!s_image.have_image && !s_image.complete);
    ui_set_page(UI_PAGE_SYSTEM);assert(ui_page_from_name("5")==UI_PAGE_SYSTEM);
    /* Invoke the actual reset event: child indices in the scope toolbar are
       groups, not dropdown widgets. A bad cast used to crash the live GUI. */
    const int reset_pages[]={UI_PAGE_SCOPE,UI_PAGE_MOD,UI_PAGE_LOCK,UI_PAGE_IMAGE};
    for(unsigned k=0;k<sizeof reset_pages/sizeof reset_pages[0];k++){
        ui_set_page(reset_pages[k]);S.tb_idx=TIMEBASE_DEF+1;
        cb_reset(NULL);draw(d);
        assert(S.tb_idx==TIMEBASE_DEF && !s_tb_manual && S.running);
    }
    ui_set_page(UI_PAGE_SCOPE);s_cur_on=0;draw(d);save(argv[1],"scope_home");
    assert(lv_obj_has_flag(lv_obj_get_parent(s_lbl_cur_t1),LV_OBJ_FLAG_HIDDEN));
    s_cur_on=1;draw(d);save(argv[1],"scope_cursors");
    assert(!lv_obj_has_flag(lv_obj_get_parent(s_lbl_cur_t1),LV_OBJ_FLAG_HIDDEN));
    assert(lv_obj_has_flag(lv_obj_get_parent(s_lbl_m_vmax),LV_OBJ_FLAG_HIDDEN));
    lv_area_t metric,panel;lv_obj_get_coords(lv_obj_get_parent(s_lbl_cur_dv),&metric);
    lv_obj_get_coords(lv_obj_get_parent(lv_obj_get_parent(s_lbl_cur_dv)),&panel);assert(metric.y2<panel.y2);
    s_cur_on=0;draw(d);cb_fullscreen(NULL);draw(d);save(argv[1],"scope_fullscreen");cb_fullscreen(NULL);
    /* Unchanged geometry must not cause a redraw; style-only changes must. */
    lv_obj_t *probe=plot_create(lv_screen_active(),PLOT_SCOPE);lv_obj_set_size(probe,160,90);
    float trace[8]={.1,.2,.3,.4,.5,.6,.7,.8};
    plot_set_trace(probe,0,trace,8,V32_C_CH1,2);plot_hide_trace(probe,2);
    plot_set_trigger(probe,true,.2f);plot_set_trigger_level(probe,true,.5f);
    plot_set_cursors(probe,0,NULL,NULL,false);lv_obj_update_layout(lv_screen_active());lv_refr_now(d);
    unsigned before=flush_count;
    plot_set_trace(probe,0,trace,8,V32_C_CH1,2);plot_hide_trace(probe,2);
    plot_set_trigger(probe,true,.2f);plot_set_trigger_level(probe,true,.5f);
    plot_set_cursors(probe,0,NULL,NULL,false);plot_set_mode(probe,PLOT_SCOPE);lv_refr_now(d);
    assert(flush_count==before);
    plot_set_trace(probe,0,trace,8,V32_C_CH2,2);lv_refr_now(d);assert(flush_count>before);
    lv_obj_delete(probe);
    /* A valid, constant RAW frame has no AC frequency. Do not report FFT bin1. */
    s_tb_manual=0;fixture_write(200,20,1,0,0,0,1,1);
    FILE *zero_file=fopen(fixture,"r+b");assert(zero_file);uint8_t zero_raw[4140];
    assert(fread(zero_raw,1,sizeof zero_raw,zero_file)==sizeof zero_raw);
    memset(zero_raw+40,0,4096);put(zero_raw+4136,crc(zero_raw,4136));
    rewind(zero_file);assert(fwrite(zero_raw,1,sizeof zero_raw,zero_file)==sizeof zero_raw);fclose(zero_file);
    assert(v32_init_file(fixture)==0);ui_set_page(UI_PAGE_SCOPE);draw(d);
    assert(!strcmp(lv_label_get_text(s_lbl_scope_freq),"—"));
    /* Mismatched RAW/SPEC sample rates must never mix axes or cursor data. */
    S.running=true;fixture_write(210,21,1,0,0,0,1,1);append_spec();assert(v32_init_file(fixture)==0);
    ui_set_page(UI_PAGE_FFT);fft_ch_set(1);draw(d);assert(s_spec_in_use);
    assert(fabsf(s_spec_peak_hz-15625.f)<.1f);
    assert(strstr(lv_label_get_text(s_lbl_fft_res),"244.1Hz"));
    s_fft_cur_on=1;s_fft_cur_x=64.f/2048.f;fft_cursor_tick();
    assert(strstr(lv_label_get_text(s_lbl_fft_cur),"15.625kHz"));
    assert(strstr(lv_label_get_text(s_lbl_fft_cur),"-7.1 dBFS"));save(argv[1],"spectrum_fpga");
    /* Host THD must use exact powers, not the v2 on-wire zero placeholder. */
    fixture_write(215,22,1,0,0,0,1,1);append_harmonic(1,0,0,0);assert(v32_init_file(fixture)==0);
    s_hrm_on=1;s_hrm_baseline_seq=0;draw(d);assert(!strcmp(lv_label_get_text(s_hrm_thd),"CHECK"));
    draw(d);assert(!strcmp(lv_label_get_text(s_hrm_thd),"CHECK"));
    for(int id=100;id<102;id++){fixture_write(215,22,1,0,0,0,1,1);append_harmonic(id,0,0,0);assert(v32_init_file(fixture)==0);draw(d);}
    assert(strstr(lv_label_get_text(s_hrm_thd),"6.325"));
    assert(strstr(lv_label_get_text(s_hrm_orders[1]),"-30.0"));save(argv[1],"spectrum_harmonics");
    hrm_settings(NULL);assert(s_hrm_dialog);draw(d);save(argv[1],"harmonic_settings");hrm_close(NULL);
    fixture_write(216,22,1,0,0,0,1,1);append_harmonic(2,0,30,0);assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(lv_label_get_text(s_hrm_thd),"< NOISE"));
    fixture_write(217,22,1,0,0,0,1,1);append_harmonic(3,1,0,0);assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(lv_label_get_text(s_hrm_thd),"—"));
    test_ms+=1600;lv_tick_inc(1600);assert(v32_init_file(fixture)==0);draw(d);
    assert(strstr(lv_label_get_text(s_hrm_note),"超时"));s_hrm_on=0;
    fixture_write(218,22,1,0,0,0,1,1);append_spec();assert(v32_init_file(fixture)==0);draw(d);
    S.running=false;fft_ch_set(2);draw(d);assert(!s_spec_in_use && s_real_fft_ch==1);
    assert(strstr(lv_label_get_text(s_lbl_fft_res),"63.477kHz"));
    float old_mag[4096];memcpy(old_mag,s_real_mag,sizeof old_mag);cb_win_cycle(NULL);
    assert(memcmp(old_mag,s_real_mag,sizeof old_mag));save(argv[1],"spectrum_local_paused");
    S.running=true;S.win_idx=0;s_fft_cur_on=0;
    fixture_write(211,22,1,0,0,0,1,1);append_lock_meas(3,0);assert(v32_init_file(fixture)==0);
    ui_set_page(UI_PAGE_LOCK);draw(d);assert(strstr(lv_label_get_text(s_lbl_lock_state),"锁定"));
    assert(strstr(lv_label_get_text(s_lbl_lock_phase),"45.00"));save(argv[1],"lock_qualified");
    fixture_write(212,22,1,0,0,0,1,1);append_lock_meas(1,0);assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(lv_label_get_text(s_lbl_lock_phase),"—"));save(argv[1],"lock_single_channel");
    fixture_write(213,22,1,0,0,0,1,1);append_lock_meas(3,1);assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(lv_label_get_text(s_lbl_lock_phase),"—"));
    test_ms+=2000;lv_tick_inc(2000);fixture_write(214,22,1,0,0,0,1,1);append_iq();assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(lv_label_get_text(s_lbl_lock_phase),"—"));assert(strstr(lv_label_get_text(s_lbl_lock_state),"NCO"));save(argv[1],"lock_iq_reference");
    ui_set_page(UI_PAGE_SYSTEM);draw(d);save(argv[1],"system_large");
    cb_scope_help(NULL);assert(s_help_overlay);draw(d);save(argv[1],"scope_help");cb_help_close(NULL);assert(!s_help_overlay);
    /* Scope must remain RAW even when legacy IQ frames arrive. */
    fixture_write(219,22,1,0,0,0,1,1);assert(v32_init_file(fixture)==0);ui_set_page(UI_PAGE_SCOPE);draw(d);
    char raw_axis[80];snprintf(raw_axis,sizeof raw_axis,"%s",lv_label_get_text(lv_obj_get_child(s_chip_tb,0)));
    append_iq();assert(v32_init_file(fixture)==0);draw(d);
    assert(!strcmp(raw_axis,lv_label_get_text(lv_obj_get_child(s_chip_tb,0))));
    lv_area_t grid,widget;assert(plot_get_data_area(s_plot_scope,&grid));lv_obj_get_coords(s_plot_scope,&widget);
    int border=lv_obj_get_style_border_width(s_plot_scope,LV_PART_MAIN);
    assert(grid.x1==widget.x1+80+border && grid.y1==widget.y1+20+border);save(argv[1],"scope_raw_only");
    s_an_type=0;
    const int expected_modes[]={1,2,7,5,3,6,4,0};
    for(unsigned i=0;i<8;i++){cb_an_type(NULL);assert(s_an_type==expected_modes[i]);}
    analysis_apply(4,0,1);assert(s_an_type==4);
    /* Actual math samples, paused edits and rendered third trace, not just chip text. */
    ui_set_page(UI_PAGE_SCOPE);s_trig_mode=0;S.running=false;s_cur_on=0;s_zoom_span=128;s_zoom_start=0;
    S.ch1=S.ch2=false;
    for(int mode=1;mode<=4;mode++){
        s_math_mode=mode;draw(d);
        for(int i=0;i<s_full_n;i++){
            float wanted=scope_math_sample(s_full0[i],s_full1[i],mode);
            assert((isnan(wanted)&&isnan(s_math[i])) || fabsf(s_math[i]-wanted)<1e-5f);
        }
        assert(!lv_obj_has_flag(s_math_note,LV_OBJ_FLAG_HIDDEN));
    }
    static float saved_divisor[WAVE_MAX_FRAME];memcpy(saved_divisor,s_full1,s_full_n*sizeof(float));
    memset(s_full1,0,s_full_n*sizeof(float));s_math_mode=4;draw(d);
    assert(strstr(lv_label_get_text(s_math_note),"DIV ZERO"));memcpy(s_full1,saved_divisor,s_full_n*sizeof(float));
    s_math_mode=3;draw(d);assert(plot_get_data_area(s_plot_scope,&grid));
    int green=0;for(int y=grid.y1+30;y<grid.y2-10;y++)for(int x=grid.x1+5;x<grid.x2-5;x++)if((pixels[y*1024+x]&0xffffff)==0x80e4b3)green++;
    assert(green>30);save(argv[1],"math_product_only");S.ch1=S.ch2=true;s_math_mode=0;
    s_math_mode=1;s_zoom_span=s_full_n*2;draw(d);assert(strstr(lv_label_get_text(s_math_note),"FRAME ONLY"));s_zoom_span=128;s_math_mode=0;
    assert(isnan(scope_math_sample(5,0,4)) && scope_math_sample(5,2,4)==2.5f);
    /* A complete pre/post trigger must align to 20%, not clamp first crossing. */
    float pulse[512];for(int i=0;i<512;i++)pulse[i]=(i%100)<50?-50:50;
    int start=scope_trigger_start(pulse,512,200,0,0);assert(start==10 && pulse[start+40-1]<0 && pulse[start+40]>=0);
    assert(scope_trigger_start(pulse,512,200,100,0)<0);
    S.running=true;s_trig_mode=2;s_trig_counts=10000;
    uint32_t held=s_real_fid;fixture_write(221,23,1,0,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);
    assert(s_trig_wait && s_real_fid==held);
    s_trig_counts=0;cb_single(NULL);
    fixture_write(222,23,1,0,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);
    assert(s_single_done && !s_single_armed && !S.running && s_trig_hit);held=s_real_fid;save(argv[1],"single_hold");
    fixture_write(223,23,1,0,0,0,1,1);assert(v32_init_file(fixture)==0);draw(d);assert(s_real_fid==held);
    cb_run(NULL);assert(S.running && !s_single_done && !s_trig_mode);
    s_single_done=1;S.running=false;cb_auto(NULL);assert(S.running && !s_single_done && !s_single_armed);s_trig_mode=0;
    /* Drag the chosen physical channel without changing ADC values. */
    S.running=false;fake_touch=1;s_move_channel=0;s_vertical_offset[0]=s_vertical_offset[1]=0;
    fake_touch_x=(grid.x1+grid.x2)/2;fake_touch_y=(grid.y1+grid.y2)/2;
    lv_obj_send_event(s_plot_scope,LV_EVENT_PRESSED,NULL);fake_touch_y-=40;lv_obj_send_event(s_plot_scope,LV_EVENT_PRESSING,NULL);lv_obj_send_event(s_plot_scope,LV_EVENT_RELEASED,NULL);
    assert(s_vertical_offset[0]==0 && s_vertical_offset[1]==0);save(argv[1],"scope_vertical_pan_disabled");
    assert(s_vertical_offset[0]==0 && s_vertical_offset[1]==0);
    /* Cursor remains attached even when dragged past the other cursor. */
    s_cur_on=1;fake_touch_x=grid.x1+(int)(.3f*(grid.x2-grid.x1));fake_touch_y=grid.y1+30;
    lv_obj_send_event(s_plot_scope,LV_EVENT_PRESSED,NULL);fake_touch_x=grid.x1+(int)(.85f*(grid.x2-grid.x1));
    lv_obj_send_event(s_plot_scope,LV_EVENT_PRESSING,NULL);lv_obj_send_event(s_plot_scope,LV_EVENT_RELEASED,NULL);assert(s_cur_x[0]>.8f && s_cur_x[1]<.8f);s_cur_on=0;
    lv_indev_t *touch=lv_indev_create();assert(touch);ui_set_page(UI_PAGE_FFT);lv_obj_update_layout(lv_screen_active());assert(plot_get_data_area(s_plot_fft,&grid));
    fake_touch_x=grid.x1+(grid.x2-grid.x1)/4;fake_touch_y=(grid.y1+grid.y2)/2;lv_obj_send_event(s_plot_fft,LV_EVENT_PRESSED,NULL);float cursor_start=s_fft_cur_x;
    fake_touch_x=grid.x1+3*(grid.x2-grid.x1)/4;lv_obj_send_event(s_plot_fft,LV_EVENT_PRESSING,NULL);assert(s_fft_cur_x>cursor_start+.4f);save(argv[1],"spectrum_drag");fake_touch=0;
    /* Exercise the historical overrun: full 4096-point RAW while math is ON. */
    uint8_t full_raw[44+4096*4]={0};memcpy(full_raw,"V32CAP3!",8);put(full_raw+8,3);put(full_raw+12,++raw_id);
    put(full_raw+16,65000000);put(full_raw+20,1);put(full_raw+24,4096);put(full_raw+32,raw_id*1000000u);
    for(int i=0;i<4096;i++){int16_t a=(int16_t)(200*sin(i*.1)),b=(int16_t)(100*cos(i*.1));memcpy(full_raw+40+4*i,&a,2);memcpy(full_raw+42+4*i,&b,2);}
    put(full_raw+sizeof full_raw-4,crc(full_raw,sizeof full_raw-4));FILE *full_file=fopen(fixture,"wb");assert(full_file);assert(fwrite(full_raw,1,sizeof full_raw,full_file)==sizeof full_raw);fclose(full_file);
    assert(v32_init_file(fixture)==0);S.running=true;s_trig_mode=0;s_math_mode=3;ui_set_page(UI_PAGE_SCOPE);draw(d);assert(s_full_n==4096);
    assert(s_math[4095]==s_full0[4095]*s_full1[4095]);
    /* Long timebases and manual volts/div must share exactly the same map. */
    S.running=false;s_math_mode=0;s_trig_mode=0;s_cur_on=0;s_move_channel=0;s_vertical_offset[0]=s_vertical_offset[1]=0;
    s_vdiv_mv=5000;s_vdiv_idx=1;s_gain=.05f;s_zoom_span=s_full_n*2;
    s_roll_len=4;for(int i=0;i<4;i++)s_roll0[i]=s_roll1[i]=(i&1)?100:-100;
    draw(d);assert(s_gain==1.f);assert(fabsf(V32_ADC_MV_PER_LSB/(s_real_sc*V32_GRID_Y)-5000.f)<.01f);
    float roll0[8],roll1[8];map_roll(roll0,roll1,8);
    float lower=1,upper=0;for(int i=0;i<8;i++){if(roll0[i]<lower)lower=roll0[i];if(roll0[i]>upper)upper=roll0[i];}
    assert(upper-lower<.02f && upper-lower>.01f);save(argv[1],"long_manual_5v");
    ui_pinch_gain(.001f);assert(s_vdiv_mv==5000 && s_gain==1);ui_pinch_gain(100000);assert(s_vdiv_mv==10 && s_gain==1);
    s_vdiv_mv=100;s_vdiv_idx=7;s_zoom_span=128;s_cur_on=1;s_vertical_offset[0]=.1f;s_vertical_offset[1]=-.1f;s_cur_y[0]=.4f;s_cur_y[1]=.6f;draw(d);
    assert(strstr(lv_label_get_text(s_lbl_cur_v1),"0mV"));
    char measured_vpp[80];snprintf(measured_vpp,sizeof measured_vpp,"%s",lv_label_get_text(s_lbl_v1));
    cb_move_channel(NULL);draw(d);assert(strstr(lv_label_get_text(s_lbl_cur_v2),"0mV"));
    assert(strstr(lv_label_get_text(lv_obj_get_child(lv_obj_get_parent(s_lbl_cur_v2),0)),"CH2"));
    assert(!strcmp(measured_vpp,lv_label_get_text(s_lbl_v1)));save(argv[1],"channel2_voltage_reference");
    unlink(fixture);puts("MODULATION_VIEW_PASS time_axes raw_stale independent_views local_controls gaps flags IQ FM pause exact_RAW_rate glyphs image_empty_partial_CRC_complete_stale scope_layout_cached_redraw_style_change harmonics scope_RAW_only supported_modes math4096 drag trigger_single unified_roll_scale voltage_reference");return 0;
}
