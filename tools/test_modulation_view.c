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
static void flush_cb(lv_display_t *d,const lv_area_t *a,uint8_t *p){(void)a;(void)p;lv_display_flush_ready(d);}
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
static void draw(lv_display_t *d){test_ms+=40;lv_tick_inc(40);ui_tick();lv_obj_update_layout(lv_screen_active());lv_obj_invalidate(lv_screen_active());lv_refr_now(d);}
static void save(const char *dir,const char *name){char p[1024];snprintf(p,sizeof p,"%s/%s.ppm",dir,name);FILE *f=fopen(p,"wb");assert(f);fprintf(f,"P6\n1024 600\n255\n");for(int i=0;i<1024*600;i++){uint8_t b[3]={pixels[i]>>16,pixels[i]>>8,pixels[i]};fwrite(b,1,3,f);}fclose(f);}
static unsigned next_cp(const unsigned char **p){unsigned c=*(*p)++;if(c<128)return c;unsigned n=(c&0xe0)==0xc0?1:(c&0xf0)==0xe0?2:3;c&=(1u<<(6-n))-1;while(n--)c=(c<<6)|(*(*p)++&63);return c;}
static void glyphs(void){
    const char *text="波形原始包络窗口等待有效数据分析历史相对频偏峰值码中心载波参考实测○● BPSK PN7 待定判决比对中匹配检出误码有无效位对频频率无效/↺⊞图片接收中图已复位";
    const lv_font_t *fonts[]={&lv_font_v32_12,&lv_font_v32_16,&lv_font_v32_18,&lv_font_v32_20,&lv_font_v32_12_bold,&lv_font_v32_16_bold,&lv_font_v32_18_bold,&lv_font_v32_20_bold};
    for(unsigned k=0;k<sizeof fonts/sizeof fonts[0];k++){
        const unsigned char *p=(const unsigned char *)text;
        while(*p){unsigned cp=next_cp(&p);lv_font_glyph_dsc_t g;int present=fonts[k]->get_glyph_dsc(fonts[k],&g,cp,0);if(!present)fprintf(stderr,"MISSING_GLYPH font=%u U+%04X\n",k,cp);assert(present);}
    }
}

int main(int argc,char **argv)
{
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
    lv_tick_inc(4000);image_refresh();assert(strstr(lv_label_get_text(s_image_info),"NO NEW PACKETS"));draw(d);save(argv[1],"image_stale");
    cb_image_clear(NULL);assert(!s_image.have_image && !s_image.complete);
    ui_set_page(UI_PAGE_SYSTEM);assert(ui_page_from_name("5")==UI_PAGE_SYSTEM);
    unlink(fixture);puts("MODULATION_VIEW_PASS time_axes raw_stale independent_views local_controls gaps flags IQ FM pause exact_RAW_rate glyphs image_empty_partial_CRC_complete_stale");return 0;
}
