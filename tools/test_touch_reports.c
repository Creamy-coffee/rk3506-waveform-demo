#define _DEFAULT_SOURCE
#include <assert.h>
#include "../src/port.c"
void ui_pinch_zoom(float r,float x){(void)r;(void)x;}
void ui_pinch_gain(float r){(void)r;}
static void emit(int fd,int type,int code,int value){struct input_event e={0};e.type=type;e.code=code;e.value=value;assert(write(fd,&e,sizeof e)==sizeof e);}
int main(void){
    lv_init();lv_display_t *display=lv_display_create(1024,600);assert(display);
    lv_indev_t *indev=lv_indev_create();assert(indev);
    for(int protocol=0;protocol<2;protocol++){
        int f[2];assert(pipe(f)==0);fcntl(f[0],F_SETFL,O_NONBLOCK);
        touch_t t={0};t.fd=f[0];t.is_mt=1;t.slot_mode=protocol;t.hres=t.maxx=1024;t.vres=t.maxy=600;t.pinch_axis=-1;
        lv_indev_set_user_data(indev,&t);
        if(protocol)emit(f[1],EV_ABS,ABS_MT_SLOT,0);
        emit(f[1],EV_ABS,ABS_MT_TRACKING_ID,14);emit(f[1],EV_ABS,ABS_MT_POSITION_X,222);emit(f[1],EV_ABS,ABS_MT_POSITION_Y,572);
        if(!protocol)emit(f[1],EV_SYN,SYN_MT_REPORT,0);
        emit(f[1],EV_SYN,SYN_REPORT,0);
        if(protocol)emit(f[1],EV_ABS,ABS_MT_TRACKING_ID,-1);
        emit(f[1],EV_SYN,SYN_REPORT,0);
        /* Both reports were queued BEFORE this first callback (slow render). */
        lv_indev_data_t d={0};touch_read_cb(indev,&d);assert(d.state==LV_INDEV_STATE_PRESSED && d.continue_reading);assert(d.point.x==222 && d.point.y==572);
        touch_read_cb(indev,&d);assert(d.state==LV_INDEV_STATE_RELEASED && d.continue_reading);
        assert(d.point.x==222 && d.point.y==572);
        touch_read_cb(indev,&d);assert(d.state==LV_INDEV_STATE_RELEASED && !d.continue_reading);
        close(f[0]);close(f[1]);
    }
    puts("TOUCH_REPORTS_PASS queued press/release protocol A and B");return 0;
}
