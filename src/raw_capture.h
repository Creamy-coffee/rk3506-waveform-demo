#ifndef V32_RAW_CAPTURE_H
#define V32_RAW_CAPTURE_H
/* Optional bounded diagnostic copy. Only the writer touches disk; the UART
 * reader never waits for disk I/O. Overflow stops at a prefix, never splices. */
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#define RAW_CAPTURE_QUEUE (1024u*1024u)
typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_t thread;
    uint8_t *queue;
    size_t head, count, accepted, limit, written;
    int fd, accepting, overflow, error, limited;
} raw_capture_t;

static void *raw_capture_writer(void *arg)
{
    raw_capture_t *c=arg;
    uint8_t block[16384];
    for(;;) {
        pthread_mutex_lock(&c->lock);
        while(!c->count && c->accepting)pthread_cond_wait(&c->ready,&c->lock);
        size_t n=c->count<sizeof block?c->count:sizeof block;
        if(!n){pthread_mutex_unlock(&c->lock);break;}
        size_t first=RAW_CAPTURE_QUEUE-c->head;if(first>n)first=n;
        memcpy(block,c->queue+c->head,first);memcpy(block+first,c->queue,n-first);
        c->head=(c->head+n)%RAW_CAPTURE_QUEUE;c->count-=n;
        pthread_mutex_unlock(&c->lock);
        size_t at=0;
        while(at<n) {
            ssize_t w=write(c->fd,block+at,n-at);
            if(w<0 && errno==EINTR)continue;
            if(w<=0) {
                pthread_mutex_lock(&c->lock);
                c->error=w<0?errno:EIO;c->accepting=0;c->count=0;
                pthread_mutex_unlock(&c->lock);
                goto done;
            }
            at+=(size_t)w;c->written+=(size_t)w;
        }
    }
done:
    if(close(c->fd)<0 && !c->error)c->error=errno;
    return NULL;
}

static int raw_capture_start(raw_capture_t *c,const char *path,size_t limit)
{
    memset(c,0,sizeof *c);
    if(!path || !*path)return 0;
    if(!limit){errno=EINVAL;return -1;}
    /* Never overwrite an earlier capture or follow an existing destination. */
    c->fd=open(path,O_WRONLY|O_CREAT|O_EXCL,0600);
    if(c->fd<0)return -1;
    c->queue=malloc(RAW_CAPTURE_QUEUE);
    if(!c->queue){close(c->fd);errno=ENOMEM;return -1;}
    int e=pthread_mutex_init(&c->lock,NULL);
    if(e){free(c->queue);c->queue=NULL;close(c->fd);errno=e;return -1;}
    e=pthread_cond_init(&c->ready,NULL);
    if(e){pthread_mutex_destroy(&c->lock);free(c->queue);c->queue=NULL;close(c->fd);errno=e;return -1;}
    c->limit=limit;c->accepting=1;
    e=pthread_create(&c->thread,NULL,raw_capture_writer,c);
    if(e){pthread_cond_destroy(&c->ready);pthread_mutex_destroy(&c->lock);free(c->queue);c->queue=NULL;close(c->fd);errno=e;return -1;}
    return 1;
}

static void raw_capture_feed(raw_capture_t *c,const void *data,size_t n)
{
    if(!c->queue || !n)return;
    pthread_mutex_lock(&c->lock);
    if(!c->accepting){pthread_mutex_unlock(&c->lock);return;}
    size_t remaining=c->limit-c->accepted;
    if(n>remaining)n=remaining;
    if(n>RAW_CAPTURE_QUEUE-c->count) {
        c->overflow=1;c->accepting=0;
    } else {
        size_t tail=(c->head+c->count)%RAW_CAPTURE_QUEUE;
        size_t first=RAW_CAPTURE_QUEUE-tail;if(first>n)first=n;
        memcpy(c->queue+tail,data,first);memcpy(c->queue,(const uint8_t *)data+first,n-first);
        c->count+=n;c->accepted+=n;
        if(c->accepted==c->limit){c->limited=1;c->accepting=0;}
    }
    pthread_cond_signal(&c->ready);
    pthread_mutex_unlock(&c->lock);
}

static void raw_capture_finish(raw_capture_t *c)
{
    if(!c->queue)return;
    pthread_mutex_lock(&c->lock);c->accepting=0;pthread_cond_signal(&c->ready);pthread_mutex_unlock(&c->lock);
    pthread_join(c->thread,NULL);
    fprintf(stderr,"[rawcap] finished accepted=%zu written=%zu overflow=%d error=%d limit=%d\n",c->accepted,c->written,c->overflow,c->error,c->limited);
    pthread_cond_destroy(&c->ready);pthread_mutex_destroy(&c->lock);free(c->queue);c->queue=NULL;
}
#endif
