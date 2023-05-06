#include "../../Utils/ring_fifo.h"

#include <string.h>

#define min(a, b)       ((a) > (b) ? (b) : (a))
#define fifo_max_depth  (0xffffffff >> 1)

/* 环形缓冲区结构 */
struct ring_fifo_t {
    volatile uint32_t   head;           /* 消费者指针 */
    volatile uint32_t   tail;           /* 生产者指针 */

    uint32_t            size;           /* 缓冲区的大小 */
    uint32_t            mask;           /* 缓冲区的大小掩码 */

    void                *buf;           /* 缓冲区指针 */
    uint32_t            is_dynamic;     /* 是否使用了动态内存 */

    enum ring_fifo_type type;           /* fifo的类型 */
};

static inline uint32_t is_pow_of_2(uint32_t n)
{
    return (0 != n) && (0 == (n & (n - 1)));
}

// 返回比x大的最小的2的N次方数
static inline uint32_t pow2gt(uint32_t x)
{
    --x;

    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;

    return x + 1;
}

struct ring_fifo_t * ring_fifo_init(void *buf, uint32_t size, enum ring_fifo_type type)
{
    struct ring_fifo_t *ring;

    if((NULL != buf) && (0 == is_pow_of_2(size)))
    {
        return NULL;
    }

    ring = malloc(sizeof(struct ring_fifo_t));
    if(NULL == ring)
    {
        return NULL;
    }

    if(size > fifo_max_depth)
    {
        size = fifo_max_depth;
    }

    size = pow2gt(size);

    if(NULL == buf)
    {
        ring->buf = malloc(size);
        if(NULL == ring->buf)
        {
            free(ring);

            return NULL;
        }
        ring->is_dynamic = 1;
    }
    else
    {
        ring->buf = buf;
        ring->is_dynamic = 0;
    }

    ring->head = ring->tail = 0xfffff000;
    ring->size = size;
    ring->mask = size - 1;
    ring->type = type;

    return ring;
}

void ring_fifo_destroy(struct ring_fifo_t *ring)
{
    if(0 != ring->is_dynamic)
    {
        free(ring->buf);
        ring->buf = NULL;
    }

    free(ring);
}

uint32_t ring_fifo_write(struct ring_fifo_t *ring, const void *buf, uint32_t len)
{
    uint32_t wlen;
    uint32_t unused;
    uint32_t off, l;
    uint32_t frame_off;

    unused = ring->size - (ring->tail - ring->head);
    switch(ring->type)
    {
        case RF_TYPE_FRAME:
        /* 如果不能存下此帧，丢弃 */
        if(len + sizeof(uint32_t) > unused) { return 0; }
        wlen = len;
        frame_off = sizeof(uint32_t);
        /* 写入帧长 */
        *(uint32_t *)(ring->buf + (ring->tail & ring->mask)) = len;
        break;
        default: /* RF_TYPE_STREAM */
        if(0 == unused) { return 0; }
        wlen = min(len, unused);
        frame_off = 0;
        break;
    }

    /* 计算写入位置 */
    off = (ring->tail + frame_off) & ring->mask;
    l = min(wlen, ring->size - off);
    memcpy(ring->buf + off, buf, l);
    memcpy(ring->buf, buf + l, wlen - l);

    ring->tail += wlen + frame_off;

    return wlen;
}

uint32_t ring_fifo_read(struct ring_fifo_t *ring, void *buf, uint32_t len)
{

    uint32_t rlen;
    uint32_t used;
    uint32_t off, l;
    uint32_t frame_off;

    used = ring->tail - ring->head;
    if(0 == used) { return 0; }
    switch(ring->type)
    {
        case RF_TYPE_FRAME:
        /* 读取帧长 */
        rlen = *(uint32_t *)(ring->buf + (ring->head & ring->mask));
        /* 给定的缓冲区小于要读出的帧长 */
        if(len < rlen) { return 0; }
        frame_off = sizeof(uint32_t);
        break;
        default:  /* RF_TYPE_STREAM */
        rlen = min(len, used);
        frame_off = 0;
        break;
    }

    /* 计算读取位置 */
    off = (ring->head + frame_off) & ring->mask;
    l = min(rlen, ring->size - off);
    memcpy(buf, ring->buf + off, l);
    memcpy(buf + l, ring->buf, rlen - l);

    ring->head += rlen + frame_off;

    return rlen;
}

uint32_t ring_fifo_is_full(struct ring_fifo_t *ring)
{
    return ring->size == (ring->tail - ring->head);
}

uint32_t ring_fifo_is_empty(struct ring_fifo_t *ring)
{
    return ring->tail == ring->head;
}

uint32_t ring_fifo_avail(struct ring_fifo_t *ring)
{
    return ring->size - (ring->tail - ring->head);
}

uint32_t ring_fifo_count(struct ring_fifo_t *ring)
{
    return ring->tail - ring->head;
}
