#ifndef GAUSS_DUAL_CORE_H
#define GAUSS_DUAL_CORE_H

#include <stdint.h>
#define MAX_JDIM 320

typedef void* TaskHandle_t;   // opaque, OK ici

typedef struct {
    uint8_t  *out;
    const uint8_t *in;
    int idim;
    int jdim;
    int i_start;
    int i_end;
    uint8_t (*row_buf)[MAX_JDIM];
    TaskHandle_t notify_task;
} gaussian_task_ctx_t;

/* API publique */
void gaussian_dual_core_init();
//void gaussian_task_core1(void *arg);
void gaussian_blur_15x15_dual_core(uint8_t *out, const uint8_t *in, int idim, int jdim);

#endif
