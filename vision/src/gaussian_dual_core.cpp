#include "gaussian_dual_core.h"
#include <image_process.h>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#define KERNEL_SIZE 15

/* ============================================================
 * Statique global (DRAM / PSRAM selon ton implémentation)
 * ============================================================ */

static TaskHandle_t task_core1 = nullptr;
static gaussian_task_ctx_t ctx_core1;

/* Buffers de lignes séparés (pas de conflit inter-core) */
static uint8_t row_buf_0[KERNEL_SIZE][MAX_JDIM];
static uint8_t row_buf_1[KERNEL_SIZE][MAX_JDIM];

/* ============================================================
 * Task Core 1
 * ============================================================ */
static void gaussian_task_core1(void *arg)
{
    gaussian_task_ctx_t *ctx = (gaussian_task_ctx_t *)arg;

    /* Attente obligatoire avant toute utilisation */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;) {
        gaussian_blur_15x15_stream_8bits_band(
            ctx->out,
            ctx->in,
            ctx->idim,
            ctx->jdim,
            ctx->i_start,
            ctx->i_end,
            ctx->row_buf
        );

        /* Signal fin Core 1 → Core 0 */
        xTaskNotifyGive(ctx->notify_task);

        /* Attente prochain job */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/* ============================================================
 * Initialisation dual-core (appelée UNE SEULE FOIS)
 * ============================================================ */
void gaussian_dual_core_init()
{
    if (task_core1 != nullptr) {
        return; // déjà initialisé
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        gaussian_task_core1,
        "GAUSS_CORE1",
        8192,                 // stack SAFE pour traitement image
        &ctx_core1,
        1,
        &task_core1,
        1                      // Core 1
    );

    if (ok != pdPASS) {
        while (1) {
            // échec création task → blocage volontaire
        }
    }
}

/* ============================================================
 * Fonction publique dual-core
 * ============================================================ */
void gaussian_blur_15x15_dual_core(
    uint8_t *out,
    const uint8_t *in,
    int idim,
    int jdim
)
{
    const int mid = idim / 2;

    /* --- Préparer contexte Core 1 --- */
    ctx_core1.out         = out;
    ctx_core1.in          = in;
    ctx_core1.idim        = idim;
    ctx_core1.jdim        = jdim;
    ctx_core1.i_start     = mid;
    ctx_core1.i_end       = idim - KERNEL_SIZE;
    ctx_core1.row_buf     = row_buf_1;
    ctx_core1.notify_task = xTaskGetCurrentTaskHandle();

    /* --- Lancer Core 1 --- */
    xTaskNotifyGive(task_core1);

    /* --- Core 0 travaille en parallèle --- */
    gaussian_blur_15x15_stream_8bits_band(
        out,
        in,
        idim,
        jdim,
        0,
        mid - 1,
        row_buf_0
    );

    /* --- Attente fin Core 1 --- */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* --- Nettoyage bas d’image --- */
    for (int i = idim - KERNEL_SIZE + 1; i < idim; i++) {
        memset(&out[i * jdim], 0, jdim);
    }
}
