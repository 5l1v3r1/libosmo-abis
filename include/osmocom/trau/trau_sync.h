#pragma once
#include <osmocom/core/bits.h>
#include <osmocom/core/fsm.h>

typedef void (*frame_out_cb_t)(void *user_data, const ubit_t *bits, unsigned int num_bits);

struct osmo_fsm_inst *
osmo_trau_sync_alloc(void *ctx, const char *name, frame_out_cb_t frame_out_cb, void *user_data);

void osmo_trau_sync_rx_ubits(struct osmo_fsm_inst *fi, const ubit_t *bits, size_t n_bits);
