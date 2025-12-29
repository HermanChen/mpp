/*
 * Copyright (C) 2016 The FFmpeg project
 * Copyright (c) 2016 Rockchip Electronics Co., Ltd.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef VPX_RAC_H
#define VPX_RAC_H

#include <stdint.h>
#include <stdlib.h>

#include "mpp_common.h"

#define DECLARE_ALIGNED(n,t,v)      t v

typedef struct Vpxmv {
    DECLARE_ALIGNED(4, int16_t, x);
    rk_s16 y;
} Vpxmv;

typedef struct VpxRangeCoder {
    rk_s32 high;
    rk_s32 bits; /* stored negated (i.e. negative "bits" is a positive number of
                 bits left) in order to eliminate a negate in cache refilling */
    const rk_u8 *buffer;
    const rk_u8 *end;
    rk_u32 code_word;
} VpxRangeCoder;

/**
 * vp56 specific range coder implementation
 */

extern const uint8_t vpx_norm_shift[256];
void vpx_init_range_decoder(VpxRangeCoder *c, const uint8_t *buf, int buf_size);
rk_u32 vpx_rac_renorm(VpxRangeCoder *c);
rk_s32 vpx_rac_get_prob(VpxRangeCoder *c, uint8_t prob);
rk_s32 vpx_rac_get_prob_branchy(VpxRangeCoder *c, int prob);
// rounding is different than vpx_rac_get, is vpx_rac_get wrong?
rk_s32 vpx_rac_get(VpxRangeCoder *c);
rk_s32 vpx_rac_get_uint(VpxRangeCoder *c, int bits);

#endif /* VPX_RAC_H */
