/*
 * Copyright 2015 - 2017 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _HAL_H264E_RC_H_
#define _HAL_H264E_RC_H_

#include <stdio.h>
#include <string.h>

#include "hal_h264e_com.h"

extern const RK_S32 h264_q_step[];

MPP_RET h264e_vpu_update_hw_cfg(H264eHalContext *ctx, HalEncTask *task,
                                H264eHwCfg *hw_cfg);
MPP_RET h264e_vpu_update_buffers(H264eHalContext *ctx, H264eHwCfg *hw_cfg);
MPP_RET h264e_vpu_mad_threshold(H264eHwCfg *hw_cfg, MppLinReg *mad, RK_U32 madCount);

void h264e_vpu_tsvc_rc_init(H264eHalTsvcRc *svc_rc, RK_S32 max_layer_id);
void h264e_vpu_tsvcrc_update(H264eHalTsvcRc *tsvc_rc, RK_S32 pic_bits,  RK_S32 avg_qp);

void h264e_vpu_update_result(H264eHwCfg *hw_cfg, RK_S32 pic_bits, RK_S32 target_bits);
void h264e_check_reencode(void *hal, HalTaskInfo *task, void *reg_out,
                          MPP_RET (*send)(void* hal, RK_U32 *reg, RK_S32 d_qp),
                          MPP_RET (*feedback)(h264e_feedback* fb, void * reg));

#endif
