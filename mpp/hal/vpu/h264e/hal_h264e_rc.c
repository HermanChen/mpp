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

#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "mpp_mem.h"
#include "mpp_common.h"

#include "hal_h264e_com.h"
#include "hal_h264e_vepu.h"
#include "hal_h264e_rc.h"
#include "hal_h264e_vpu_tbl.h"

const RK_S32 h264_q_step[] = {
    3,   3,   3,   4,   4,   5,   5,   6,   7,   7,
    8,   9,   10,  11,  13,  14,  16,  18,  20,  23,
    25,  28,  32,  36,  40,  45,  51,  57,  64,  72,
    80,  90,  101, 114, 128, 144, 160, 180, 203, 228,
    256, 288, 320, 360, 405, 456, 513, 577, 640, 720,
    810, 896
};

static RK_S32 find_best_qp(MppLinReg *ctx, MppEncH264Cfg *codec,
                           RK_S32 qp_start, RK_S32 bits)
{
    RK_S32 qp = qp_start;
    RK_S32 qp_best = qp_start;
    RK_S32 qp_min = codec->qp_min;
    RK_S32 qp_max = codec->qp_max;
    RK_S32 diff_best = INT_MAX;

    if (ctx->a == 0 && ctx->b == 0)
        return qp_best;

    if (bits <= 0) {
        qp_best = mpp_clip(qp_best + codec->qp_max_step, qp_min, qp_max);
    } else {
        do {
            RK_S32 est_bits = mpp_quadreg_calc(ctx, h264_q_step[qp]);
            RK_S32 diff = est_bits - bits;
            if (MPP_ABS(diff) < MPP_ABS(diff_best)) {
                diff_best = MPP_ABS(diff);
                qp_best = qp;
                if (diff > 0)
                    qp++;
                else
                    qp--;
            } else
                break;
        } while (qp <= qp_max && qp >= qp_min);
    }

    return qp_best;
}

#define WORD_CNT_MAX      65535

void h264e_vpu_rc_calIfrmqp(H264eHalContext *ctx, RcSyntax *rc_syn, H264eHwCfg *hw_cfg)
{
    int intraQpDelta = -3;
    MppEncCfgSet *cfg = ctx->cfg;
    MppEncH264Cfg *codec = &cfg->codec.h264;

    hw_cfg->frame_num = 0;
    if (ctx->frame_cnt > 0) {
        hw_cfg->qp = mpp_data_avg(ctx->qp_p, -1, 1, 1);
        hw_cfg->qp += intraQpDelta;
    }
    /*
     * Previous frame is inter then intra frame can not
     * have a big qp step between these two frames
     */
    if (hw_cfg->pre_frame_type  == H264E_VPU_FRAME_P)
        hw_cfg->qp = mpp_clip(hw_cfg->qp, hw_cfg->qp_prev - 4,
                              42);
    else
        hw_cfg->qp = find_best_qp(ctx->intra_qs, codec, hw_cfg->qp_prev,
                                  rc_syn->bit_target);
}

void h264e_vpu_rc_calPfrmqp(H264eHalContext *ctx, H264eHwCfg *hw_cfg)
{
    RK_S32 ratio, ratio2;
    RK_S32 PrePicBits = hw_cfg->pre_pic_bits;
    RK_S32 PreMeanQp =  mpp_data_avg(ctx->qp_p, -1, 1, 1);

    if (100 * PrePicBits < 105 * hw_cfg->pre_target_bit) {
        if (100 * PrePicBits > 90 * hw_cfg->pre_target_bit) {
            hw_cfg->qp = hw_cfg->qp_prev;
        } else {
            ratio = (hw_cfg->pre_target_bit - PrePicBits) * 100 / hw_cfg->pre_target_bit;
            if (ratio >= 40) {
                hw_cfg->qp  = hw_cfg->qp_prev - 3;
            } else if ((ratio > 20) && (ratio < 40)) {
                hw_cfg->qp  = hw_cfg->qp_prev - 2;
            } else {
                hw_cfg->qp  = hw_cfg->qp_prev - 1;
            }
            if (PreMeanQp > 0)
                hw_cfg->qp  = MPP_MIN(PreMeanQp, hw_cfg->qp);
        }
    } else {
        ratio2 = (PrePicBits - hw_cfg->pre_target_bit) * 100 / hw_cfg->pre_target_bit;
        ratio2 = MPP_MIN(ratio2, 200);
        hw_cfg->qp = hw_cfg->qp_prev + (ratio2 * 5) / 100 + 1;
        hw_cfg->qp = MPP_MIN(hw_cfg->qp, 51);
    }

    if (PreMeanQp > 0 && (PreMeanQp - hw_cfg->qp_prev) > 5) {
        hw_cfg->qp = hw_cfg->qp_prev + 1;
    }
    h264e_hal_dbg(H264E_DBG_RC, "calPfrmQp %d\n", hw_cfg->qp);
}

void h264e_vpu_tsvc_rc_calqp(H264eHalContext *ctx, RcSyntax *rc_syn, H264eHwCfg *hw_cfg)
{
    H264eHalTsvcRc *svc_rc = &ctx->tsvc_rc;
    svc_rc->uiTemporalId = svc_rc->iTlOfFrames[svc_rc->iFrameCodedInVGop];
    if (rc_syn->type == INTRA_FRAME) {
        svc_rc->iFrameCodedInVGop = 0;
        hw_cfg->frame_type = H264E_VPU_FRAME_I;
        h264e_vpu_rc_calIfrmqp(ctx, rc_syn, hw_cfg);
    } else {
        RK_S32 iLastIdxCodecInVGop, iTlLast, iDeltaQpTemporal;
        hw_cfg->frame_type = H264E_VPU_FRAME_P;
        if (svc_rc->uiTemporalId == 0) {
            if (svc_rc->iGopIndexInVGop == svc_rc->iGopNumberInVGop) {
                svc_rc->iGopIndexInVGop = 0;
                svc_rc->iFrameCodedInVGop = 0;
            }
            svc_rc->iGopIndexInVGop++;
        }
        h264e_vpu_rc_calPfrmqp(ctx, hw_cfg);
        iLastIdxCodecInVGop = svc_rc->iFrameCodedInVGop - 1;
        if (iLastIdxCodecInVGop < 0)
            iLastIdxCodecInVGop += VGOP_SIZE;
        iTlLast = svc_rc->iTlOfFrames[iLastIdxCodecInVGop];
        iDeltaQpTemporal = svc_rc->uiTemporalId - iTlLast;
        if (0 == iTlLast && svc_rc->uiTemporalId > 0)
            iDeltaQpTemporal += 4;
        else if (0 == svc_rc->uiTemporalId && iTlLast > 0)
            iDeltaQpTemporal -= 3;
        hw_cfg->qp += iDeltaQpTemporal;
        if (svc_rc->iQp[svc_rc->iFrameCodedInVGop] > 0)
            hw_cfg->qp = (hw_cfg->qp + svc_rc->iQp[svc_rc->iFrameCodedInVGop]) >> 1;
        h264e_hal_dbg(H264E_DBG_RC, "deltaQp %d, iQp[%d]=%d, iTlLast %d\n", iDeltaQpTemporal, svc_rc->iFrameCodedInVGop, svc_rc->iQp[svc_rc->iFrameCodedInVGop], iTlLast);
    }
    h264e_hal_dbg(H264E_DBG_RC, "tsvc temporalid %d targetbit %d config qp %d\n", svc_rc->uiTemporalId, rc_syn->bit_target, hw_cfg->qp);
}

MPP_RET h264e_vpu_mb_rc_cfg(H264eHalContext *ctx, RcSyntax *rc_syn, H264eHwCfg *hw_cfg)
{
    const RK_S32 sscale = 256;
    VepuQpCtrl *qc = &hw_cfg->qpCtrl;
    RK_S32 scaler, srcPrm;
    RK_S32 i;
    RK_S32 tmp, nonZeroTarget;
    MppEncCfgSet *cfg = ctx->cfg;
    MppEncH264Cfg *codec = &cfg->codec.h264;
    MppEncRcCfg *rc = &cfg->rc;
    RK_S32 mbPerPic = (hw_cfg->width + 15) / 16 * (hw_cfg->height + 15) / 16;
    RK_S32 coeffCntMax = mbPerPic * 24 * 16;
    RK_S32 bits_per_pic = axb_div_c(rc->bps_target,
                                    rc->fps_out_denorm,
                                    rc->fps_out_num);

    if (hw_cfg->qp <= 0) {
        RK_S32 qp_tbl[2][13] = {
            {
                26, 36, 48, 63, 85, 110, 152, 208, 313, 427, 936,
                1472, 0x7fffffff
            },
            {42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6}
        };
        RK_S32 pels = ctx->cfg->prep.width * ctx->cfg->prep.height;
        if (pels) {
            RK_S32 upscale = 8000;
            if (bits_per_pic > 1000000)
                hw_cfg->qp = codec->qp_min;
            else {
                RK_S32 j = -1;

                pels >>= 8;
                bits_per_pic >>= 5;

                bits_per_pic *= pels + 250;
                bits_per_pic /= 350 + (3 * pels) / 4;
                bits_per_pic = axb_div_c(bits_per_pic, upscale, pels << 6);

                while (qp_tbl[0][++j] < bits_per_pic);

                hw_cfg->qp = qp_tbl[1][j];
                hw_cfg->qp_prev = hw_cfg->qp;
            }
        }
        //first frame init
    }
    if (ctx->frame_cnt == 0) {
        RK_S32 mbRows = ctx->cfg->prep.height / 16;
        hw_cfg->mad_qp_delta = 2;
        hw_cfg->mad_threshold = 256 * 6;
        hw_cfg->qpCtrl.checkPoints = MPP_MIN(mbRows - 1, CHECK_POINTS_MAX);
        if (rc->rc_mode == MPP_ENC_RC_MODE_CBR) {
            hw_cfg->qpCtrl.checkPointDistance =
                mbPerPic / (hw_cfg->qpCtrl.checkPoints + 1);
        } else {
            hw_cfg->qpCtrl.checkPointDistance = 0;
        }
    }
    /* frame type and rate control setup */
    if (ctx->tsvc_rc.tlayer_num > 1) {
        h264e_vpu_tsvc_rc_calqp(ctx, rc_syn, hw_cfg);
    } else {
        hw_cfg->pre_frame_type = hw_cfg->frame_type;
        if (rc_syn->type == INTRA_FRAME) {
            hw_cfg->frame_type = H264E_VPU_FRAME_I;
            h264e_vpu_rc_calIfrmqp(ctx, rc_syn, hw_cfg);
            hw_cfg->qp_min = 22;
            hw_cfg->qp_max = 45;
            hw_cfg->qp = mpp_clip(hw_cfg->qp, hw_cfg->qp_min, hw_cfg->qp_max);
        } else {
            hw_cfg->frame_type = H264E_VPU_FRAME_P;
            h264e_vpu_rc_calPfrmqp(ctx, hw_cfg);
        }
    }

    hw_cfg->qp = mpp_clip(hw_cfg->qp,
                          hw_cfg->qp_prev - codec->qp_max_step,
                          hw_cfg->qp_prev + codec->qp_max_step);

    hw_cfg->qp = mpp_clip(hw_cfg->qp, codec->qp_min, codec->qp_max);


    if (qc->nonZeroCnt == 0) {
        qc->nonZeroCnt = 1;
    }

    srcPrm = axb_div_c(qc->frameBitCnt, 256, qc->nonZeroCnt);
    /* Disable Mb Rc for Intra Slices, because coeffTarget will be wrong */
    if (hw_cfg->frame_type == INTRA_FRAME || srcPrm == 0) {
        return 0;
    }

    /* Required zero cnt */
    nonZeroTarget = axb_div_c(rc_syn->bit_target, 256, srcPrm);
    nonZeroTarget = MPP_MIN(coeffCntMax, MPP_MAX(0, nonZeroTarget));
    nonZeroTarget = MPP_MIN(0x7FFFFFFFU / 1024U, (RK_U32)nonZeroTarget);

    if (nonZeroTarget > 0) {
        scaler = axb_div_c(nonZeroTarget, sscale, (RK_S32) mbPerPic);
    } else {
        return 0;
    }

    if ((hw_cfg->frame_type != hw_cfg->pre_frame_type) || (qc->nonZeroCnt == 0)) {

        for (i = 0; i < qc->checkPoints; i++) {
            tmp = (scaler * (qc->checkPointDistance * (i + 1) + 1)) / sscale;
            tmp = MPP_MIN(WORD_CNT_MAX, tmp / 32 + 1);
            if (tmp < 0) tmp = WORD_CNT_MAX;    /* Detect overflow */
            hw_cfg->cp_target[i] = tmp; /* div32 for regs */
        }
        tmp = axb_div_c(bits_per_pic, 256, srcPrm);
    } else {
        for (i = 0; i < qc->checkPoints; i++) {
            tmp = (RK_S32) (qc->wordCntPrev[i] * scaler) / sscale;
            tmp = MPP_MIN(WORD_CNT_MAX, tmp / 32 + 1);
            if (tmp < 0) tmp = WORD_CNT_MAX;    /* Detect overflow */
            hw_cfg->cp_target[i] = tmp; /* div32 for regs */
        }
        tmp = axb_div_c(bits_per_pic, 256, srcPrm);
    }
    if (ctx->tsvc_rc.tlayer_num > 1) {
        hw_cfg->target_error[0] = -tmp * 3;
        hw_cfg->delta_qp[0] = -1;
        hw_cfg->target_error[1] = -tmp * 2;
        hw_cfg->delta_qp[1] = -1;
        hw_cfg->target_error[2] = -tmp * 1;
        hw_cfg->delta_qp[2] = -1;
        hw_cfg->target_error[3] = tmp * 1;
        hw_cfg->delta_qp[3] = 0;
        hw_cfg->target_error[4] = tmp * 2;
        hw_cfg->delta_qp[4] = 1;
        hw_cfg->target_error[5] = tmp * 3;
        hw_cfg->delta_qp[5] = 4;
        hw_cfg->target_error[6] = tmp * 4;
        hw_cfg->delta_qp[6] = 7;
    } else {
        hw_cfg->target_error[0] = -tmp * 3;
        hw_cfg->delta_qp[0] = -2;
        hw_cfg->target_error[1] = -tmp * 2;
        hw_cfg->delta_qp[1] = -2;
        hw_cfg->target_error[2] = -tmp * 1;
        hw_cfg->delta_qp[2] = -1;
        hw_cfg->target_error[3] = tmp * 1;
        hw_cfg->delta_qp[3] = 0;
        hw_cfg->target_error[4] = tmp * 2;
        hw_cfg->delta_qp[4] = 1;
        hw_cfg->target_error[5] = tmp * 3;
        hw_cfg->delta_qp[5] = 2;
        hw_cfg->target_error[6] = tmp * 4;
        hw_cfg->delta_qp[6] = 3;
    }

    if (ctx->tsvc_rc.uiTemporalId == 0) {
        hw_cfg->delta_qp[0] = 0;
        hw_cfg->delta_qp[1] = 0;
        hw_cfg->delta_qp[2] = 0;
        hw_cfg->delta_qp[3] = 0;
        hw_cfg->delta_qp[4] = 1;
        hw_cfg->delta_qp[5] = 2;
        hw_cfg->delta_qp[6] = 3;
    }

    for (i = 0; i < CTRL_LEVELS; i++) {
        tmp =  hw_cfg->cp_target[i];
        tmp = mpp_clip(tmp / 4, -32768, 32767);
        hw_cfg->cp_target[i] = tmp;
    }
    hw_cfg->cp_distance_mbs = hw_cfg->qpCtrl.checkPointDistance;
    return 0;
}

MPP_RET h264e_vpu_mad_threshold(H264eHwCfg *hw_cfg, MppLinReg *mad, RK_U32 madCount)
{
    RK_S32 mbPerPic = (hw_cfg->width + 15) / 16 * (hw_cfg->height + 15) / 16;
    RK_U32 targetCount = 30 * mbPerPic / 100;
    RK_S32 threshold = hw_cfg->mad_threshold;
    RK_S32 lowLimit, highLimit;

    mpp_save_regdata(mad, hw_cfg->mad_threshold, madCount);
    mpp_linreg_update(mad);
//    mpp_log("hw_cfg->mad_threshold = %d",hw_cfg->mad_threshold);
    /* Calculate new threshold for next frame using either linear regression
     * model or adjustment based on current setting */
    if (mad->a)
        threshold = mad->a * targetCount / 32 + mad->b;
    else if (madCount < targetCount)
        threshold = MPP_MAX(hw_cfg->mad_threshold * 5 / 4, hw_cfg->mad_threshold + 256);
    else
        threshold = MPP_MIN(hw_cfg->mad_threshold * 3 / 4, hw_cfg->mad_threshold - 256);

    /* For small count, ensure that we increase the threshold minimum 1 step */
    if (madCount < targetCount / 2)
        threshold = MPP_MAX(threshold, hw_cfg->mad_threshold + 256);

    /* If previous frame had zero count, ensure that we increase threshold */
    if (!madCount)
        threshold = MPP_MAX(threshold, hw_cfg->mad_threshold + 256 * 4);

    /* Limit how much the threshold can change between two frames */
    lowLimit = hw_cfg->mad_threshold / 2;
    highLimit = MPP_MAX(hw_cfg->mad_threshold * 2, 256 * 4);
    hw_cfg->mad_threshold = MPP_MIN(highLimit, MPP_MAX(lowLimit, threshold));

    /* threshold_div256 has 6-bits range [0,63] */
    hw_cfg->mad_threshold = ((hw_cfg->mad_threshold + 128) / 256) * 256;
    hw_cfg->mad_threshold = MPP_MAX(0, MPP_MIN(63 * 256, hw_cfg->mad_threshold));
    return 0;
}

MPP_RET h264e_vpu_update_hw_cfg(H264eHalContext *ctx, HalEncTask *task,
                                H264eHwCfg *hw_cfg)
{
    MppEncCfgSet *cfg = ctx->cfg;
    MppEncH264Cfg *codec = &cfg->codec.h264;
    MppEncPrepCfg *prep = &cfg->prep;
    MppEncRcCfg *rc = &cfg->rc;
    RcSyntax *rc_syn = (RcSyntax *)task->syntax.data;

    /* preprocess setup */
    if (prep->change) {
        RK_U32 change = prep->change;

        if (change & MPP_ENC_PREP_CFG_CHANGE_INPUT) {
            hw_cfg->width   = prep->width;
            hw_cfg->height  = prep->height;

            hw_cfg->hor_stride = prep->hor_stride;
            hw_cfg->ver_stride = prep->ver_stride;
        }

        if (change & MPP_ENC_PREP_CFG_CHANGE_FORMAT) {
            hw_cfg->input_format = prep->format;
            h264e_vpu_set_format(hw_cfg, prep);
            switch (prep->color) {
            case MPP_FRAME_SPC_RGB : {
                /* BT.601 */
                /* Y  = 0.2989 R + 0.5866 G + 0.1145 B
                 * Cb = 0.5647 (B - Y) + 128
                 * Cr = 0.7132 (R - Y) + 128
                 */
                hw_cfg->color_conversion_coeff_a = 19589;
                hw_cfg->color_conversion_coeff_b = 38443;
                hw_cfg->color_conversion_coeff_c = 7504;
                hw_cfg->color_conversion_coeff_e = 37008;
                hw_cfg->color_conversion_coeff_f = 46740;
            } break;
            case MPP_FRAME_SPC_BT709 : {
                /* BT.709 */
                /* Y  = 0.2126 R + 0.7152 G + 0.0722 B
                 * Cb = 0.5389 (B - Y) + 128
                 * Cr = 0.6350 (R - Y) + 128
                 */
                hw_cfg->color_conversion_coeff_a = 13933;
                hw_cfg->color_conversion_coeff_b = 46871;
                hw_cfg->color_conversion_coeff_c = 4732;
                hw_cfg->color_conversion_coeff_e = 35317;
                hw_cfg->color_conversion_coeff_f = 41615;
            } break;
            default : {
                hw_cfg->color_conversion_coeff_a = 19589;
                hw_cfg->color_conversion_coeff_b = 38443;
                hw_cfg->color_conversion_coeff_c = 7504;
                hw_cfg->color_conversion_coeff_e = 37008;
                hw_cfg->color_conversion_coeff_f = 46740;
            } break;
            }
        }

        prep->change = 0;
    }

    if (codec->change) {
        // TODO: setup sps / pps here
        hw_cfg->idr_pic_id = ctx->idr_pic_id;
        ctx->idr_pic_id = !ctx->idr_pic_id;
        ctx->svc = ctx->cfg->codec.h264.svc;

        hw_cfg->filter_disable = codec->deblock_disable;
        hw_cfg->slice_alpha_offset = codec->deblock_offset_alpha;
        hw_cfg->slice_beta_offset = codec->deblock_offset_beta;
        hw_cfg->inter4x4_disabled = (codec->profile >= 31) ? (1) : (0);
        hw_cfg->cabac_init_idc = codec->cabac_init_idc;
        hw_cfg->qp = codec->qp_init;
        hw_cfg->qp_prev = hw_cfg->qp;
        codec->change = 0;
    }
    if (NULL == ctx->intra_qs)
        mpp_linreg_init(&ctx->intra_qs, MPP_MIN(rc->gop, 10), 2);
    if (NULL == ctx->inter_qs)
        mpp_linreg_init(&ctx->inter_qs, MPP_MIN(rc->gop, 10), 2);

    if (NULL == ctx->mad)
        mpp_linreg_init(&ctx->mad, 5, 1);

    if (NULL == ctx->qp_p)
        mpp_data_init(&ctx->qp_p, MPP_MIN(rc->gop, 6));

    mpp_assert(ctx->intra_qs);
    mpp_assert(ctx->inter_qs);

    if (rc_syn->type == INTRA_FRAME) {
        hw_cfg->frame_type = H264E_VPU_FRAME_I;
        hw_cfg->frame_num = 0;
        h264e_hal_dbg(H264E_DBG_RC, "frame type I, frame_num 0\n");
    } else {
        hw_cfg->frame_type = H264E_VPU_FRAME_P;
        h264e_hal_dbg(H264E_DBG_RC, "frame type P, frame_num %d\n", hw_cfg->frame_num);
    }


    hw_cfg->keyframe_max_interval = rc->gop;
    hw_cfg->qp_min = codec->qp_min;
    hw_cfg->qp_max = codec->qp_max;

    if (rc->rc_mode == MPP_ENC_RC_MODE_VBR &&
        rc->quality == MPP_ENC_RC_QUALITY_CQP) {
        hw_cfg->qp = codec->qp_init;
    } else {
        /* enable mb rate control*/
        h264e_vpu_mb_rc_cfg(ctx, rc_syn, hw_cfg);
    }
    /* slice mode setup */
    hw_cfg->slice_size_mb_rows = 0; //(prep->height + 15) >> 4;

    if (ctx->cfg->misc.split.split_en) {
        RK_U32 mb_per_col = (hw_cfg->height + 15) / 16;
        MppEncSliceSplit *split = &ctx->cfg->misc.split;

        mpp_assert(split->slice_size > 0);
        RK_S32 slice_num = rc_syn->bit_target / (split->slice_size * 8);
        h264e_hal_dbg(H264E_DBG_RC, "ideal slice_num %d, slice_size %d\n", slice_num, split->slice_size);
        if (slice_num <= 0) {
            slice_num = 4;
        }

        RK_U32 slice_mb_rows =  (mb_per_col + slice_num - 1) / slice_num;
        h264e_hal_dbg(H264E_DBG_RC, "fixed_slice_size_mb_rows %d\n", hw_cfg->fixed_slice_size_mb_rows);
        hw_cfg->slice_size_mb_rows = hw_cfg->fixed_slice_size_mb_rows ?
                                     hw_cfg->fixed_slice_size_mb_rows :
                                     (RK_U32)mpp_clip(slice_mb_rows, 1, 127);

        slice_num = (mb_per_col + hw_cfg->slice_size_mb_rows - 1) / (hw_cfg->slice_size_mb_rows);
        h264e_hal_dbg(H264E_DBG_RC, "real slice_num %d hw_cfg->slice_size_mb_rows = %d, bit_target = %d, mb_per_column = %d",
                      slice_num, hw_cfg->slice_size_mb_rows,
                      rc_syn->bit_target, mb_per_col);
    }

    /* input and preprocess config, the offset is at [31:10] */
    hw_cfg->input_luma_addr = mpp_buffer_get_fd(task->input);

    switch (prep->format) {
    case MPP_FMT_YUV420SP: {
        RK_U32 offset_uv = hw_cfg->hor_stride * hw_cfg->ver_stride;

        // mpp_assert(prep->hor_stride == MPP_ALIGN(prep->width, 8));
        // mpp_assert(prep->ver_stride == MPP_ALIGN(prep->height, 8));

        hw_cfg->input_cb_addr = hw_cfg->input_luma_addr + (offset_uv << 10);
        hw_cfg->input_cr_addr = 0;
        break;
    }
    case MPP_FMT_YUV420P: {
        RK_U32 offset_y = hw_cfg->hor_stride * hw_cfg->ver_stride;

        mpp_assert(prep->hor_stride == MPP_ALIGN(prep->width, 8));
        // mpp_assert(prep->ver_stride == MPP_ALIGN(prep->height, 8));

        hw_cfg->input_cb_addr = hw_cfg->input_luma_addr + (offset_y << 10);
        hw_cfg->input_cr_addr = hw_cfg->input_cb_addr + (offset_y << 8);
        break;
    }
    case MPP_FMT_YUV422_YUYV:
    case MPP_FMT_YUV422_UYVY:
    case MPP_FMT_RGB565:
    case MPP_FMT_BGR444:
    case MPP_FMT_BGR888:
    case MPP_FMT_RGB888:
    case MPP_FMT_ARGB8888:
    case MPP_FMT_ABGR8888:
    case MPP_FMT_BGR101010:
        hw_cfg->input_cb_addr = 0;
        hw_cfg->input_cr_addr = 0;
        break;
    default: {
        mpp_err_f("invalid input format %d", prep->format);
        return MPP_ERR_VALUE;
    }
    }
    hw_cfg->output_strm_addr = mpp_buffer_get_fd(task->output);
    hw_cfg->output_strm_limit_size = mpp_buffer_get_size(task->output);

    /* context update */
    ctx->idr_pic_id = !ctx->idr_pic_id;
    return MPP_OK;
}


void h264e_vpu_update_result(H264eHwCfg *hw_cfg, RK_S32 pic_bits, RK_S32 target_bits)
{
    hw_cfg->pre_bit_diff = pic_bits - target_bits;
    hw_cfg->pre_pic_bits = pic_bits;
    hw_cfg->pre_target_bit = target_bits;
}

void h264e_vpu_tsvcrc_update(H264eHalTsvcRc *tsvc_rc, RK_S32 pic_bits,  RK_S32 avg_qp)
{
    tsvc_rc->iQp[tsvc_rc->iFrameCodedInVGop] = avg_qp;
    tsvc_rc->iFrameBits[tsvc_rc->iFrameCodedInVGop] = pic_bits;
    tsvc_rc->iFrameCodedInVGop++;
    if (tsvc_rc->iFrameCodedInVGop >= VGOP_SIZE) {
        tsvc_rc->iFrameCodedInVGop = 0;
    }
}

void h264e_vpu_tsvc_rc_init(H264eHalTsvcRc *svc_rc, RK_S32 max_layer_id)
{

    RK_S32 kiGopSize;
    RK_S32 i, n, k;
    kiGopSize = (1 << max_layer_id);
    svc_rc->iGopNumberInVGop = VGOP_SIZE / kiGopSize;
    svc_rc->iGopIndexInVGop = 0;
    svc_rc->iFrameCodedInVGop = 0;
    for (n = 0; n < VGOP_SIZE; n += kiGopSize) {
        svc_rc->iTlOfFrames[n] = 0;
        for (i = 1; i <= max_layer_id; i++) {
            for (k = 1 << (max_layer_id - i); k < kiGopSize; k += (kiGopSize >> (i - 1))) {
                svc_rc->iTlOfFrames[k + n] = i;
            }
        }
    }
    svc_rc->tlayer_num = max_layer_id + 1;
}

void h264e_check_reencode(void *hal, HalTaskInfo *task, void *reg_out,
                          MPP_RET (*send)(void* hal, RK_U32 *reg, RK_S32 d_qp),
                          MPP_RET (*feedback)(h264e_feedback* fb, void * reg))
{
    H264eHalContext *ctx = (H264eHalContext *)hal;
    h264e_feedback *fb = &ctx->feedback;
    RcSyntax *rc_syn = (RcSyntax *)task->enc.syntax.data;
    H264eHwCfg *hw_cfg = &ctx->hw_cfg;
    MppEncPrepCfg *prep = &ctx->set->prep;
    RK_S32 ratio = 0;
    RK_S32 pic_bits;
    RK_S32 num_mb = (MPP_ALIGN(prep->width, 16) * MPP_ALIGN(prep->height, 16)) >> 8;
    H264eHalTsvcRc *tsvc_rc = &ctx->tsvc_rc;
    if (tsvc_rc->tlayer_num > 1) {
        int dealt_qp = 0;
        int flag = 1;
        int cnt = 1;
        int re_qp = hw_cfg->qp;
        RK_S32 re_max_bits = 0, re_min_bits = 0;
        RK_S32 re_min_qp = 1, re_max_qp = 60;
        if (fb->out_strm_size * 8 > rc_syn->bit_target) {
            re_max_bits = fb->out_strm_size * 8;
            re_min_qp = hw_cfg->qp;
        } else {
            re_min_bits = fb->out_strm_size * 8;
            re_max_qp = hw_cfg->qp;
        }
        if (tsvc_rc->uiTemporalId <= 1) {
            cnt = 2;
        }
        do {
            pic_bits =  fb->out_strm_size * 8;
            h264e_hal_dbg(H264E_DBG_RC, "pic_bits %d rc_syn->bit_target %d ", pic_bits, rc_syn->bit_target);
            if (pic_bits - rc_syn->bit_target >= rc_syn->bit_target * 20 / 100
                || pic_bits - rc_syn->bit_target <= -pic_bits * 20 / 100) {

                if (pic_bits - rc_syn->bit_target > 0) {
                    ratio = (pic_bits - rc_syn->bit_target) * 100 / rc_syn->bit_target;
                    if (pic_bits < re_max_bits || re_max_bits == 0) {
                        re_max_bits = pic_bits;
                        re_min_qp = hw_cfg->qp;
                    }
                } else {
                    ratio = (pic_bits - rc_syn->bit_target) * 100 / pic_bits;
                    if (pic_bits > re_min_bits || re_min_bits == 0) {
                        re_min_bits = pic_bits;
                        re_max_qp = hw_cfg->qp;
                    }
                }
                ratio = MPP_MIN(ratio, 300);
                if (rc_syn->type == INTRA_FRAME) {
                    dealt_qp = (ratio * 6) / 100 + 1;
                } else {
                    if (cnt == 0) {
                        if (!flag && ratio > 0) {
                            dealt_qp = (dealt_qp >> 1);
                            hw_cfg->qp = re_qp;
                        } else {
                            dealt_qp = 0;
                        }

                    } else {
                        if (ratio < 0) {
                            flag = 0;
                        }
                        if (tsvc_rc->uiTemporalId <= 4) {
                            ratio = MPP_MIN(ratio, 300);
                            ratio = MPP_MAX(ratio, -200);
                            if (tsvc_rc->uiTemporalId <= 2)
                                dealt_qp = (ratio * 3) / 100 + flag;
                            else
                                dealt_qp = (ratio * 4) / 100 + flag;
                        }

                    }
                    h264e_hal_dbg(H264E_DBG_RC, "pic_bits %d, re_max_bits %d, re_min_bits %d\n", pic_bits, re_max_bits, re_min_bits);
                    h264e_hal_dbg(H264E_DBG_RC, "re_min qp %d re_max_qp %d delta_qp %d hw_cfg->qp %d\n",
                                  re_min_qp, re_max_qp, dealt_qp, hw_cfg->qp);
                    if ((hw_cfg->qp + dealt_qp) != MPP_CLIP3(re_min_qp, re_max_qp, (hw_cfg->qp + dealt_qp))) {
                        dealt_qp = (re_max_qp + re_min_qp) / 2 - hw_cfg->qp;
                    }
                }
                if (dealt_qp != 0) {
                    h264e_hal_dbg(H264E_DBG_RC, "reencode dealt_qp %d, qp %d\n", dealt_qp, hw_cfg->qp + dealt_qp);
                    send(hal, (RK_U32 *)reg_out, dealt_qp);
                    feedback(fb, reg_out);
                    task->enc.length = fb->out_strm_size;
                    hw_cfg->qp_prev = fb->qp_sum / num_mb;
                    if ((cnt-- <= 0) || (hw_cfg->qp == hw_cfg->qp_max)) {
                        break;
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        } while (1);
    } else {
        if (rc_syn->type == INTER_P_FRAME) {
            int dealt_qp = 3;
            int cnt = 2;
            do {
                if (hw_cfg->qp < 30) {
                    dealt_qp = 5;
                } else if (hw_cfg->qp < 42) {
                    dealt_qp = 3;
                } else {
                    dealt_qp = 2;
                }
                if ((fb->out_strm_size * 8 >  (RK_U32)rc_syn->bit_target * 3) && (hw_cfg->qp < hw_cfg->qp_max)) {
                    send(hal, (RK_U32 *)reg_out, dealt_qp);
                    feedback(fb, reg_out);
                    task->enc.length = fb->out_strm_size;
                    hw_cfg->qp_prev = fb->qp_sum / num_mb;
                    if ((cnt-- <= 0) || (hw_cfg->qp == hw_cfg->qp_max)) {
                        break;
                    }
                } else {
                    break;
                }
            } while (1);
        }
    }
}

