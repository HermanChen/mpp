/*
 * Copyright 2015 Rockchip Electronics Co. LTD
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

#define MODULE_TAG "hal_h264e_vepu2"

#include <string.h>
#include "mpp_device.h"

#include "mpp_env.h"
#include "mpp_mem.h"
#include "mpp_frame.h"
#include "mpp_common.h"
#include "mpp_rc.h"

#include "hal_h264e_com.h"
#include "hal_h264e_header.h"
#include "hal_h264e_vpu_tbl.h"

#include "hal_h264e_vepu.h"
#include "hal_h264e_rc.h"
#include "hal_h264e_vepu2.h"
#include "hal_h264e_vepu2_reg_tbl.h"

static RK_U32 hal_vpu_h264e_debug = 0;

MPP_RET hal_h264e_vepu2_init(void *hal, MppHalCfg *cfg)
{
    H264eHalContext *ctx = (H264eHalContext *)hal;
    MPP_RET ret = MPP_OK;

    h264e_hal_enter();

    ctx->int_cb     = cfg->hal_int_cb;
    ctx->regs       = mpp_calloc(H264eVpu2RegSet, 1);
    ctx->buffers    = mpp_calloc(h264e_hal_vpu_buffers, 1);
    ctx->extra_info = mpp_calloc(H264eVpuExtraInfo, 1);
    ctx->param_buf  = mpp_calloc_size(void,  H264E_EXTRA_INFO_BUF_SIZE);
    mpp_packet_init(&ctx->packeted_param, ctx->param_buf, H264E_EXTRA_INFO_BUF_SIZE);

    ctx->buf_size = SZ_128K;
    ctx->src_buf = mpp_calloc(RK_U8, ctx->buf_size);
    ctx->dst_buf = mpp_calloc(RK_U8, ctx->buf_size);

    h264e_vpu_init_extra_info(ctx->extra_info);

    MppDevCfg dev_cfg = {
        .type = MPP_CTX_ENC,            /* type */
        .coding = MPP_VIDEO_CodingAVC,  /* coding */
        .platform = 0,                  /* platform */
        .pp_enable = 0,                 /* pp_enable */
    };
    ret = mpp_device_init(&ctx->dev_ctx, &dev_cfg);
    if (ret) {
        mpp_err("mpp_device_init failed. ret: %d\n", ret);
        return ret;
    }

    ctx->hw_cfg.qp_prev = ctx->cfg->codec.h264.qp_init;
    mpp_env_get_u32("hal_vpu_h264e_debug", &hal_vpu_h264e_debug, 0);

    ret = h264e_vpu_allocate_buffers(ctx);
    if (ret) {
        h264e_hal_err("allocate buffers failed\n");
        h264e_vpu_free_buffers(ctx);
    }

    h264e_hal_leave();
    return ret;
}

MPP_RET hal_h264e_vepu2_deinit(void *hal)
{
    MPP_RET ret = MPP_OK;
    H264eHalContext *ctx = (H264eHalContext *)hal;
    h264e_hal_enter();

    if (ctx->dpb) {
        h264e_dpb_deinit(ctx->dpb);
        ctx->dpb = NULL;
    }

    MPP_FREE(ctx->regs);
    MPP_FREE(ctx->param_buf);

    if (ctx->extra_info) {
        h264e_vpu_deinit_extra_info(ctx->extra_info);
        MPP_FREE(ctx->extra_info);
    }

    if (ctx->packeted_param) {
        mpp_packet_deinit(&ctx->packeted_param);
        ctx->packeted_param = NULL;
    }

    if (ctx->buffers) {
        h264e_vpu_free_buffers(ctx);
        MPP_FREE(ctx->buffers);
    }

    MPP_FREE(ctx->param_buf);
    MPP_FREE(ctx->src_buf);
    MPP_FREE(ctx->dst_buf);

    if (ctx->intra_qs) {
        mpp_linreg_deinit(ctx->intra_qs);
        ctx->intra_qs = NULL;
    }

    if (ctx->inter_qs) {
        mpp_linreg_deinit(ctx->inter_qs);
        ctx->inter_qs = NULL;
    }

    if (ctx->qp_p) {
        mpp_data_deinit(ctx->qp_p);
        ctx->qp_p = NULL;
    }

    if (ctx->mad) {
        mpp_linreg_deinit(ctx->mad);
        ctx->mad = NULL;
    }
    ret = mpp_device_deinit(ctx->dev_ctx);
    if (ret)
        mpp_err("mpp_device_deinit failed, ret: %d", ret);

    h264e_hal_leave();
    return ret;
}

MPP_RET hal_h264e_vepu2_gen_regs(void *hal, HalTaskInfo *task)
{
    MPP_RET ret = MPP_OK;
    H264eHalContext *ctx = (H264eHalContext *)hal;
    H264eVpuExtraInfo *extra_info = (H264eVpuExtraInfo *)ctx->extra_info;
    H264ePps *pps = &extra_info->pps;
    h264e_hal_vpu_buffers *bufs = (h264e_hal_vpu_buffers *)ctx->buffers;
    MppEncPrepCfg *prep = &ctx->cfg->prep;
    H264eHwCfg *hw_cfg = &ctx->hw_cfg;
    RK_U32 *reg = (RK_U32 *)ctx->regs;
    HalEncTask *enc_task = &task->enc;

    RK_S32 scaler = 0, i = 0;
    RK_U32 val = 0, skip_penalty = 0;
    RK_U32 overfill_r = 0, overfill_b = 0;

    RK_U32 mb_w;
    RK_U32 mb_h;

    MppBuffer recn = NULL;
    MppBuffer refr = NULL;

    h264e_hal_enter();
    // generate parameter from config
    h264e_vpu_update_hw_cfg(ctx, enc_task, hw_cfg);
    ret = h264e_vpu_update_buffers(ctx, hw_cfg);
    if (ret != MPP_OK) {
        mpp_err_f("update buffers error.\n");
        return ret;
    }
    if (ctx->hdr_status & HDR_NEED_UPDATED) {
        // mpp_log("update header\n");
        mpp_assert(hw_cfg->frame_type == H264E_VPU_FRAME_I);
        mpp_assert(!(ctx->hdr_status & HDR_UPDATED));

        MppPacket  pkt = ctx->packeted_param;
        H264eVpuExtraInfo *src = (H264eVpuExtraInfo *)ctx->extra_info;
        H264eStream *sps_stream = &src->sps_stream;
        H264eStream *pps_stream = &src->pps_stream;
        H264eStream *sei_stream = &src->sei_stream;

        size_t offset = 0;

        h264e_vpu_set_extra_info(ctx);

        mpp_packet_write(pkt, offset, sps_stream->buffer, sps_stream->byte_cnt);
        offset += sps_stream->byte_cnt;

        mpp_packet_write(pkt, offset, pps_stream->buffer, pps_stream->byte_cnt);
        offset += pps_stream->byte_cnt;

        mpp_packet_write(pkt, offset, sei_stream->buffer, sei_stream->byte_cnt);
        offset += sei_stream->byte_cnt;

        mpp_packet_set_length(pkt, offset);

        ctx->hdr_status &= ~HDR_NEED_UPDATED;
        ctx->hdr_status |= HDR_UPDATED;

        // mpp_log("header length %d\n", offset);
    }
    mb_w = (prep->width  + 15) / 16;
    mb_h = (prep->height + 15) / 16;

    if (ctx->usr_hier) {
        h264e_init_vepu_slice(ctx);

        if (NULL == ctx->dpb) {
            RK_U32 size = mb_w * mb_h * 256 * 2;
            H264eSps *sps = &extra_info->sps;
            H264eDpbCfg cfg;

            cfg.poc_type = sps->i_poc_type;
            cfg.ref_frm_num = ctx->slice.max_num_ref_frames;
            cfg.log2_max_frm_num = sps->i_log2_max_frame_num;
            cfg.log2_max_poc_lsb = sps->i_log2_max_poc_lsb;

            h264e_dpb_init(&ctx->dpb, &cfg);
            h264e_dpb_setup_buf_size(ctx->dpb, &size, 1);
            h264e_dpb_setup_hier(ctx->dpb, &ctx->hier_cfg);
        }

        MppMeta meta = mpp_frame_get_meta(enc_task->frame);
        RK_S32 lt_ref_idx = -1;
        H264eDpbFrmCfg frm_cfg;

        mpp_meta_get_s32(meta, KEY_LONG_REF_IDX, &lt_ref_idx);
        frm_cfg.idr_req = (hw_cfg->frame_type == H264E_VPU_FRAME_I);
        frm_cfg.ref_to_lt_idx = lt_ref_idx;

        H264eDpbFrm *frm = h264e_dpb_get_curr(ctx->dpb, &frm_cfg);
        H264eDpbFrm *ref = frm->ref_frm;

        h264e_dpb_build_list(ctx->dpb);

        recn = h264e_dpb_frm_get_buf(frm, 0);
        mpp_assert(recn);

        refr = h264e_dpb_frm_get_buf(ref, 0);
        if (NULL == refr)
            refr = recn;

        enc_task->temporal_id = frm->info.temporal_id;

        h264e_dpb_build_marking(ctx->dpb);

        meta = mpp_packet_get_meta(enc_task->packet);
        if (meta) {
            mpp_meta_set_s32(meta, KEY_TEMPORAL_ID, frm->info.temporal_id);
            mpp_meta_set_s32(meta, KEY_LONG_REF_IDX, frm->lt_idx);
        }
        hw_cfg->frame_num = frm->frame_num;
    } else {
        RK_U32 buf2_idx = ctx->frame_cnt & 1;

        recn = bufs->hw_rec_buf[buf2_idx];
        refr = bufs->hw_rec_buf[1 - buf2_idx];
        enc_task->temporal_id = 0;

        if (ctx->dpb) {
            h264e_dpb_deinit(ctx->dpb);
            ctx->dpb = NULL;
        }
    }
    memset(reg, 0, sizeof(H264eVpu2RegSet));

    h264e_hal_dbg(H264E_DBG_DETAIL, "frame %d generate regs now", ctx->frame_cnt);

    /* output buffer size is 64 bit address then 8 multiple size */
    val = mpp_buffer_get_size(enc_task->output);
    val >>= 3;
    val &= ~7;
    H264E_HAL_SET_REG(reg, VEPU_REG_STR_BUF_LIMIT, val);

    /*
     * The hardware needs only the value for luma plane, because
     * values of other planes are calculated internally based on
     * format setting.
     */
    val = VEPU_REG_INTRA_AREA_TOP(mb_h)
          | VEPU_REG_INTRA_AREA_BOTTOM(mb_h)
          | VEPU_REG_INTRA_AREA_LEFT(mb_w)
          | VEPU_REG_INTRA_AREA_RIGHT(mb_w);
    H264E_HAL_SET_REG(reg, VEPU_REG_INTRA_AREA_CTRL, val); //FIXED
    H264E_HAL_SET_REG(reg, VEPU_REG_STR_HDR_REM_MSB, 0);
    H264E_HAL_SET_REG(reg, VEPU_REG_STR_HDR_REM_LSB, 0);


    val = VEPU_REG_AXI_CTRL_READ_ID(0);
    val |= VEPU_REG_AXI_CTRL_WRITE_ID(0);
    val |= VEPU_REG_AXI_CTRL_BURST_LEN(16);
    val |= VEPU_REG_AXI_CTRL_INCREMENT_MODE(0);
    val |= VEPU_REG_AXI_CTRL_BIRST_DISCARD(0);
    H264E_HAL_SET_REG(reg, VEPU_REG_AXI_CTRL, val);

    H264E_HAL_SET_REG(reg, VEPU_QP_ADJUST_MAD_DELTA_ROI, hw_cfg->mad_qp_delta);

    val = 0;
    if (mb_w * mb_h > 3600)
        val = VEPU_REG_DISABLE_QUARTER_PIXEL_MV;
    val |= VEPU_REG_CABAC_INIT_IDC(hw_cfg->cabac_init_idc);
    if (pps->b_cabac)
        val |= VEPU_REG_ENTROPY_CODING_MODE;
    if (pps->b_transform_8x8_mode)
        val |= VEPU_REG_H264_TRANS8X8_MODE;
    if (hw_cfg->inter4x4_disabled)
        val |= VEPU_REG_H264_INTER4X4_MODE;
//    reg |= VEPU_REG_H264_STREAM_MODE;
    val |= VEPU_REG_H264_SLICE_SIZE(hw_cfg->slice_size_mb_rows);
    H264E_HAL_SET_REG(reg, VEPU_REG_ENC_CTRL0, val);

    scaler = H264E_HAL_MAX(1, 200 / (mb_w + mb_h));
    skip_penalty = H264E_HAL_MIN(255, h264_skip_sad_penalty[hw_cfg->qp] * scaler);
    skip_penalty = 0xff;
    if (prep->width & 0x0f)
        overfill_r = (16 - (prep->width & 0x0f) ) / 4;
    if (prep->height & 0x0f)
        overfill_b = 16 - (prep->height & 0x0f);
    val = VEPU_REG_STREAM_START_OFFSET(0) | /* first_free_bit */
          VEPU_REG_SKIP_MACROBLOCK_PENALTY(skip_penalty) |
          VEPU_REG_IN_IMG_CTRL_OVRFLR_D4(overfill_r) |
          VEPU_REG_IN_IMG_CTRL_OVRFLB(overfill_b);
    H264E_HAL_SET_REG(reg, VEPU_REG_ENC_OVER_FILL_STRM_OFFSET, val);

    // When offset is zero row length should be total 16 aligned width
    val = VEPU_REG_IN_IMG_CHROMA_OFFSET(0)
          | VEPU_REG_IN_IMG_LUMA_OFFSET(0)
          | VEPU_REG_IN_IMG_CTRL_ROW_LEN(mb_w * 16);
    H264E_HAL_SET_REG(reg, VEPU_REG_INPUT_LUMA_INFO, val);

    val = VEPU_REG_CHECKPOINT_CHECK1(hw_cfg->cp_target[0])
          | VEPU_REG_CHECKPOINT_CHECK0(hw_cfg->cp_target[1]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHECKPOINT(0), val);

    val = VEPU_REG_CHECKPOINT_CHECK1(hw_cfg->cp_target[2])
          | VEPU_REG_CHECKPOINT_CHECK0(hw_cfg->cp_target[3]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHECKPOINT(1), val);

    val = VEPU_REG_CHECKPOINT_CHECK1(hw_cfg->cp_target[4])
          | VEPU_REG_CHECKPOINT_CHECK0(hw_cfg->cp_target[5]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHECKPOINT(2), val);

    val = VEPU_REG_CHECKPOINT_CHECK1(hw_cfg->cp_target[6])
          | VEPU_REG_CHECKPOINT_CHECK0(hw_cfg->cp_target[7]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHECKPOINT(3), val);

    val = VEPU_REG_CHECKPOINT_CHECK1(hw_cfg->cp_target[8])
          | VEPU_REG_CHECKPOINT_CHECK0(hw_cfg->cp_target[9]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHECKPOINT(4), val);

    val = VEPU_REG_CHKPT_WORD_ERR_CHK1(hw_cfg->target_error[0])
          | VEPU_REG_CHKPT_WORD_ERR_CHK0(hw_cfg->target_error[1]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHKPT_WORD_ERR(0), val);

    val = VEPU_REG_CHKPT_WORD_ERR_CHK1(hw_cfg->target_error[2])
          | VEPU_REG_CHKPT_WORD_ERR_CHK0(hw_cfg->target_error[3]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHKPT_WORD_ERR(1), val);

    val = VEPU_REG_CHKPT_WORD_ERR_CHK1(hw_cfg->target_error[4])
          | VEPU_REG_CHKPT_WORD_ERR_CHK0(hw_cfg->target_error[5]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHKPT_WORD_ERR(2), val);

    val = VEPU_REG_CHKPT_DELTA_QP_CHK6(hw_cfg->delta_qp[6])
          | VEPU_REG_CHKPT_DELTA_QP_CHK5(hw_cfg->delta_qp[5])
          | VEPU_REG_CHKPT_DELTA_QP_CHK4(hw_cfg->delta_qp[4])
          | VEPU_REG_CHKPT_DELTA_QP_CHK3(hw_cfg->delta_qp[3])
          | VEPU_REG_CHKPT_DELTA_QP_CHK2(hw_cfg->delta_qp[2])
          | VEPU_REG_CHKPT_DELTA_QP_CHK1(hw_cfg->delta_qp[1])
          | VEPU_REG_CHKPT_DELTA_QP_CHK0(hw_cfg->delta_qp[0]);
    H264E_HAL_SET_REG(reg, VEPU_REG_CHKPT_DELTA_QP, val);

    val = VEPU_REG_MAD_THRESHOLD(hw_cfg->mad_threshold)
          | VEPU_REG_IN_IMG_CTRL_FMT(hw_cfg->input_format)
          | VEPU_REG_IN_IMG_ROTATE_MODE(0)
          | VEPU_REG_SIZE_TABLE_PRESENT; //FIXED
    H264E_HAL_SET_REG(reg, VEPU_REG_ENC_CTRL1, val);

    val = VEPU_REG_INTRA16X16_MODE(h264_intra16_favor[hw_cfg->qp])
          | VEPU_REG_INTER_MODE(h264_inter_favor[hw_cfg->qp]);
    H264E_HAL_SET_REG(reg, VEPU_REG_INTRA_INTER_MODE, val);

    val = VEPU_REG_PPS_INIT_QP(pps->i_pic_init_qp)
          | VEPU_REG_SLICE_FILTER_ALPHA(hw_cfg->slice_alpha_offset)
          | VEPU_REG_SLICE_FILTER_BETA(hw_cfg->slice_beta_offset)
          | VEPU_REG_CHROMA_QP_OFFSET(pps->i_chroma_qp_index_offset)
          | VEPU_REG_IDR_PIC_ID(hw_cfg->idr_pic_id);

    if (hw_cfg->filter_disable)
        val |= VEPU_REG_FILTER_DISABLE;

    if (pps->b_constrained_intra_pred)
        val |= VEPU_REG_CONSTRAINED_INTRA_PREDICTION;
    H264E_HAL_SET_REG(reg, VEPU_REG_ENC_CTRL2, val);

    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_NEXT_PIC, 0);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_MV_OUT, 0);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_CABAC_TBL, mpp_buffer_get_fd(bufs->hw_cabac_table_buf));

    val = VEPU_REG_ROI1_TOP_MB(mb_h)
          | VEPU_REG_ROI1_BOTTOM_MB(mb_h)
          | VEPU_REG_ROI1_LEFT_MB(mb_w)
          | VEPU_REG_ROI1_RIGHT_MB(mb_w);
    H264E_HAL_SET_REG(reg, VEPU_REG_ROI1, val); //FIXED

    val = VEPU_REG_ROI2_TOP_MB(mb_h)
          | VEPU_REG_ROI2_BOTTOM_MB(mb_h)
          | VEPU_REG_ROI2_LEFT_MB(mb_w)
          | VEPU_REG_ROI2_RIGHT_MB(mb_w);
    H264E_HAL_SET_REG(reg, VEPU_REG_ROI2, val); //FIXED
    H264E_HAL_SET_REG(reg, VEPU_REG_STABLILIZATION_OUTPUT, 0);

    val = VEPU_REG_RGB2YUV_CONVERSION_COEFB(hw_cfg->color_conversion_coeff_b)
          | VEPU_REG_RGB2YUV_CONVERSION_COEFA(hw_cfg->color_conversion_coeff_a);
    H264E_HAL_SET_REG(reg, VEPU_REG_RGB2YUV_CONVERSION_COEF1, val); //FIXED

    val = VEPU_REG_RGB2YUV_CONVERSION_COEFE(hw_cfg->color_conversion_coeff_e)
          | VEPU_REG_RGB2YUV_CONVERSION_COEFC(hw_cfg->color_conversion_coeff_c);
    H264E_HAL_SET_REG(reg, VEPU_REG_RGB2YUV_CONVERSION_COEF2, val); //FIXED

    val = VEPU_REG_RGB2YUV_CONVERSION_COEFF(hw_cfg->color_conversion_coeff_f);
    H264E_HAL_SET_REG(reg, VEPU_REG_RGB2YUV_CONVERSION_COEF3, val); //FIXED

    val = VEPU_REG_RGB_MASK_B_MSB(hw_cfg->b_mask_msb)
          | VEPU_REG_RGB_MASK_G_MSB(hw_cfg->g_mask_msb)
          | VEPU_REG_RGB_MASK_R_MSB(hw_cfg->r_mask_msb);
    H264E_HAL_SET_REG(reg, VEPU_REG_RGB_MASK_MSB, val); //FIXED

    {
        RK_U32 diff_mv_penalty[3] = {0};
        diff_mv_penalty[0] = h264_diff_mv_penalty4p[hw_cfg->qp];
        diff_mv_penalty[1] = h264_diff_mv_penalty[hw_cfg->qp];
        diff_mv_penalty[2] = h264_diff_mv_penalty[hw_cfg->qp];

        val = VEPU_REG_1MV_PENALTY(diff_mv_penalty[1])
              | VEPU_REG_QMV_PENALTY(diff_mv_penalty[2])
              | VEPU_REG_4MV_PENALTY(diff_mv_penalty[0]);
    }

    val |= VEPU_REG_SPLIT_MV_MODE_EN;
    H264E_HAL_SET_REG(reg, VEPU_REG_MV_PENALTY, val);

    val = VEPU_REG_H264_LUMA_INIT_QP(hw_cfg->qp)
          | VEPU_REG_H264_QP_MAX(hw_cfg->qp_max)
          | VEPU_REG_H264_QP_MIN(hw_cfg->qp_min)
          | VEPU_REG_H264_CHKPT_DISTANCE(hw_cfg->cp_distance_mbs);
    H264E_HAL_SET_REG(reg, VEPU_REG_QP_VAL, val);

    val = VEPU_REG_ZERO_MV_FAVOR_D2(10);
    H264E_HAL_SET_REG(reg, VEPU_REG_MVC_RELATE, val);

    if (hw_cfg->input_format < H264E_VPU_CSP_RGB565) {
        val = VEPU_REG_OUTPUT_SWAP32
              | VEPU_REG_OUTPUT_SWAP16
              | VEPU_REG_OUTPUT_SWAP8
              | VEPU_REG_INPUT_SWAP8
              | VEPU_REG_INPUT_SWAP16
              | VEPU_REG_INPUT_SWAP32;
    } else if (hw_cfg->input_format == H264E_VPU_CSP_ARGB8888) {
        val = VEPU_REG_OUTPUT_SWAP32
              | VEPU_REG_OUTPUT_SWAP16
              | VEPU_REG_OUTPUT_SWAP8
              | VEPU_REG_INPUT_SWAP32;
    } else {
        val = VEPU_REG_OUTPUT_SWAP32
              | VEPU_REG_OUTPUT_SWAP16
              | VEPU_REG_OUTPUT_SWAP8
              | VEPU_REG_INPUT_SWAP16
              | VEPU_REG_INPUT_SWAP32;
    }
    H264E_HAL_SET_REG(reg, VEPU_REG_DATA_ENDIAN, val);

    val = VEPU_REG_PPS_ID(pps->i_id)
          | VEPU_REG_INTRA_PRED_MODE(h264_prev_mode_favor[hw_cfg->qp])
          | VEPU_REG_FRAME_NUM(hw_cfg->frame_num);
    H264E_HAL_SET_REG(reg, VEPU_REG_ENC_CTRL3, val);

    val = VEPU_REG_INTERRUPT_TIMEOUT_EN;
    H264E_HAL_SET_REG(reg, VEPU_REG_INTERRUPT, val);

    {
        RK_U8 dmv_penalty[128] = {0};
        RK_U8 dmv_qpel_penalty[128] = {0};

        for (i = 0; i < 128; i++) {
            dmv_penalty[i] = i;
            dmv_qpel_penalty[i] = H264E_HAL_MIN(255, exp_golomb_signed(i));
        }

        for (i = 0; i < 128; i += 4) {
            val = VEPU_REG_DMV_PENALTY_TABLE_BIT(dmv_penalty[i], 3);
            val |= VEPU_REG_DMV_PENALTY_TABLE_BIT(dmv_penalty[i + 1], 2);
            val |= VEPU_REG_DMV_PENALTY_TABLE_BIT(dmv_penalty[i + 2], 1);
            val |= VEPU_REG_DMV_PENALTY_TABLE_BIT(dmv_penalty[i + 3], 0);
            H264E_HAL_SET_REG(reg, VEPU_REG_DMV_PENALTY_TBL(i / 4), val);

            val = VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
                      dmv_qpel_penalty[i], 3);
            val |= VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
                       dmv_qpel_penalty[i + 1], 2);
            val |= VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
                       dmv_qpel_penalty[i + 2], 1);
            val |= VEPU_REG_DMV_Q_PIXEL_PENALTY_TABLE_BIT(
                       dmv_qpel_penalty[i + 3], 0);
            H264E_HAL_SET_REG(reg, VEPU_REG_DMV_Q_PIXEL_PENALTY_TBL(i / 4), val);
        }
    }

    /* set buffers addr */
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_IN_LUMA, hw_cfg->input_luma_addr);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_IN_CB, hw_cfg->input_cb_addr);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_IN_CR, hw_cfg->input_cr_addr);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_OUTPUT_STREAM, hw_cfg->output_strm_addr);
    H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_OUTPUT_CTRL, mpp_buffer_get_fd(bufs->hw_nal_size_table_buf));

    {
        RK_S32 recon_chroma_addr = 0, ref_chroma_addr = 0;
        RK_U32 frame_luma_size = mb_h * mb_w * 256;
        RK_S32 recon_luma_addr = mpp_buffer_get_fd(recn);
        RK_S32 ref_luma_addr = mpp_buffer_get_fd(refr);

        recon_chroma_addr = recon_luma_addr | (frame_luma_size << 10);
        ref_chroma_addr   = ref_luma_addr   | (frame_luma_size << 10);

        H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_REC_LUMA, recon_luma_addr);
        H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_REC_CHROMA, recon_chroma_addr);
        H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_REF_LUMA, ref_luma_addr);
        H264E_HAL_SET_REG(reg, VEPU_REG_ADDR_REF_CHROMA , ref_chroma_addr);
    }


    /* set important encode mode info */
    val = VEPU_REG_MB_HEIGHT(mb_h)
          | VEPU_REG_MB_WIDTH(mb_w)
          | VEPU_REG_PIC_TYPE(hw_cfg->frame_type)
          | VEPU_REG_ENCODE_FORMAT(3)
          | VEPU_REG_ENCODE_ENABLE;
    H264E_HAL_SET_REG(reg, VEPU_REG_ENCODE_START, val);

    ctx->frame_cnt++;
    hw_cfg->frame_num++;
    if (hw_cfg->frame_type == H264E_VPU_FRAME_I)
        hw_cfg->idr_pic_id++;
    if (hw_cfg->idr_pic_id == 16)
        hw_cfg->idr_pic_id = 0;

    h264e_hal_leave();
    return MPP_OK;
}

MPP_RET hal_h264e_vepu2_start(void *hal, HalTaskInfo *task)
{
    MPP_RET ret = MPP_OK;
    H264eHalContext *ctx = (H264eHalContext *)hal;
    (void)task;

    h264e_hal_enter();

    if (ctx->dev_ctx) {
        RK_U32 *p_regs = (RK_U32 *)ctx->regs;
        h264e_hal_dbg(H264E_DBG_DETAIL, "vpu client is sending %d regs", VEPU2_H264E_NUM_REGS);
        ret = mpp_device_send_reg(ctx->dev_ctx, p_regs, VEPU2_H264E_NUM_REGS);
        if (ret)
            mpp_err("mpp_device_send_reg failed ret %d", ret);
        else
            h264e_hal_dbg(H264E_DBG_DETAIL, "mpp_device_send_reg success!");
    } else {
        mpp_err("invalid device ctx: %p", ctx->dev_ctx);
        ret = MPP_NOK;
    }

    h264e_hal_leave();

    return ret;
}

static MPP_RET h264e_vpu_set_feedback(h264e_feedback *fb, void *reg)
{
    RK_S32 i = 0;
    RK_U32 cpt_prev = 0, overflow = 0;
    RK_U32 cpt_idx = VEPU_REG_CHECKPOINT(0) / 4;
    RK_U32 *reg_val = (RK_U32 *)reg;
    fb->hw_status = reg_val[VEPU_REG_INTERRUPT / 4];
    fb->qp_sum = ((reg_val[VEPU_REG_QP_SUM_DIV2 / 4] >> 11) & 0x001fffff) * 2;
    fb->mad_count = (reg_val[VEPU_REG_MB_CTRL / 4] >> 16) & 0xffff;
    fb->rlc_count = reg_val[VEPU_REG_RLC_SUM / 4] & 0x3fffff;
    fb->out_strm_size = reg_val[VEPU_REG_STR_BUF_LIMIT / 4] / 8;

    for (i = 0; i < CHECK_POINTS_MAX; i++) {
        RK_U32 cpt = VEPU_REG_CHECKPOINT_RESULT(reg_val[cpt_idx]);
        if (cpt < cpt_prev)
            overflow += (1 << 21);
        cpt_prev = cpt;
        fb->cp[i] = cpt + overflow;
        cpt_idx += (i & 1);
    }

    return MPP_OK;
}
static MPP_RET hal_h264e_vpu2_resend(void *hal, RK_U32 *reg_out, RK_S32 dealt_qp)
{

    H264eHalContext *ctx = (H264eHalContext *)hal;
    RK_U32 *p_regs = (RK_U32 *)ctx->regs;
    H264eHwCfg *hw_cfg = &ctx->hw_cfg;
    RK_U32 val = 0;
    RK_S32 hw_ret = 0;
    hw_cfg->qp += dealt_qp;
    hw_cfg->qp = mpp_clip(hw_cfg->qp, hw_cfg->qp_min, hw_cfg->qp_max);

    val = VEPU_REG_H264_LUMA_INIT_QP(hw_cfg->qp)
          | VEPU_REG_H264_QP_MAX(hw_cfg->qp_max)
          | VEPU_REG_H264_QP_MIN(hw_cfg->qp_min)
          | VEPU_REG_H264_CHKPT_DISTANCE(hw_cfg->cp_distance_mbs);

    H264E_HAL_SET_REG(p_regs, VEPU_REG_QP_VAL, val);

    {
        RK_U32 diff_mv_penalty[3] = {0};
        diff_mv_penalty[0] = h264_diff_mv_penalty4p[hw_cfg->qp];
        diff_mv_penalty[1] = h264_diff_mv_penalty[hw_cfg->qp];
        diff_mv_penalty[2] = h264_diff_mv_penalty[hw_cfg->qp];

        val = VEPU_REG_1MV_PENALTY(diff_mv_penalty[1])
              | VEPU_REG_QMV_PENALTY(diff_mv_penalty[2])
              | VEPU_REG_4MV_PENALTY(diff_mv_penalty[0]);
    }

    val |= VEPU_REG_SPLIT_MV_MODE_EN;
    H264E_HAL_SET_REG(p_regs, VEPU_REG_MV_PENALTY, val);

    hw_ret = mpp_device_send_reg(ctx->dev_ctx, p_regs, VEPU2_H264E_NUM_REGS);
    if (hw_ret)
        mpp_err("mpp_device_send_reg failed ret %d", hw_ret);
    else
        h264e_hal_dbg(H264E_DBG_DETAIL, "mpp_device_send_reg success!");

    hw_ret = mpp_device_wait_reg(ctx->dev_ctx, (RK_U32 *)reg_out, VEPU2_H264E_NUM_REGS);
    if (hw_ret) {
        h264e_hal_err("hardware returns error:%d", hw_ret);
        return MPP_ERR_VPUHW;
    }
    return MPP_OK;
}

#define START_CODE 0x000001 ///< start_code_prefix_one_3bytes
static RK_S32 get_next_nal(RK_U8 *buf, RK_S32 *length)
{
    RK_S32 i, consumed = 0;
    RK_S32 len = *length;
    RK_U8 *tmp_buf = buf;
    /* search start code */
    while (len >= 4) {
        if (tmp_buf[2] == 0) {
            len--;
            tmp_buf++;
            continue;
        }
        if (tmp_buf[0] != 0 || tmp_buf[1] != 0 || tmp_buf[2] != 1) {
            RK_U32 state = (RK_U32) - 1;
            RK_S32 has_nal = 0;
            for (i = 0; i < (RK_S32)len; i++) {
                state = (state << 8) | tmp_buf[i];
                if (((state >> 8) & 0xFFFFFF) == START_CODE) {
                    has_nal = 1;
                    i = i - 3;
                    break;
                }
            }

            if (has_nal) {
                len -= i;
                tmp_buf += i;
                consumed = *length - len - 1;
                break;
            }
            consumed = *length;
            break;
        }
        tmp_buf   += 3;
        len       -= 3;
    }
    *length = *length - consumed;
    return consumed;
}

MPP_RET reorganize_hwstream2tsvc(H264eHalContext *ctx, HalTaskInfo *task)
{
    MppBuffer buf = task->enc.output;
    h264e_feedback *fb = &ctx->feedback;
    RK_U8 *p = mpp_buffer_get_ptr(buf);
    RK_S32 len = task->enc.length;
    RK_S32 size = mpp_buffer_get_size(buf);
    RK_S32 hw_len_bit = 0;
    RK_S32 sw_len_bit = 0;
    RK_S32 hw_len_byte = 0;
    RK_S32 sw_len_byte = 0;
    RK_S32 diff_size = 0;
    RK_S32 tail_0bit = 0;
    RK_U8  tail_byte = 0;
    RK_U8  tail_tmp = 0;
    RK_U8 *dst_buf = NULL;
    RK_S32 buf_size;
    RK_S32 prefix_bit = 0;
    RK_S32 prefix_byte = 0;
    RK_U32 final_len = 0;
    {
        RK_S32 more_buf = 0;

        while (len > ctx->buf_size - 16) {
            ctx->buf_size *= 2;
            more_buf = 1;
        }

        if (more_buf) {
            MPP_FREE(ctx->src_buf);
            MPP_FREE(ctx->dst_buf);
            ctx->src_buf = mpp_malloc(RK_U8, ctx->buf_size);
            ctx->dst_buf = mpp_malloc(RK_U8, ctx->buf_size);
        }
    }

    memset(ctx->dst_buf, 0, ctx->buf_size);
    memset(ctx->src_buf, 0, ctx->buf_size);
    dst_buf = ctx->dst_buf;
    buf_size = ctx->buf_size;
    ctx->slice.last_slice_flag = 0;
    ctx->slice.first_slice_flag = 1;
    do {
        RK_U32 nal_len = 0;
        tail_0bit = 0;
        // copy hw stream to stream buffer first
        if (!ctx->hw_cfg.slice_size_mb_rows) {
            memcpy(ctx->src_buf, p, len);
            nal_len = len;
            ctx->slice.last_slice_flag = 1;
        } else {
            nal_len = get_next_nal(p, &len);
            memcpy(ctx->src_buf, p, nal_len);
            p += nal_len;
            if (len == 0) {
                ctx->slice.last_slice_flag = 1;
            }
            h264e_dpb_slice("nal_len = %d, lastbyte %1x", nal_len, ctx->src_buf[nal_len - 1]);
        }

        hw_len_bit = h264e_slice_read(&ctx->slice, ctx->src_buf, size);

        // update slice information
        h264e_slice_update(&ctx->slice, ctx->dpb);
        if (ctx->slice.first_slice_flag) {
            if (ctx->svc) {
                H264ePrefixNal prefix;

                prefix.idr_flag = ctx->slice.idr_flag;
                prefix.nal_ref_idc = ctx->slice.nal_reference_idc;
                prefix.priority_id = 0;
                prefix.no_inter_layer_pred_flag = 1;
                prefix.dependency_id = 0;
                prefix.quality_id = 0;
                prefix.temporal_id = task->enc.temporal_id;
                prefix.use_ref_base_pic_flag = 0;
                prefix.discardable_flag = 0;
                prefix.output_flag = 1;

                prefix_bit = h264e_slice_write_prefix_nal_unit_svc(&prefix, dst_buf, buf_size);
                prefix_byte = prefix_bit /= 8;
                h264e_dpb_slice("prefix_len %d\n", prefix_byte);
                dst_buf += prefix_byte;
                buf_size -= prefix_byte;
            }
            ctx->slice.first_slice_flag = 0;
        }

        ctx->slice.frame_num = ctx->dpb->curr->frame_num;
        ctx->slice.pic_order_cnt_lsb = ctx->dpb->curr->poc;
        // write new header to header buffer
        sw_len_bit = h264e_slice_write(&ctx->slice, dst_buf, buf_size);

        hw_len_byte = (hw_len_bit + 7) / 8;
        sw_len_byte = (sw_len_bit + 7) / 8;

        tail_byte = ctx->src_buf[nal_len - 1];
        tail_tmp = tail_byte;

        while (!(tail_tmp & 1) && tail_0bit < 8) {
            tail_tmp >>= 1;
            tail_0bit++;
        }

        mpp_assert(tail_0bit < 8);

        // move the reset slice data from src buffer to dst buffer
        diff_size = h264e_slice_move(dst_buf, ctx->src_buf,
                                     sw_len_bit, hw_len_bit, nal_len);

        h264e_dpb_slice("tail 0x%02x %d hw_hdr %d sw_hdr %d len %d hw_byte %d sw_byte %d diff %d\n",
                        tail_byte, tail_0bit, hw_len_bit, sw_len_bit, nal_len, hw_len_byte, sw_len_byte, diff_size);

        if (ctx->slice.entropy_coding_mode) {
            memcpy(dst_buf + sw_len_byte, ctx->src_buf + hw_len_byte,
                   nal_len - hw_len_byte);
            final_len += nal_len - hw_len_byte + sw_len_byte;
            nal_len = nal_len - hw_len_byte + sw_len_byte;
        } else {
            RK_S32 hdr_diff_bit = sw_len_bit - hw_len_bit;
            RK_S32 bit_len = nal_len * 8 - tail_0bit + hdr_diff_bit;
            RK_S32 new_len = (bit_len + diff_size * 8 + 7) / 8;

            dst_buf[new_len] = 0;

            h264e_dpb_slice("frm %4d %c len %d bit hw %d sw %d byte hw %d sw %d diff %d -> %d\n",
                            ctx->dpb->curr->frm_cnt, (ctx->slice.idr_flag ? 'I' : 'P'),
                            nal_len, hw_len_bit, sw_len_bit,
                            hw_len_byte, sw_len_byte, diff_size, new_len);

            h264e_dpb_slice("%02x %02x %02x %02x -> %02x %02x %02x %02x\n",
                            ctx->src_buf[nal_len - 4], ctx->src_buf[nal_len - 3],
                            ctx->src_buf[nal_len - 2], ctx->src_buf[nal_len - 1],
                            dst_buf[new_len - 4], dst_buf[new_len - 3],
                            dst_buf[new_len - 2], dst_buf[new_len - 1]);
            nal_len = new_len;
            final_len += new_len;
        }

        if (!ctx->hw_cfg.slice_size_mb_rows || !len) {
            p = mpp_buffer_get_ptr(buf);
            final_len += prefix_byte;
            memcpy(p, ctx->dst_buf, final_len);
            if (ctx->slice.entropy_coding_mode) {
                if (final_len < task->enc.length) {
                    memset(p + final_len, 0,  task->enc.length - final_len);
                }
            } else {
                p[final_len] = 0;
            }
            break;
        }
        dst_buf += nal_len;
        buf_size -= nal_len;
    } while (1);

    h264e_dpb_curr_ready(ctx->dpb);

    fb->out_strm_size = final_len;
    task->enc.length = fb->out_strm_size;
    return MPP_OK;
}

MPP_RET hal_h264e_vepu2_wait(void *hal, HalTaskInfo *task)
{
    H264eHalContext *ctx = (H264eHalContext *)hal;
    H264eVpu2RegSet reg_out_tmp;
    H264eVpu2RegSet *reg_out = &reg_out_tmp;
    MppEncPrepCfg *prep = &ctx->set->prep;
    IOInterruptCB int_cb = ctx->int_cb;
    h264e_feedback *fb = &ctx->feedback;
    RcSyntax *rc_syn = (RcSyntax *)task->enc.syntax.data;
    H264eHwCfg *hw_cfg = &ctx->hw_cfg;
    RK_S32 num_mb = MPP_ALIGN(prep->width, 16) * MPP_ALIGN(prep->height, 16) / 16 / 16;

    memset(reg_out, 0, sizeof(H264eVpu2RegSet));
    h264e_hal_enter();

    if (ctx->dev_ctx) {
        RK_S32 hw_ret = mpp_device_wait_reg(ctx->dev_ctx, (RK_U32 *)reg_out,
                                            VEPU2_H264E_NUM_REGS);

        h264e_hal_dbg(H264E_DBG_DETAIL, "mpp_device_wait_reg: ret %d\n", hw_ret);

        if (hw_ret != MPP_OK) {
            mpp_err("hardware returns error:%d", hw_ret);
            return MPP_ERR_VPUHW;
        }
    } else {
        mpp_err("invalid device ctx: %p", ctx->dev_ctx);
        return MPP_NOK;
    }

    h264e_vpu_set_feedback(fb, (void*)reg_out);

    task->enc.length = fb->out_strm_size;
    hw_cfg->qp_prev = hw_cfg->qp;

    h264e_check_reencode( hal, task, (void*)reg_out,
                          hal_h264e_vpu2_resend,  h264e_vpu_set_feedback);
    if (ctx->usr_hier) {
        reorganize_hwstream2tsvc(ctx, task);
    }
    if (int_cb.callBack) {
        RcSyntax *syn = (RcSyntax *)task->enc.syntax.data;
        RcHalResult result;
        RK_S32 i;
        RK_S32 avg_qp = fb->qp_sum / num_mb;

        mpp_assert(avg_qp >= 0);
        mpp_assert(avg_qp <= 51);

        avg_qp = hw_cfg->qp;

        result.bits = fb->out_strm_size * 8;
        result.type = syn->type;
        fb->result = &result;
        hw_cfg->qpCtrl.nonZeroCnt = fb->rlc_count;
        hw_cfg->qpCtrl.frameBitCnt = result.bits;
        hw_cfg->pre_bit_diff = result.bits - syn->bit_target;
        h264e_vpu_update_result(hw_cfg, result.bits, rc_syn->bit_target);

        if (ctx->tsvc_rc.tlayer_num > 1)
            h264e_vpu_tsvcrc_update(&ctx->tsvc_rc, result.bits, avg_qp);

        if (syn->type == INTER_P_FRAME || syn->gop_mode == MPP_GOP_ALL_INTRA) {
            mpp_data_update(ctx->qp_p, avg_qp);
        }

        for (i = 0; i < CHECK_POINTS_MAX; i++) {
            hw_cfg->qpCtrl.wordCntPrev[i] = fb->cp[i];
        }

        mpp_save_regdata((syn->type == INTRA_FRAME) ?
                         (ctx->intra_qs) :
                         (ctx->inter_qs),
                         h264_q_step[avg_qp], result.bits);

        mpp_linreg_update((syn->type == INTRA_FRAME) ?
                          (ctx->intra_qs) :
                          (ctx->inter_qs));

        h264e_vpu_mad_threshold(hw_cfg, ctx->mad, fb->mad_count);
        int_cb.callBack(int_cb.opaque, fb);
    }

    //h264e_vpu_dump_mpp_strm_out(ctx, NULL);

    if (!(ctx->hdr_status & HDR_OUTPUTED)) {
        RK_U32 hdr_len = mpp_packet_get_length(ctx->packeted_param);
        RK_U8 *hdr_ptr = mpp_packet_get_data(ctx->packeted_param);
        RK_U32 len = task->enc.length;
        RK_U8 *p = mpp_buffer_get_ptr(task->enc.output);
        RK_U8 *d = mpp_malloc(RK_U8, len);

        memcpy(d, p, len);
        memcpy(p, hdr_ptr, hdr_len);
        memcpy(p + hdr_len, d, len);

        MPP_FREE(d);
        ctx->hdr_status |= HDR_OUTPUTED;
        task->enc.length += hdr_len;
    }

    h264e_hal_leave();

    return MPP_OK;
}

MPP_RET hal_h264e_vepu2_reset(void *hal)
{
    (void)hal;
    h264e_hal_enter();

    h264e_hal_leave();
    return MPP_OK;
}

MPP_RET hal_h264e_vepu2_flush(void *hal)
{
    (void)hal;
    h264e_hal_enter();

    h264e_hal_leave();
    return MPP_OK;
}

MPP_RET hal_h264e_vepu2_control(void *hal, RK_S32 cmd_type, void *param)
{
    H264eHalContext *ctx = (H264eHalContext *)hal;
    MPP_RET ret = MPP_OK;
    h264e_hal_enter();

    h264e_hal_dbg(H264E_DBG_DETAIL, "h264e_vpu_control cmd 0x%08x, info %p", cmd_type, param);
    switch (cmd_type) {
    case MPP_ENC_SET_EXTRA_INFO: {
    } break;
    case MPP_ENC_GET_EXTRA_INFO: {
        MppPacket  pkt      = ctx->packeted_param;
        MppPacket *pkt_out  = (MppPacket *)param;

        H264eVpuExtraInfo *src = (H264eVpuExtraInfo *)ctx->extra_info;
        H264eStream *sps_stream = &src->sps_stream;
        H264eStream *pps_stream = &src->pps_stream;
        H264eStream *sei_stream = &src->sei_stream;

        size_t offset = 0;

        h264e_vpu_set_extra_info(ctx);

        mpp_packet_write(pkt, offset, sps_stream->buffer, sps_stream->byte_cnt);
        offset += sps_stream->byte_cnt;

        mpp_packet_write(pkt, offset, pps_stream->buffer, pps_stream->byte_cnt);
        offset += pps_stream->byte_cnt;

        mpp_packet_write(pkt, offset, sei_stream->buffer, sei_stream->byte_cnt);
        offset += sei_stream->byte_cnt;

        mpp_packet_set_length(pkt, offset);

        *pkt_out = pkt;
        ctx->hdr_status = HDR_UPDATED | HDR_OUTPUTED;
    } break;
    case MPP_ENC_SET_PREP_CFG : {
        MppEncPrepCfg *set = &ctx->set->prep;
        RK_U32 change = set->change;

        if (change & MPP_ENC_PREP_CFG_CHANGE_INPUT) {
            if ((set->width < 0 || set->width > 1920) ||
                (set->height < 0 || set->height > 3840) ||
                (set->hor_stride < 0 || set->hor_stride > 3840) ||
                (set->ver_stride < 0 || set->ver_stride > 3840)) {
                mpp_err("invalid input w:h [%d:%d] [%d:%d]\n",
                        set->width, set->height,
                        set->hor_stride, set->ver_stride);
                ret = MPP_NOK;
            }
        }

        if (change & MPP_ENC_PREP_CFG_CHANGE_FORMAT) {
            if ((set->format < MPP_FRAME_FMT_RGB &&
                 set->format >= MPP_FMT_YUV_BUTT) ||
                set->format >= MPP_FMT_RGB_BUTT) {
                mpp_err("invalid format %d\n", set->format);
                ret = MPP_NOK;
            }
        }
    } break;
    case MPP_ENC_SET_RC_CFG : {
        // TODO: do rate control check here
    } break;
    case MPP_ENC_SET_CODEC_CFG : {
        MppEncH264Cfg *src = &ctx->set->codec.h264;
        MppEncH264Cfg *dst = &ctx->cfg->codec.h264;
        RK_U32 change = src->change;

        // TODO: do codec check first

        if (change & MPP_ENC_H264_CFG_STREAM_TYPE)
            dst->stream_type = src->stream_type;
        if (change & MPP_ENC_H264_CFG_CHANGE_PROFILE) {
            dst->svc = src->svc;
            dst->profile = src->profile;
            dst->level = src->level;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_ENTROPY) {
            dst->entropy_coding_mode = src->entropy_coding_mode;
            dst->cabac_init_idc = src->cabac_init_idc;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_TRANS_8x8)
            dst->transform8x8_mode = src->transform8x8_mode;
        if (change & MPP_ENC_H264_CFG_CHANGE_CONST_INTRA)
            dst->constrained_intra_pred_mode = src->constrained_intra_pred_mode;
        if (change & MPP_ENC_H264_CFG_CHANGE_CHROMA_QP) {
            dst->chroma_cb_qp_offset = src->chroma_cb_qp_offset;
            dst->chroma_cr_qp_offset = src->chroma_cr_qp_offset;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_DEBLOCKING) {
            dst->deblock_disable = src->deblock_disable;
            dst->deblock_offset_alpha = src->deblock_offset_alpha;
            dst->deblock_offset_beta = src->deblock_offset_beta;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_LONG_TERM)
            dst->use_longterm = src->use_longterm;
        if (change & MPP_ENC_H264_CFG_CHANGE_QP_LIMIT) {
            dst->qp_init = src->qp_init;
            dst->qp_max = src->qp_max;
            dst->qp_min = src->qp_min;
            dst->qp_max_step = src->qp_max_step;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_INTRA_REFRESH) {
            dst->intra_refresh_mode = src->intra_refresh_mode;
            dst->intra_refresh_arg = src->intra_refresh_arg;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_SLICE_MODE) {
            dst->slice_mode = src->slice_mode;
            dst->slice_arg = src->slice_arg;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_VUI) {
            dst->vui = src->vui;
        }
        if (change & MPP_ENC_H264_CFG_CHANGE_SEI) {
            dst->sei = src->sei;
        }

        /*
         * NOTE: use OR here for avoiding overwrite on multiple config
         * When next encoding is trigger the change flag will be clear
         */
        dst->change |= change;
        src->change = 0;
    } break;
    case MPP_ENC_SET_OSD_PLT_CFG:
    case MPP_ENC_SET_OSD_DATA_CFG: {
        mpp_err("vepu2 do not support osd cfg\n");
        ret = MPP_NOK;
    } break;
    case MPP_ENC_SET_SEI_CFG: {
        ctx->sei_mode = *((MppEncSeiMode *)param);
    } break;
    case MPP_ENC_SET_ROI_CFG: {
        mpp_err("vepu2 do not support roi cfg\n");
        ret = MPP_NOK;
    } break;
    case MPP_ENC_PRE_ALLOC_BUFF:
        // vepu do not support prealloc buff, ignore cmd
        break;
    case MPP_ENC_SET_GOPREF: {
        MppEncGopRef *ref = (MppEncGopRef *)param;
        ctx->cfg->misc.gop_ref = *ref;

        ctx->hdr_status = HDR_NEED_UPDATED;

        if (!ref->gop_cfg_enable) {
            ctx->usr_hier = 0;
            break;
        }

        if (ref->ref_gop_len > MAX_GOP_REF_LEN) {
            mpp_err("ref gop length %d is too large\n", ref->ref_gop_len);
            ret = MPP_NOK;
            break;
        }

        RK_S32 max_layer_id = 0;
        MppEncHierCfg *hier = &ctx->hier_cfg;

        hier->length = ref->ref_gop_len;
        hier->lt_ref_interval = ref->lt_ref_interval;
        hier->max_lt_ref_cnt = ref->max_lt_ref_cnt;

        h264e_dpb_dbg("ref_gop_len %d LTR interval %d max_lt_ref_idx_p1 %d\n",
                      ref->ref_gop_len, hier->lt_ref_interval,
                      hier->max_lt_ref_cnt);

        if (hier->lt_ref_interval)
            mpp_assert(hier->max_lt_ref_cnt > 0);

        RK_S32 i;
        RK_S32 pos = 0;
        char *fmt = hier->ref_fmt;
        size_t size = sizeof(hier->ref_fmt);
        RK_S32 max_lt_ref_idx = -1;

        for (i = 0; i < hier->length + 1; i++) {
            RK_S32 is_non_ref = ref->gop_info[i].is_non_ref;
            RK_S32 is_lt_ref = ref->gop_info[i].is_lt_ref;

            if (max_layer_id < ref->gop_info[i].temporal_id) {
                max_layer_id = ref->gop_info[i].temporal_id;
            }


            hier->ref_idx[i] = ref->gop_info[i].ref_idx;

            pos += snprintf(fmt + pos, size - pos, "%s%d", (i) ? "P" : "I", i);
            pos += snprintf(fmt + pos, size - pos, "%s",
                            (is_non_ref) ? "N" : (is_lt_ref) ? "L" : "S");

            if (is_lt_ref) {
                pos += snprintf(fmt + pos, size - pos, "%d", ref->gop_info[i].lt_idx);
                if (ref->gop_info[i].lt_idx > max_lt_ref_idx)
                    max_lt_ref_idx = ref->gop_info[i].lt_idx;
            }

            h264e_vpu_tsvc_rc_init(&ctx->tsvc_rc, max_layer_id);

            pos += snprintf(fmt + pos, size - pos, "T%d", ref->gop_info[i].temporal_id);

            h264e_dpb_dbg("pos %d fmt %s \n", pos, fmt);
        }

        if (max_lt_ref_idx + 1 > hier->max_lt_ref_cnt) {
            mpp_err("mismatch max_lt_ref_idx_p1 %d vs %d\n",
                    max_lt_ref_idx + 1, hier->max_lt_ref_cnt);
        }

        if (hier->lt_ref_interval && max_lt_ref_idx >= 0)
            mpp_err("Can NOT use both lt_ref_interval and gop_info lt_ref at the same time!\n");

        ctx->usr_hier = 1;
    } break;
    case MPP_ENC_SET_SPLIT: {
        MppEncSliceSplit *split = (MppEncSliceSplit *)param;
        mpp_log("MPP_ENC_SET_SPLIT");
        ctx->cfg->misc.split = *split;
        ret = MPP_OK;
    } break;
    default : {
        mpp_err("unrecognizable cmd type %x", cmd_type);
        ret = MPP_NOK;
    } break;
    }

    h264e_hal_leave();
    return ret;
}
