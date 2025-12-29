/*
 * Copyright (C) 2015 The FFmpeg project
 * Copyright (c) 2015 Rockchip Electronics Co., Ltd.
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

#ifndef H265D_CODEC_H
#define H265D_CODEC_H

#include "mpp_frame.h"
#include "mpp_soc.h"
#include "mpp_dec_cfg.h"

typedef struct MppRational {
    RK_S32 num; ///< numerator
    RK_S32 den; ///< denominator
} MppRational_t;

enum MppPictureStructure {
    MPP_PICTURE_STRUCTURE_UNKNOWN,      //< unknown
    MPP_PICTURE_STRUCTURE_TOP_FIELD,    //< coded as top field
    MPP_PICTURE_STRUCTURE_BOTTOM_FIELD, //< coded as bottom field
    MPP_PICTURE_STRUCTURE_FRAME,        //< coded as frame
};

#define END_NOT_FOUND (-100)
#define MPP_PARSER_PTS_NB 4

typedef struct SplitContext {
    RK_U8 *buffer;
    RK_U32 buffer_size;
    RK_S32 index;
    RK_S32 last_index;
    RK_U32 state;             ///< contains the last few bytes in MSB order
    RK_S32 frame_start_found;
    RK_S32 overread;               ///< the number of bytes which where irreversibly read from the next frame
    RK_S32 overread_index;         ///< the index into ParseContext.buffer of the overread bytes
    RK_U64 state64;           ///< contains the last 8 bytes in MSB order
    RK_S64 pts;     /* pts of the current frame */
    RK_S64 dts;     /* dts of the current frame */
    RK_S64 frame_offset; /* offset of the current frame */
    RK_S64 cur_offset; /* current offset
                           (incremented by each av_parser_parse()) */
    RK_S64 next_frame_offset; /* offset of the next frame */
    /* private data */
    RK_S64 last_pts;
    RK_S64 last_dts;
    RK_S32 fetch_timestamp;

    RK_S32 cur_frame_start_index;
    RK_S64 cur_frame_offset[MPP_PARSER_PTS_NB];
    RK_S64 cur_frame_pts[MPP_PARSER_PTS_NB];
    RK_S64 cur_frame_dts[MPP_PARSER_PTS_NB];

    RK_S64 offset;      ///< byte offset from starting packet start
    RK_S64 cur_frame_end[MPP_PARSER_PTS_NB];
    /**
     * Set by parser to 1 for key frames and 0 for non-key frames.
     * It is initialized to -1, so if the parser doesn't set this flag,
     * old-style fallback using AV_PICTURE_TYPE_I picture type as key frames
     * will be used.
     */
    RK_S32 key_frame;
    RK_S32 eos;
} SplitContext_t;

typedef struct H265dContext {

    void *priv_data;

    void *split_cxt;

    /**
    * for rk log printf
    **/
    // RK_LOG_CONTEX_t *log_ctx;
    /**
     * display width & height
    **/
    RK_S32 width, height;

    /**
     *codec decoder width & height
    **/
    RK_S32 coded_width, coded_height;

    RK_U8 *extradata;

    RK_U32 extradata_size;

    /**
     * Pixel format
    **/
    RK_U32 pix_fmt;

    RK_U32 nBitDepth;
    RK_U32 err_recognition;

    /**
         * sample aspect ratio (0 if unknown)
         * That is the width of a pixel divided by the height of the pixel.
         * Numerator and denominator must be relatively prime and smaller than 256 for some video standards.
         * - decoding: Set by rkcodec.
        */
    MppRational_t sample_aspect_ratio;

    /**
     * YUV colorspace type.
     * - decoding: Set by rkcodec
     */
    MppFrameColorSpace colorspace;

    /**
     * MPEG vs JPEG YUV range.
     * - decoding: Set by rkcodec
     */
    MppFrameColorRange color_range;

    void *compare_info;

    MppDecCfgSet *cfg;

    const MppDecHwCap *hw_info;
} H265dContext_t;
#ifdef  __cplusplus
extern "C" {
#endif

RK_S32 h265d_parser2_syntax(void *ctx);
RK_S32 h265d_syntax_fill_slice(void *ctx, RK_S32 input_index);

#ifdef  __cplusplus
}
#endif

#endif /* H265D_CODEC_H */
