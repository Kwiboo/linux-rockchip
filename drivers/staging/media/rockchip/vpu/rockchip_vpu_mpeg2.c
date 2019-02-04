// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VPU codec driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 */

#include "rockchip_vpu.h"

static const u8 zigzag[64] = {
	0,   1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63
};

void rockchip_vpu_mpeg2_dec_copy_qtable(u8 *qtable,
	const struct v4l2_ctrl_mpeg2_quantization *ctrl)
{
	int i, n;

	if (!qtable || !ctrl)
		return;

	for (i = 0; i < 64; i++) {
		n = zigzag[i];
		qtable[n + 0] = ctrl->intra_quantiser_matrix[i];
		qtable[n + 64] = ctrl->non_intra_quantiser_matrix[i];
		qtable[n + 128] = ctrl->chroma_intra_quantiser_matrix[i];
		qtable[n + 192] = ctrl->chroma_non_intra_quantiser_matrix[i];
	}
}

void rockchip_vpu_mpeg2_dec_start(struct rockchip_vpu_ctx *ctx)
{
	struct rockchip_vpu_dev *vpu = ctx->dev;

	ctx->dir_mv_size = 8160 * 4 * 4 * 4 * 4;
	ctx->dir_mv_buf =
		dma_zalloc_coherent(vpu->dev, ctx->dir_mv_size,
				    &ctx->dir_mv_dma_addr, GFP_KERNEL);

	ctx->qtable_size = 64 * 4;
	ctx->qtable_buf =
		dma_zalloc_coherent(vpu->dev, ctx->qtable_size,
				    &ctx->qtable_dma_addr, GFP_KERNEL);
}

void rockchip_vpu_mpeg2_dec_stop(struct rockchip_vpu_ctx *ctx)
{
	struct rockchip_vpu_dev *vpu = ctx->dev;

	dma_free_coherent(vpu->dev, ctx->dir_mv_size,
			  ctx->dir_mv_buf,
			  ctx->dir_mv_dma_addr);

	dma_free_coherent(vpu->dev, ctx->qtable_size,
			  ctx->qtable_buf,
			  ctx->qtable_dma_addr);
}
