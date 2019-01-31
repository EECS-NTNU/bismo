#ifndef SLIDINGWINDOW_H
#define SLIDINGWINDOW_H

//Input stream instruction encoding based on example (32 : 16 - 8 - 4 - 4)
// 15 - 0 Base Addr
// 23 - 16 Image Size
// 27 - 24 Kernel Size
// 31 - 28 Stride

#define BASE_ADDR_BITWIDTH 16
#define IMG_SIZE_BITWIDTH 8
#define KRNL_SIZE_BITWIDTH 4
#define STRIDE_BITWIDTH 4

#define BASE_ADDR_LSB 0
#define BASE_ADDR_MSB BASE_ADDR_BITWIDTH - 1
#define IMG_SIZE_LSB BASE_ADDR_MSB + 1
#define IMG_SIZE_MSB BASE_ADDR_MSB + IMG_SIZE_BITWIDTH - 1
#define KRNL_SIZE_LSB IMG_SIZE_MSB + 1
#define KRNL_SIZE_MSB KRNL_SIZE_LSB + KRNL_SIZE_BITWIDTH
#define STRIDE_LSB KRNL_SIZE_MSB + 1
#define STRIDE_MSB STRIDE_LSB + STRIDE_BITWIDTH - 1

#define IN_STREAM_WIDTH BASE_ADDR_BITWIDTH + IMG_SIZE_BITWIDTH + KRNL_SIZE_BITWIDTH  + STRIDE_BITWIDTH

#define OUT_ADDR_BITWIDTH 8

#define ITER_NMBR_BITWIDTH 16
using namespace hls;
void SlidingWindowUnit(
		//32 bits: 16 for BASE Address, 8 for Image size, 4 Kernel size, 4 Stride
		stream<ap_uint<IN_STREAM_WIDTH>> &strm_in,
		stream<ap_uint<OUT_ADDR_BITWIDTH>> &strm_out,
		const ap_uint<ITER_NMBR_BITWIDTH> iter_number);

#endif
