#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>
#include <stdio.h>
#include <SlidingWindowUnitHeader.hpp>
#define BASE_ADDRESS 0
#define IMG_SIZE 4
#define KRNL_SIZE 3
#define STRIDE 1
//void SlidingWindowUnit(
//		hls::stream<ap_uint<128>> &strm_in,
//		hls::stream<ap_uint<32>> &strm_out);

//Helper function for wrting "data" starting at bit position "bit_pos" into "elem"
ap_uint<IN_STREAM_WIDTH> write_bit_pos(ap_uint<IN_STREAM_WIDTH> &elem, int data, int bit_pos){
	ap_uint<IN_STREAM_WIDTH> tmp = 0;
	elem = elem |((tmp | data) <<  bit_pos);
	return elem;
}
int main(int argc, char * argv[]){
	int test_mat [IMG_SIZE][IMG_SIZE];
//	for(int i = 0; i < IMG_SIZE; i++)
//		for(int j=0; j < IMG_SIZE; j++)
//			test_mat[i][j] = i * IMG_SIZE + j;
	int mat_out_dim = ((IMG_SIZE - KRNL_SIZE)/ STRIDE + 1);
//	std::cout << mat_out_dim << std::endl;

	hls::stream<ap_uint<OUT_ADDR_BITWIDTH>> out_mem_address;
	hls::stream<ap_uint<IN_STREAM_WIDTH>> in_strm;

	ap_uint<IN_STREAM_WIDTH> inElem = 0;
	inElem(BASE_ADDR_MSB, BASE_ADDR_LSB) = BASE_ADDRESS;
	std::cout << std::hex << inElem << std::endl;
	inElem(IMG_SIZE_MSB, IMG_SIZE_LSB) = IMG_SIZE;
	std::cout << inElem << std::endl;
	inElem(KRNL_SIZE_MSB, KRNL_SIZE_LSB) = KRNL_SIZE;
	std::cout << inElem << std::endl;
	inElem(STRIDE_MSB, STRIDE_LSB) = STRIDE;
	std::cout << inElem;
	std::cout << std::dec << std::endl;

	in_strm.write(inElem);

	int iterations =  mat_out_dim * mat_out_dim * KRNL_SIZE * KRNL_SIZE;//IMG_SIZE * IMG_SIZE * KRNL_SIZE * KRNL_SIZE;
	int iter_number = IMG_SIZE * IMG_SIZE * KRNL_SIZE * KRNL_SIZE;
	SlidingWindowUnit(in_strm, out_mem_address /*, iter_number*/);
	for(int i=0; i < iterations ; i++){
		std::cout << out_mem_address.read() << std::endl;
	}
}
