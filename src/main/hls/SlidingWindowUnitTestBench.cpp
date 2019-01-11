#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>
#include <stdio.h>
#define BASE_ADDRESS 0
#define IMG_SIZE 4
#define KRNL_SIZE 2
#define STRIDE 1
void SlidingWindowUnit(
		hls::stream<ap_uint<128>> &strm_in,
		hls::stream<ap_uint<32>> &strm_out,
		bool & out_err);
int main(int argc, char * argv[]){
	int test_mat [IMG_SIZE][IMG_SIZE];
	for(int i = 0; i < IMG_SIZE; i++)
		for(int j=0; j < IMG_SIZE; j++)
			test_mat[i][j] = i * IMG_SIZE + j;
	bool error = false;
	int mat_out_dim = ((IMG_SIZE - KRNL_SIZE)/ STRIDE + 1);
//	std::cout << mat_out_dim << std::endl;
	hls::stream<ap_uint<32>> out_mem_address;
	hls::stream<ap_uint<128>> in_strm;
	ap_uint<128> prova = 0;
	ap_uint<128> tmp = 0;
	prova = prova | BASE_ADDRESS;
//	std::cout << std::hex << prova;
//	std::cout << std::endl;
	prova =prova | ((tmp | IMG_SIZE) << 32);
	tmp = 0;
//	std::cout << prova;
//	std::cout << std::endl;
	prova = prova |( (tmp | KRNL_SIZE) << 64);
	tmp = 0;
//	std::cout << prova;
//	std::cout << std::endl;
	prova = prova | ((tmp | STRIDE) << 96);
	tmp = 0;
//	std::cout << std::endl;
//	std::cout << prova;
//	std::cout << std::dec << std::endl;
	in_strm.write(prova);
	SlidingWindowUnit(in_strm,out_mem_address,error);
	for(int i=0; i < mat_out_dim * mat_out_dim * mat_out_dim * mat_out_dim; i++){
		std::cout << out_mem_address.read() << std::endl;
	}
}
