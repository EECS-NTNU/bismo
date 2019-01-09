#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>
#include <stdio.h>
#define IMG_SIZE 5
#define KRNL_SIZE 3
#define STRIDE 1
void SlidingWindowUnit(
		ap_uint<32> in_base_addres,
		ap_uint<32> in_img_size,
		ap_uint<32> in_krnl_size,
		ap_uint<32> in_stride,
		bool &out_err,
//		ap_uint<32> & out_mem_address
		hls::stream<ap_uint<32>> & out_mem_address
);
int main(int argc, char * argv[]){
	int test_mat [IMG_SIZE][IMG_SIZE];
	for(int i = 0; i < IMG_SIZE; i++)
		for(int j=0; j < IMG_SIZE; j++)
			test_mat[i][j] = i * IMG_SIZE + j;
	bool error = false;
	int mat_out_dim = ((IMG_SIZE - KRNL_SIZE)/ STRIDE + 1);
//	std::cout << mat_out_dim << std::endl;
	hls::stream<ap_uint<32>> out_mem_address;

	SlidingWindowUnit(0,IMG_SIZE,KRNL_SIZE,STRIDE,error,out_mem_address);

	for(int i=0; i < mat_out_dim * mat_out_dim * mat_out_dim * mat_out_dim; i++){
		std::cout << out_mem_address.read() << std::endl;
	}
}
