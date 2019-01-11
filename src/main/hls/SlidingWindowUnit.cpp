#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>
//#define DEBUG 0

using namespace hls;
//Sliding window unit: Assuming to work with single channel
// Assuming Padding = 0
// Assuming Square dimensions for image, kernel and stride.
// Outputs the indices of the input matrix to consider for a matrix matrix multiplication
void SlidingWindowUnit(
		//128 bits: 32 for BASE Address, 32 for Image size, 32 Kernel size, 32 Stride
		hls::stream<ap_uint<128>> &strm_in,
		hls::stream<ap_uint<32>> &strm_out,
		bool &out_err)
{
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS interface axis port=strm_out
#pragma HLS interface axis port=strm_in

	ap_uint<128> in_tmp = strm_in.read();
	ap_uint<32> base_addr = in_tmp;
	ap_uint<32> img_size = in_tmp >> 32;
	ap_uint<32> krnl_size = in_tmp >> 64;
	ap_uint<32> stride = in_tmp >> 96;
//	if(DEBUG){
//		std::cout << in_tmp << std::endl;
//		printf("First 32 bits %x\n", static_cast<int>(base_addr) );
//		printf("Second 32 bits %x\n", static_cast<int>(img_size)  );
//		printf("Third 32 bits%x\n", static_cast<int>(krnl_size)  );
//		printf("Fourth 32 bits%x\n", static_cast<int>(stride)  );
//	}
	//Run time check for computability
	//TODO Needed? or assuming that this control is performed by a "compiler"
//	out_err = !( (in_img_size - in_krnl_size) %  in_stride == 0);

	//Formula from zero padding arbitrary stride convolution
	ap_uint<32> mat_out_dim = ((img_size - krnl_size)/ stride + 1);
	//Number of iterations of this convolution
	ap_uint<32> iter_number = mat_out_dim * mat_out_dim;
	//Input matrix iteration
	IMG_ROW_LOOP: for(int i=0; i < img_size; i= i + stride)

#pragma HLS PIPELINE II=1
//#pragma HLS LOOP_FLATTEN
		IMG_COL_LOOP: for(int j=0; j < img_size; j= j + stride){
//#pragma HLS PIPELINE II=1
			//Kernel iteration
//			std::cout << std::endl;
//			std::cout << i << ", " << j << std::endl;
			bool kernel_inbound = i-1 + krnl_size < img_size && j-1 + krnl_size < img_size;
			//Inbound check not needed in the inside loop
			KRNL_ROW_LOOP: for(int r = i; r < krnl_size + i && kernel_inbound; r++)
				KRNL_COL_LOOP: for(int c = j; c < krnl_size + j; c++){
//#pragma HLS PIPELINE II=1
					ap_uint<32> outElem = r * img_size + c;
//					std::cout << outElem << " ";// << std::endl;
					strm_out.write(outElem);
				}
			}


}
