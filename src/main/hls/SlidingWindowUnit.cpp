#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>

using namespace hls;
//Sliding window unit: Assuming to work with single channel
// Assuming Padding = 0
// Assuming Square dimensions for image, kernel and stride.
// Outputs the indices of the input matrix to consider for a matrix matrix multiplication
void SlidingWindowUnit(
		//This should a single data structure or a stream combined
//		hls::stream<ap_uint<128>> &strm_in,
//		hls::stream<ap_uint<32>> &strm_out,
		ap_uint<32> * out_mem_address,
		bool & out_err,
		ap_uint<32> in_base_address,
		ap_uint<32> in_img_size,
		ap_uint<32> in_krnl_size,
		ap_uint<32> in_stride)
{
//#pragma HLS INTERFACE ap_ctrl_none port=return
//#pragma HLS interface axis port=strm_out
//#pragma HLS interface axis port=strm_in
	//Run time check for computability
	//TODO Needed? or assuming that this control is performed by a "compiler"
//	out_err = !( (in_img_size - in_krnl_size) %  in_stride == 0);
	ap_uint<32> img_size = in_img_size;
	ap_uint<32> base_addr = in_base_address;
	ap_uint<32> krnl_size = in_krnl_size;
	ap_uint<32> stride = in_stride;
	//Formula from zero padding arbitrary stride convolution
	ap_uint<32> mat_out_dim = ((in_img_size - in_krnl_size)/ in_stride + 1);
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
			bool kernel_inbound = i-1 + in_krnl_size < in_img_size && j-1 + in_krnl_size < in_img_size;
			//Inbound check not needed in the inside loop
			KRNL_ROW_LOOP: for(int r = i; r < in_krnl_size + i && kernel_inbound; r++)
				KRNL_COL_LOOP: for(int c = j; c < in_krnl_size + j; c++){
//#pragma HLS PIPELINE II=1
					ap_uint<32> outElem = r * in_img_size + c;
//					std::cout << outElem << " ";// << std::endl;
//					strm_out.write(outElem);//.write(outElem);
					*out_mem_address = outElem;
				}
			}


}
