#include <ap_int.h>
#include <stdint.h>
#include <hls_stream.h>
#include <SlidingWindow.hpp>

//Sliding window unit: Assuming to work with single channel
// Assuming Padding = 0
// Assuming Square dimensions for image, kernel and stride.
// Outputs the indices of the input matrix to consider for a matrix matrix multiplication

void SlidingWindowUnit(
		//32 bits: 16 for BASE Address, 8 for Image size, 4 Kernel size, 4 Stride
		stream<ap_uint<IN_STREAM_WIDTH>> &strm_in,
		stream<ap_uint<OUT_ADDR_BITWIDTH>> &strm_out){

//#pragma HLS INTERFACE ap_ctrl_none port=return
//#pragma HLS interface axis port=strm_out
//#pragma HLS interface axis port=strm_in
//#pragma HLS dataflow
	ap_uint<IN_STREAM_WIDTH> in_tmp = strm_in.read();
	const ap_uint<BASE_ADDR_BITWIDTH> base_addr = in_tmp(BASE_ADDR_MSB, BASE_ADDR_LSB);
	const ap_uint<IMG_SIZE_BITWIDTH> img_size = in_tmp(IMG_SIZE_MSB, IMG_SIZE_LSB);
	const ap_uint<KRNL_SIZE_BITWIDTH> krnl_size = in_tmp(KRNL_SIZE_MSB, KRNL_SIZE_LSB);
	const ap_uint<STRIDE_BITWIDTH> stride = in_tmp(STRIDE_MSB, STRIDE_LSB);
/*	if(true){
		std::cout << in_tmp << std::endl;
		printf("First 32 bits %x\n", static_cast<int>(base_addr) );
		printf("Second 32 bits %x\n", static_cast<int>(img_size)  );
		printf("Third 32 bits%x\n", static_cast<int>(krnl_size)  );
		printf("Fourth 32 bits%x\n", static_cast<int>(stride)  );
	}*/
	//Run time check for computability
	//ASSUMPTION: this check is performed by the compiler!!!
//	out_err = !( (in_img_size - in_krnl_size) %  in_stride == 0);

	//Formula from zero padding arbitrary stride convolution
	const ap_uint<32> mat_out_dim = ((img_size - krnl_size)/ stride + 1);
	//Number of minimum iterations for this convolution
	const ap_uint<32> iter_number = mat_out_dim * mat_out_dim * krnl_size * krnl_size;
	//Input matrix iteration
	int i = 0, j = 0, kr = 0, kc = 0 ;
	MAIN_LOOP: for(int iter = 0; iter < iter_number; iter++){
#pragma HLS PIPELINE II=1
		std::cout << "Iter: " << iter << "Idx: " << i << ", " << j << ", " << kr << ", " << kc << std::endl;
		bool kernel_inbound = i-1 + krnl_size < img_size && j-1 + krnl_size < img_size;
		if(kernel_inbound){
//			std::cout << "Kernel inbound :D" << std::endl;
			ap_uint<OUT_ADDR_BITWIDTH> outElem = (kr+i) * img_size + (kc+j);
			strm_out.write(outElem);
			kc++;
			if(kc == krnl_size){
				kc = 0;
				kr++;
				if(kr == krnl_size){
					kr = 0;
					j = j + stride;
					if(j == img_size){
						j = 0;
						i = i + stride;
						if(i == img_size){
							i = 0;
						}
					}
				}
			}
		}else{
			iter--;
			kr = 0;
			kc = 0;
			j = j + stride;
			if(j == img_size){
				j = 0;
				i = i + stride;
				if(i == img_size){
					i = 0;
				}
			}
		}
	}
/*	IMG_ROW_LOOP: for(int i=0; i < iter_number; i= i + stride)
		IMG_COL_LOOP: for(int j=0; j < img_size; j= j + stride){
			//Kernel iteration
//			std::cout << std::endl;
//			std::cout << i << ", " << j << std::endl;
			bool kernel_inbound = i-1 + krnl_size < img_size && j-1 + krnl_size < img_size;
			//Inbound check not needed in the inside loop
			KRNL_ROW_LOOP: for(int r = i; r < krnl_size + i && kernel_inbound; r++)
				KRNL_COL_LOOP: for(int c = j; c < krnl_size + j; c++){
					ap_uint<32> outElem = r * img_size + c;
//					std::cout << outElem << " ";// << std::endl;
					strm_out.write(outElem);
				}
			}

*/
}
