#ifndef UTILS_H
#define UTILS_H
//helper funtction for golden results of dot product among matrices
void dotProduct(uint8_t * a, uint8_t * b, size_t arows, size_t acols, size_t brows, size_t bcols, int32_t * res){
	assert(acols == brows);
	for (int i = 0; i < arows; i++){
		for (int j = 0; j < bcols; j++){
			for (int k = 0; k < acols; k++)
			{
				res[i * bcols + j] += a[i* acols + k] * b[k * bcols + j];
			}
		}
	}
 }


//TODO: Not working now
void generateRandMatrixInt(size_t bits, size_t rows, size_t cols, int32_t** ret) {
  int32_t minVal = 0;
  int32_t maxVal = (1 << bits);
  for(size_t i = 0; i < rows; i++) {
  	for(size_t j = 0; j < cols; j++){
	    ret[i][j] = (int32_t) (rand() % maxVal);
  	}
  }
}


//helper funtction to quantize a given matrix with a given matrix of thresholds
void quantizeMatrix(int32_t * a, int32_t * b, size_t arows, size_t acols, size_t brows, size_t bcols, int32_t * r, size_t offset ){
    // std::cout << "Quantization :D" << endl;
    for (int i = 0; i < arows; i++)
      for (int j = 0; j < acols; j++){
        for (int k = 0; k < bcols; k++)
          //for (int l = 0; l < bcols; l++)
          {
              // std::cout << a[((i+offset) * acols) + j] << " vs " << b[i * bcols + k] << " ";
            if(a[((i+offset) * acols) + j] > b[(i) * bcols + k] ){
            
              r[((i+offset) * acols) + j]++;
            }
          }
           // cout << endl;
      }
  }

// Helper function for printing Integer matrix
void printmatrixInt(int32_t ** mat, int rows, int cols) {
  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < cols; j++) {
      std::cout << (int) mat[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

#endif