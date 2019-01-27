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
void quantizeMatrix(int32_t * a, int32_t * ths, size_t arows, size_t acols, size_t trows, size_t tcols, uint8_t * r, size_t offset ){
    // std::cout << "Quantization :D" << endl;
    for (int i = 0; i < arows; i++)
      for (int j = 0; j < acols; j++){
        for (int k = 0; k < tcols; k++)
          {
              // std::cout << a[((i+offset) * acols) + j] << " vs " << b[i * bcols + k] << " ";
            if(a[((i+offset) * acols) + j] > ths[(i) * tcols + k] ){
            
              r[((i+offset) * acols) + j]++;
            }
          }
           // std::cout << endl;
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



  /* Imports a regular matrix after applying threshold quantization into this BitSerialMatrix.
  *  The threshold array is assumped to have the shape thresholds[nThres][nrows],
  *  and is assumed to be sorted s.t. the largest thresholds have the largest index.
  */
  template <typename T>
  void quantize2(T * matrix, T * thresholds, size_t nThres, size_t nrows, size_t ncols, uint8_t * res, bool readColMajor=false ) {
    // assert(!this->issigned); // threshold qnt. only makes sense for unsigned
    // this->clearAll();
    // std::cout << "Quantize 2 with thrs number " << nThres << endl;
    for(uint64_t r = 0; r < nrows; r++) {
      for(uint64_t c = 0; c < ncols; c++) {
        T currentElem = readColMajor ? matrix[c * nrows + r] : matrix[r * ncols + c];
        // std::cout << "Current Element " << currentElem << endl;
        int32_t idx = readColMajor ?(c * nrows + r) : (r * ncols + c);
        // quantize this element by finding the index of the largest crossed
        // threshold

        for(int t = 0; t < nThres; t++) {
          // std::cout << "Thr elem " << thresholds[t * nrows + r] << ", ";
          if(currentElem <= thresholds[t * nrows + r]) {
            currentElem = t;
            res[idx] = t;
            // std::cout << "Elem "  << t << endl;
            break;
          } else if(t == nThres - 1) {
            res[idx] = t + 1;
            // all thresholds crossed, set to largest quantization level
            currentElem = t + 1;
          }
        }
      }
    }
  }

  template <typename T>
  void transpose(T * matrix, size_t nrows, size_t ncols, T* res, size_t resrows, size_t rescols) {
    assert(resrows==ncols);
    assert(rescols==nrows);
    for(uint64_t r = 0; r < nrows; r++) {
      for(uint64_t c = 0; c < ncols; c++) {
        T currentElem = matrix[r * ncols + c];
        int32_t idx = (r * ncols + c);
        res[c * nrows + r] = currentElem;
      }
    }
  }

#endif