// enable to print debug info
//#define DEBUG
// use instruction generators instead
#define BISMORT_USE_INSTRGEN
// enable instrumentation for detailed measurements
#define BISMORT_INSTRUMENTATION
//#define BISMORT_INSTRUMENTATION_VERBOSE
// enable to compare hw-produced results against sw-produced ones
//#define BISMORT_CONV_VERIFY_AGAINST_CPU
//#define BISMORT_MATMUL_VERIFY_AGAINST_CPU
// #define BISMORT_P2S_VERIFY_AGAINST_CPU
// clean p2s target buffer (set to zeros) before performing p2s
// #define P2S_CLEAR_IN_BUF
// number of bytes for the p2s bit-parallel buffer on the accelerator side
#define BISMORT_P2S_BITPAR_BYTES  (1024*1024)
