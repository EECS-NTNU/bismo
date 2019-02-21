// enable to print debug info
//#define DEBUG
// use instruction generators instead
#define BISMORT_USE_INSTRGEN
// enable to compare hw-produced results against sw-produced ones
//#define BISMORT_CONV_VERIFY_AGAINST_CPU
//#define BISMORT_MATMUL_VERIFY_AGAINST_CPU
// #define BISMORT_P2S_VERIFY_AGAINST_CPU
// clean p2s target buffer (set to zeros) before performing p2s
// #define P2S_CLEAR_IN_BUF
// force sw (instead of hw) p2s
#define FORCE_SW_P2S
