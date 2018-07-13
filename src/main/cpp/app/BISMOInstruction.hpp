#include <stdint.h>
#include "bitfield.hpp"

union BISMOSyncInstruction {
  // storage for the instruction: 128 bits
  uint32_t raw[4] = {0, 0, 0, 0};
  // interpreted as bitfields
  BitField<0,1> isRunCfg;
  BitField<1,2> targetStage;
  BitField<3,1> isSendToken;
  BitField<4,2> chanID;
};
