# Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
# Copyright (c) 2019 Xilinx
#
# BSD v3 License
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of BISMO nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#!/bin/sh

# Usage: setclk.sh <freq-in-MHz>

# Set the clock frequency for fclk0
# Not all frequencies are supported due to how the PLLs work, the actual
# set frequency will be displayed.

CLK_BASE=/sys/devices/soc0/amba/f8007000.devcfg
#CLK_BASE=/sys/devices/amba.1/f8007000.devcfg
CLK_NAME="fclk0"
#CLK_NAME="FPGA0"
FCLK0_BASE=$CLK_BASE/fclk/$CLK_NAME


if [ ! -f $FCLK0_BASE ]; then
  echo $CLK_NAME > $CLK_BASE/fclk_export
fi

PREV_FREQ=$(cat $FCLK0_BASE/set_rate)
echo "Prev frequency was $PREV_FREQ"

echo "$1""000000" > $FCLK0_BASE/round_rate
ROUND_RES=$(cat $FCLK0_BASE/round_rate)
set $ROUND_RES

echo "Setting frequency to $3"

echo $3 > $FCLK0_BASE/set_rate
