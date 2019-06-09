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


CLK_FREQ=$1
CLK_NAME=fclk2_mhz

echo "Prev frequency was $PREV_FREQ"
PREV_FREQ=sudo python3 -c "from pynq.ps import Clocks; import sys; sys.stdout.write(str(Clocks.$CLK_NAME))"
echo $PREV_FREQ

echo "Setting frequency to $CLK_FREQ"
sudo python3 -c "from pynq.ps import Clocks; Clocks.$CLK_NAME = $CLK_FREQ; print(Clocks.$CLK_NAME)"
