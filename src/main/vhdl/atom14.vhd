-------------------------------------------------------------------------------
--  Copyright (C) 2017, Xilinx, Inc.
--  All rights reserved.
--
--  Author: Thomas B. Preusser <thomas.preusser@utexas.edu>
--          Marie-Curie Fellow, Xilinx Ireland, Grant Agreement No. 751339
--
--  This project has received funding from the European Union's Framework
--  Programme for Research and Innovation Horizon 2020 (2014-2020) under
--  the Marie Sklodowska-Curie Grant Agreement No. 751339.
-------------------------------------------------------------------------------
library IEEE;
use IEEE.std_logic_1164.all;

entity atom14 is
  port (
    -- Bit Inputs
    x1 : in  std_logic;
    x0 : in  std_logic_vector(3 downto 0);

    -- Carry-Chain MUX Control Outputs
    d  : out std_logic_vector(1 downto 0);
    s  : out std_logic_vector(1 downto 0)
  );
end entity atom14;


library UNISIM;
use UNISIM.vcomponents.all;

architecture xil of atom14 is
begin

  lo : LUT6_2
    generic map (
      INIT => x"6996_6996" & x"FF00_FF00"
    )
    port map (
      O6 => s(0),
      O5 => d(0),
      I5 => '1',
      I4 => '0',
      I3 => x0(3),
      I2 => x0(2),
      I1 => x0(1),
      I0 => x0(0)
    );

  hi : LUT6_2
    generic map (
      INIT => x"17E8_17E8" & x"FF00_FF00"
    )
    port map (
      O6 => s(1),
      O5 => d(1),
      I5 => '1',
      I4 => '0',
      I3 => x1,
      I2 => x0(2),
      I1 => x0(1),
      I0 => x0(0)
    );

end xil;
