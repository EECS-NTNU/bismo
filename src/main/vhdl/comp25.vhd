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

entity comp25 is
  port (
    x0 : in  std_logic_vector(4 downto 0);
    x1 : in  std_logic_vector(1 downto 0);
    o0 : out std_logic_vector(1 downto 0);
    o1 : out std_logic_vector(1 downto 0)
  );
end entity comp25;


library UNISIM;
use UNISIM.vcomponents.all;

architecture xil of comp25 is
begin

  lut0: LUT6_2
    generic map (
      INIT => x"9669_6996" & x"E88E_8EE8"
    )
    port map (
      O6 => o0(0),
      O5 => o0(1),
      I5 => '1',
      I4 => x0(4),
      I3 => x0(3),
      I2 => x0(2),
      I1 => x0(1),
      I0 => x0(0)
    );

  lut1: LUT6_2
    generic map (
      INIT => x"E817_17E8" & x"FFE8_E800"
    )
    port map (
      O6 => o1(0),
      O5 => o1(1),
      I5 => '1',
      I4 => x1(1),
      I3 => x1(0),
      I2 => x0(4),
      I1 => x0(3),
      I0 => x0(2)
    );

end xil;
