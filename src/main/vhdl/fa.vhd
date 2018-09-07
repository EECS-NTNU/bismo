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

entity fa is
  port (
    x : in  std_logic_vector(2 downto 0);
    o : out std_logic_vector(1 downto 0)
  );
end entity fa;


library UNISIM;
use UNISIM.vcomponents.all;

architecture xil of fa is
begin
  lut : LUT6_2
    generic map (
      INIT => x"E8E8_E8E8_9696_9696"
    )
    port map (
      O6 => o(1),
      O5 => o(0),
      I5 => '1',
      I4 => '0',
      I3 => '0',
      I2 => x(2),
      I1 => x(1),
      I0 => x(0)
    );
end xil;
