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

entity add42_cc is
  port (
    a, b, c, gin, cin : in  std_logic;
    gout, cout, s     : out std_logic
  );
end add42_cc;


library UNISIM;
use UNISIM.vcomponents.all;

architecture xil of add42_cc is
  signal p : std_logic;
begin
  lut : LUT6_2
    generic map (
      INIT => x"6996_6996_E8E8_E8E8"
    )
    port map (
      O6 => p,
      O5 => gout,
      I5 => '1',
      I4 => '-',
      I3 => gin,
      I2 => c,
      I1 => b,
      I0 => a
    );
  mux : MUXCY
    port map (
      O  => cout,
      S  => p,
      CI => cin,
      DI => gin
    );
  sum : XORCY
    port map (
      O  => s,
      CI => cin,
      LI => p
    );
end xil;
