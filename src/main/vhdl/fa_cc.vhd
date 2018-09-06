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

entity fa_cc is
  port (
    a, b, cin : in  std_logic;
    cout, s   : out std_logic
  );
end fa_cc;


library UNISIM;
use UNISIM.vcomponents.all;

architecture xil of fa_cc is
  signal p, d : std_logic;
begin
  p <= a xor b;
  d <= a;
  mux : MUXCY
    port map (
      O  => cout,
      S  => p,
      CI => cin,
      DI => d
    );
  sum : XORCY
    port map (
      O  => s,
      CI => cin,
      LI => p
    );
end xil;
