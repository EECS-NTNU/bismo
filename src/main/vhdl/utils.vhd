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

package utils is

  function clog2(arg : positive) return natural;

  type integer_vector is array(natural range<>) of integer;
  function MINIMUM(a, b : integer) return integer;
  function MAXIMUM(a, b : integer) return integer;
  function MAXIMUM(nv : integer_vector) return integer;
  function sum      (vec : integer_vector) return integer;
  function count    (vec : integer_vector; key : integer) return natural;
  impure function to_string(vec : integer_vector) return string;

  type natural_vector is array(natural range<>) of natural;
  function MAXIMUM(nv     : natural_vector) return natural;
  function "+" (av, bv    : natural_vector) return natural_vector;
  function "*" (c : natural; v : natural_vector) return natural_vector;
  function sum (vec       : natural_vector) return natural;
  function max_weight(vec : natural_vector) return natural;
  function clog2_maxweight(vec : natural_vector) return natural;
  function ltrim (vec     : natural_vector) return natural_vector;
  impure function to_string(vec  : natural_vector) return string;

  type positive_vector is array(natural range<>) of positive;
  function sum(vec : positive_vector) return natural;
  
  function weight(v : std_logic_vector) return natural;

end package utils;


package body utils is

  function MINIMUM(a, b : integer) return integer is
  begin
    if a < b then
      return  a;
    else
      return  b;
    end if;
  end function MINIMUM;
  function MAXIMUM(a, b : integer) return integer is
  begin
    if a > b then
      return  a;
    else
      return  b;
    end if;
  end function MAXIMUM;
  function MAXIMUM(nv : integer_vector) return integer is
    variable  res : integer := integer'low;
  begin
    for i in nv'range loop
      if nv(i) > res then
        res := nv(i);
      end if;
    end loop;  -- i
    return  res;
  end function MAXIMUM;
  function MAXIMUM(nv : natural_vector) return natural is
    variable  res : natural := natural'low;
  begin
    for i in nv'range loop
      if nv(i) > res then
        res := nv(i);
      end if;
    end loop;  -- i
    return  res;
  end function MAXIMUM;

  function clog2(arg : positive) return natural is
    variable tmp : positive;
    variable log : natural;
  begin
    if arg = 1 then return 0; end if;
    tmp := 1;
    log := 0;
    while arg > tmp loop
      tmp := tmp * 2;
      log := log + 1;
    end loop;
    return  log;
  end function;

  function sum(vec : integer_vector) return integer is
    variable  res : integer;
  begin
    res := 0;
    for i in vec'range loop
      res := res + vec(i);
    end loop;
    return  res;
  end function;

  function count(vec : integer_vector; key : integer) return natural is
    variable res : natural;
  begin
    res := 0;
    for i in vec'range loop
      if vec(i) = key then
        res := res + 1;
      end if;
    end loop;
    return  res;
  end function;

  impure function to_string(vec : integer_vector) return string is
    impure function slen return natural_vector is
      variable  res : natural_vector(vec'range);
      variable  v   : integer;
      variable  n   : natural;
    begin
      for i in vec'range loop
        v := vec(i);
        if v >= 0 then
          n := 1;
        else
          v := -v;
          n := 2;
        end if;
        while v > 9 loop
          v := v/10;
          n := n + 1;
        end loop;
        res(i) := n+1;
      end loop;
      return  res;
    end;
    constant LENGTHS : natural_vector := slen;

    variable  s : string(1 to sum(LENGTHS));
    variable  p : natural;
  begin
    p := 1;
    for i in vec'range loop
      s(p to p+LENGTHS(i)-1) := integer'image(vec(i))&',';
      p := p + LENGTHS(i);
    end loop;
    return  s(1 to p-2);
  end function;

  function "+"(av, bv : natural_vector) return natural_vector is
    constant N   : natural                      := MAXIMUM(av'length, bv'length);
    constant aa  : natural_vector(N   downto 0) := (N downto av'length => 0) & av;
    constant bb  : natural_vector(N   downto 0) := (N downto bv'length => 0) & bv;
    variable res : natural_vector(N-1 downto 0);
  begin
    for i in res'range loop
      res(i) := aa(i) + bb(i);
    end loop;
    return  res;
  end function;

  function "*" (c : natural; v : natural_vector) return natural_vector is
    variable  res : natural_vector(v'range);
  begin
    for i in v'range loop
      res(i) := c*v(i);
    end loop;
    return  res;
  end function;

  function sum(vec : natural_vector) return natural is
    variable  res : natural;
  begin
    res := 0;
    for i in vec'range loop
      res := res + vec(i);
    end loop;  -- i
    return  res;
  end function;

  function max_weight(vec : natural_vector) return natural is
    constant nv : natural_vector(vec'length-1 downto 0) := vec;
    variable res : natural;
  begin
    res := 0;
    for i in nv'range loop
      res := res + 2**i * nv(i);
    end loop;
    return  res;
  end function;

  function clog2_maxweight(vec : natural_vector) return natural is
    constant  nv : natural_vector := ltrim(vec);
    variable  cy : natural;
  begin
    cy := 0;
    for i in 0 to nv'left loop
      cy := (cy + nv(i)) / 2;
    end loop;
    return  nv'length + clog2(1+cy);
  end function;

  function ltrim(vec : natural_vector) return natural_vector is
    variable  res : natural_vector(vec'length-1 downto 0);
  begin
    res := vec;
    for i in res'range loop
      if res(i) /= 0 then
        return  res(i downto 0);
      end if;
    end loop;
    return  res(1 downto 0);
  end function;

  impure function to_string(vec : natural_vector) return string is
  begin
    return  to_string(integer_vector(vec));
  end function;

  function sum(vec : positive_vector) return natural is
    variable  res : natural;
  begin
    res := 0;
    for i in vec'range loop
      res := res + vec(i);
    end loop;  -- i
    return  res;
  end function;

  function weight(v : std_logic_vector) return natural is
    variable res : natural;
  begin
    res := 0;
    for i in v'range loop
      case to_X01(v(i)) is
        when '0' =>
          null;
        when '1' =>
          res := res + 1;
        when 'X' =>
          report "Meta-value detected, interpreting it as zero."
            severity error;
      end case;
    end loop;
    return  res;
  end function weight;

end package body utils;
