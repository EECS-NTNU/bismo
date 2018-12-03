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

use std.textio.all;
use work.utils.all;

package counters is

  constant DEBUG : boolean := false;

  -----------------------------------------------------------------------------
  -- Counter and Accessors
  type counter is record
                    tag     : natural;
                    inputs  : natural_vector(3 downto 0);
                    outputs : natural_vector(4 downto 0);
                    luts    : positive;
                  end record counter;

  impure function to_string(c : counter) return string;

  -- Asymptotic Height Reduction Ratio per Step
  function strength(c : counter) return real;

  -- Dot Reduction per LUT
  function efficiency(c : counter) return real;

  -- Numerical Slack within Maximum Output Range
  function slack(c : counter) return real;

  -- Some Preference Order based on the above: smaller is better
  function "<"(l, r : counter) return boolean;

  type order_type is (EFFICIENCY_STRENGTH, STRENGTH_EFFICIENCY, PRODUCT);
  constant ORDER : order_type := EFFICIENCY_STRENGTH;

  -----------------------------------------------------------------------------
  -- Counter Arrays
  type counter_vector is array(natural range<>) of counter;

  impure function to_string(cv : counter_vector) return string;

  function sort(cv : counter_vector) return counter_vector;

  constant COUNTERS : counter_vector := (
    ( 0, (0,6,0,6), (1,1,1,1,1), 4),
    ( 1, (0,6,1,5), (1,1,1,1,1), 4),
    ( 2, (0,6,2,3), (1,1,1,1,1), 4),

    ( 3, (1,4,0,6), (1,1,1,1,1), 4),
    ( 4, (1,4,1,5), (1,1,1,1,1), 4),
    ( 5, (1,4,2,3), (1,1,1,1,1), 4),

    ( 6, (2,2,0,6), (1,1,1,1,1), 4),
    ( 7, (2,2,1,5), (1,1,1,1,1), 4),
    ( 8, (2,2,2,3), (1,1,1,1,1), 4),

    ( 9, (1,3,2,5), (1,1,1,1,1), 4),

    (10, (0,0,2,5), (0,0,1,2,1), 2),
    (11, (0,0,0,6), (0,0,1,1,1), 3),
    (12, (0,0,0,3), (0,0,0,1,1), 1)
  );

  constant RESULT_CP : positive := COUNTERS'length + 0;
  constant RESULT_FA : positive := COUNTERS'length + 1;
  constant RESULT_42 : positive := COUNTERS'length + 2;
  constant STAGE_BUF : positive := COUNTERS'length + 3;

  -----------------------------------------------------------------------------
  -- Schedule Computation
  impure function schedule(primaries : natural_vector) return integer_vector;

end package counters;


use std.textio.all;

library IEEE;
use IEEE.numeric_bit.all;

package body counters is

  -----------------------------------------------------------------------------
  -- Counter and associated Metrics

  impure function to_string(c : counter) return string is
  begin
    return
      '('&to_string(ltrim(c.inputs))&':'&to_string(ltrim(c.outputs))&
      "] /"&integer'image(c.luts);
  end function;

  function strength(c : counter) return real is
  begin
    return  real(sum(c.inputs)) / real(sum(c.outputs));
  end function;

  function efficiency(c : counter) return real is
  begin
    return  real(sum(c.inputs)-sum(c.outputs)) / real(c.luts);
  end function;

  function slack(c : counter) return real is
  begin
    return  1.0 - real(1+max_weight(c.inputs))/real(1+max_weight(c.outputs));
  end function;

  function "<"(l, r : counter) return boolean is
    constant  le : real := efficiency(l);
    constant  re : real := efficiency(r);
    constant  ls : real := strength(l);
    constant  rs : real := strength(r);
  begin
    case ORDER is
    when EFFICIENCY_STRENGTH =>
      if le /= re then
  return  le > re;
      end if;
      if ls /= rs then
  return  ls > rs;
      end if;

    when STRENGTH_EFFICIENCY =>
      if ls /= rs then
  return  ls > rs;
      end if;
      if le /= re then
  return  le > re;
      end if;

    when others =>
      if le*ls /= re*rs then
  return  le*ls > re*rs;
      end if;

    end case;
    return  slack(l) < slack(r);
  end function;

  -----------------------------------------------------------------------------
  -- Counter Arrays
  function sort(cv : counter_vector) return counter_vector is
    variable res     : counter_vector(0 to cv'length-1);
    variable tmp     : counter;
    variable changed : boolean;
  begin
    -- Do a simple stable Bubble sort
    res := cv;
    for i in res'high-1 downto 0 loop
      -- Let bigger counters bubble up to the end
      changed := false;
      for j in 0 to i loop
        if res(j+1) < res(j) then
          tmp      := res(j);
          res(j)   := res(j+1);
          res(j+1) := tmp;
          changed  := true;
        end if;
      end loop;
      exit when not changed;
    end loop;
    return res;
  end function;

  impure function to_string(cv : counter_vector) return string is
    variable l : line;
  begin
    for i in cv'range loop
      write(l, to_string(cv(i)));
      write(l, LF);
    end loop;
    return  l.all;
  end function;

  -----------------------------------------------------------------------------
  -- Schedule Computation

  impure function schedule(primaries : natural_vector) return integer_vector is

    -- Reduction Dimensions
    constant M : natural := 2*MAXIMUM(primaries);
    constant W : natural := clog2_maxweight(primaries);

    --------------------
    -- Signal queue types and manipulation functions
    variable  sig_count : positive := 1;  -- Next ID to assign to a signal

    type natural_queue is record
      wp, rp : natural;                 -- Write and Read Pointers
      buf    : natural_vector(0 to M);  -- Data Buffer
    end record;
    type bit_queues is array(natural range<>) of natural_queue;
    variable cols : bit_queues(W-1 downto 0);

    impure function height(idx : natural) return natural is
    begin
      return  cols(idx).wp - cols(idx).rp;
    end function height;

    impure function push(idx : natural) return natural is
      variable  wp  : natural;
      variable  res : natural;
    begin
      wp  := cols(idx).wp;
      res := sig_count;
      sig_count := res + 1;
      cols(idx).buf(wp mod(M+1)) := res;
      cols(idx).wp := wp + 1;
      return  res;
    end function push;

    impure function pop(idx : natural) return natural is
      variable  rp  : natural;
      variable  res : natural;
    begin
      rp  := cols(idx).rp;
      res := cols(idx).buf(rp mod(M+1));
      cols(idx).rp := rp + 1;
      return  res;
    end function pop;

    function "<="(av, bv : natural_vector) return boolean is
      constant N   : natural                    := MAXIMUM(av'length, bv'length);
      constant aa  : natural_vector(N downto 0) := (N downto av'length => 0) & av;
      constant bb  : natural_vector(N downto 0) := (N downto bv'length => 0) & bv;
    begin
      for i in N-1 downto 0 loop
        if aa(i) > bb(i) then
          return  false;
        end if;
      end loop;
      return  true;
    end function;

    --------------------
    -- Result Construction
    variable res_buf : integer_vector(0 to 6*(sum(primaries)+W));
    variable res_ptr : natural := 0;

    procedure res_add(v : integer) is
    begin
      res_buf(res_ptr) := v;
      res_ptr := res_ptr + 1;
    end procedure res_add;

    --------------------
    -- Compression

    -- Ordered list of counters
    constant COUNTS : counter_vector := sort(COUNTERS);

    -- Height Status
    variable levels     : natural := 0;
    variable height_cur : natural_vector(W-1 downto 0);
    variable height_nxt : natural_vector(W-1 downto 0);

    -- Current Compression Anchor
    function anchor(height_eff : natural_vector(W-1 downto 0)) return natural is
      variable cry : natural;
    begin
      cry := 0;
      for i in 0 to W-1 loop
        if height_eff(i) > 4 or height_eff(i)+cry > 5 then
          return  i;
        end if;
        cry := (cry + height_eff(i))/2;
      end loop;
      return  W;
    end function anchor;

    --------------------
    -- Temporaries
    variable cnt    : counter;
    variable inlen   : natural;
    variable tn, cry : natural;

    variable l : line;

  begin

    --------------------
    -- Initialize primary inputs and initial heights
    for i in primaries'reverse_range loop
      for j in 1 to primaries(i) loop
        tn := push(abs(i-primaries'right));
      end loop;
    end loop;
    for i in height_cur'range loop
      height_cur(i) := height(i);
    end loop;
    height_nxt := (others => 0);

    if DEBUG then
      write(l, string'("Counter Preference:"&HT&HT&"[  Eff / Strng/ Slack]"));
      writeline(output, l);
      write(l, string'("---------"));
      writeline(output, l);
      for i in COUNTS'range loop
        cnt := COUNTS(i);
        write(l, i, RIGHT, 2);
        write(l, ':');
        write(l, to_string(cnt), RIGHT, 23);
        write(l, string'(HT & "[ "));
        write(l, efficiency(cnt), RIGHT, 4, 2);
        write(l, string'(" / "));
        write(l, strength(cnt), RIGHT, 4, 2);
        write(l, string'(" / "));
        write(l, slack(cnt), RIGHT, 4, 2);
        write(l, string'(" ]"));
        writeline(output, l);
      end loop;
      write(l, string'("---------"));
      writeline(output, l);
    end if;
    
    while anchor(height_cur) < W loop
      assert not DEBUG report "Heights: "&to_string(height_cur)
        severity note;

      -- for each counter starting from the best
      l0: for i in COUNTS'range loop
        cnt  := COUNTS(i);
        inlen := ltrim(cnt.inputs)'length;

        -- try full placements starting from the right
        for j in anchor(height_cur+height_nxt) to height_cur'length-inlen loop
          while anchor(height_cur+height_nxt) <= j and cnt.inputs(inlen-1 downto 0) <= height_cur(j + inlen-1 downto j) loop
            
            -- Schedule a Counter
            assert not DEBUG report to_string(cnt)&" @"&integer'image(j)
              severity note;

            -- Tag
            res_add(-cnt.tag-1);

            -- Inputs
            for k in inlen-1 downto 0 loop
              tn := cnt.inputs(k);
              for l in 1 to tn loop
                res_add(pop(j+k));
              end loop;
              height_cur(j+k) := height_cur(j+k) - tn;
            end loop;

            -- Outputs
            for k in integer range ltrim(cnt.outputs)'length-1 downto 0 loop
              tn := cnt.outputs(k);
              for l in 1 to tn loop
                res_add(push(j+k));
              end loop;
              height_nxt(j+k) := height_nxt(j+k) + tn;
            end loop;

          end loop;
        end loop;
     end loop l0; -- over counters

      height_cur := height_cur + height_nxt;
      height_nxt := (others => 0);
      levels     := levels + 1;
    
      res_add(-STAGE_BUF- 1);
      tn := sum(height_cur);
      res_add(tn);
      for i in height_cur'range loop
        for j in 1 to height_cur(i) loop
          res_add(pop(i));
          res_add(push(i));
        end loop;
      end loop;

     end loop;

    write(l, string'("Schedule:"));
    writeline(output, l);
    write(l, string'("---------"));
    writeline(output, l);
    inlen := 0;
    for i in COUNTS'range loop
      cnt := COUNTS(i);
      tn := count(res_buf(0 to res_ptr-1), -cnt.tag-1);
      if tn /= 0 then
        inlen := inlen + tn * cnt.luts;
        write(l, integer'image(tn), RIGHT, 3);
        write(l, string'("x "));
        write(l, to_string(cnt), RIGHT, 22);
        writeline(output, l);
      end if;
    end loop;
    write(l, string'("---------"));
    writeline(output, l);
    write(l, string'("Primaries: "));
    write(l, to_string(primaries)&HT&" -> "&integer'image(sum(primaries)));
    writeline(output, l);
    write(l, string'("Total Levels:"));
    write(l, integer'image(levels), RIGHT, 3);
    writeline(output, l);
    write(l, string'("Result: "));
    write(l, to_string(height_cur)&HT&" -> "&integer'image(sum(height_cur)));
    writeline(output, l);
    write(l, string'("---------"));
    writeline(output, l);

    -- Build Result Chain
    cry := 0;
    for i in height_cur'reverse_range loop
      res_add(i);

      tn := cry + height_cur(i);
      if tn <= 1 then
        res_add(-RESULT_CP-1);
        cry := 0;
      else
        if tn <= 3 then
          res_add(-RESULT_FA-1);
          cry := 1;
        else
          res_add(-RESULT_42-1);
          cry := 2;
          res_add(pop(i));
          res_add(pop(i));
        end if;
        res_add(pop(i));
      end if;

      if tn mod 2 = 0 then
        res_add(0);
      else
        res_add(pop(i));
      end if;

      if tn > 1 then
        res_add(pop(i));
        for j in integer range 1 to tn/2 loop
          if i + 1 = W then
            res_add(push(0));
          else
            res_add(push(i+1));
          end if;
        end loop;
      end if;
    end loop;

    -- Print Schedule
    assert not DEBUG
      report to_string(res_buf(0 to res_ptr-1))
      severity note;

    return  res_buf(0 to res_ptr-1);

  end function schedule;

end package body counters;
