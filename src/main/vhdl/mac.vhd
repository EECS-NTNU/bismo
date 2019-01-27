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

entity mac is
  generic (
    WA              : positive;         -- Bit Width of Accumulator
    N               : positive;         -- Number of Products to Add
    WC              : positive;         -- Bit Width of Multipliers
    WD              : positive;         -- Bit Width of Multplicands
    HAVE_ACCU_INPUT : boolean := false;   -- Use Input a, ignore otherwise
    DEPTH           : natural         -- Number of register in the tree
  );
  port (
    clk : in std_logic;
    a : in  std_logic_vector(WA-1 downto 0);
    c : in  std_logic_vector(N*WC-1 downto 0);
    d : in  std_logic_vector(N*WD-1 downto 0);
    r : out std_logic_vector(WA-1 downto 0)
  );
end mac;


library UNISIM;
use UNISIM.vcomponents.all;

use std.textio.all;
use work.utils.all;

architecture rtl of mac is

  -----------------------------------------------------------------------------
  -- Bit Product Matrix

  -- Compute the Column Heights of the Product Matrices
  function PRODUCT_HEIGHTS_INIT return natural_vector is
    variable  res : natural_vector(WC+WD-2 downto 0);
  begin
    for i in res'range loop
      res(i) := 1 + i - MAXIMUM(0, 1+i-WC) - MAXIMUM(0, 1+i-WD);
    end loop;
    return  res;
  end;
  constant PRODUCT_HEIGHTS : natural_vector(WC+WD-2 downto 0) := PRODUCT_HEIGHTS_INIT;

  -----------------------------------------------------------------------------
  -- Pre-Compression Schedule
  constant TAG3     : integer := -1;
  constant TAG2P    : integer := -2;
  constant TAG_AND  : integer := -3;
  constant TAG_COPY : integer := -4;

  procedure print_precompression(sig : integer_vector) is
    variable l, ll   : line;
    variable tag : integer;
  begin
    write(l, string'("Shape:"));
    for i in sig(0) downto 1 loop
      write(l, ' ');
      write(l, sig(i));
    end loop;
    writeline(output, l);

    write(ll, string'("Trace: "));
    for i in sig(0)+1 to sig'high loop
      tag := sig(i);
      write(ll, ' '); write(ll, tag);
      if tag < 0 then
        case tag is
          when TAG3 =>
            write(l, sig(i+1));
            write(l, ',');
            write(l, sig(i+2));
            write(l, ',');
            write(l, sig(i+3));
            write(l, string'(" -> "));
            write(l, sig(i+4));
            write(l, ',');
            write(l, sig(i+5));

          when TAG2P =>
            write(l, 'A');
            write(l, sig(i+1));
            write(l, ',');
            write(l, sig(i+2));
            write(l, ',');
            write(l, sig(i+3));
            write(l, string'(" -> "));
            write(l, sig(i+4));
            write(l, ',');
            write(l, sig(i+5));

          when TAG_AND =>
            write(l, sig(i+1));
            write(l, string'(" -> "));
            write(l, sig(i+2));

          when TAG_COPY =>
            write(l, 'A');
            write(l, sig(i+1));
            write(l, string'(" -> "));
            write(l, sig(i+2));

          when others =>
            report "Unknown Tag." severity error;
        end case;
        writeline(output, l);
      end if;
    end loop;
    writeline(output, ll);
  end procedure print_precompression;

  function SCHEDULE_PRECOMPRESSION return integer_vector is

    constant INPUT_HEIGHTS : natural_vector(MAXIMUM(PRODUCT_HEIGHTS'length, WA)-1 downto 0)
      := N*PRODUCT_HEIGHTS + (1 to WA => 0);
    variable output_heights : natural_vector(INPUT_HEIGHTS'length downto 0);

    -- Result Construction
    variable res        : integer_vector(7*(output_heights'length+N*sum(PRODUCT_HEIGHTS)) downto 0);
    variable res_ptr    : natural := output_heights'length+1;
    procedure res_add(v : integer) is
    begin
      res(res_ptr) := v;
      res_ptr      := res_ptr + 1;
    end;
    variable input_ptr : natural := 0;
    procedure res_consume_input is
    begin
      res_add(input_ptr);
      input_ptr := input_ptr + 1;
    end procedure res_consume_input;

    variable column_base     : natural;  -- lowest output dot index for this column
    variable carried_dots    : natural;  -- dots already carried from previous column
    variable column_reducers : natural;  -- reducers in column
    variable column_copies   : natural range 0 to 2;  -- bits not reduced
    variable column_height   : natural;
    variable h               : natural;  -- height of current column

  begin

    -- For each iteration:
    --  - the heights of all columns to the right are known and fixed
    --  - the population of this column may have been started
    --  - this column will be finalized at the end of the iteration
    column_base  := 0;
    carried_dots := 0;
    for col in INPUT_HEIGHTS'reverse_range loop
      h := INPUT_HEIGHTS(col);

      -- finish populating this column with index column_base+carried_dots
      if HAVE_ACCU_INPUT then
        h := h + 1;
      end if;
      column_reducers := h/3;
      column_copies   := h - 3*column_reducers;
      column_height   := carried_dots + column_reducers + column_copies;
      -- start next column with index column_base+column_height

      -- schedule reducers
      for i in 0 to column_reducers-1 loop
        if i < column_reducers-1 or not HAVE_ACCU_INPUT then
          res_add(TAG3);
          res_consume_input;
        else
          res_add(TAG2P);
          res_add(col);
        end if;
        res_consume_input;
        res_consume_input;
        res_add(column_base + carried_dots  + i);  -- sum
        res_add(column_base + column_height + i);  -- carry
      end loop;

      for i in 0 to column_copies-1 loop
        if HAVE_ACCU_INPUT and i + column_reducers = 0 then
          res_add(TAG_COPY);
          res_add(col);
        else
          res_add(TAG_AND);
          res_consume_input;
        end if;
        res_add(column_base + carried_dots + column_reducers + i);
      end loop;

      -- Loop Variable Updates
      column_base         := column_base + column_height;
      carried_dots        := column_reducers;
      output_heights(col) := column_height;
    end loop;
    output_heights(output_heights'high) := carried_dots;

    assert input_ptr = N*WC*WD
      report "Pre-compression failed"
      severity failure;

    res(output_heights'length downto 0) := integer_vector(output_heights) & output_heights'length;
    return  res(res_ptr-1 downto 0);
  end SCHEDULE_PRECOMPRESSION;
  constant PRECOMPRESSION : integer_vector := SCHEDULE_PRECOMPRESSION;

  -- The Reduction Input
  signal ma, mb : std_logic_vector(N*WC*WD-1 downto 0);
  signal mr     : std_logic_vector(sum(PRECOMPRESSION(PRECOMPRESSION(0) downto 1))-1 downto 0);
  signal r_intermediate       : std_logic_vector(WA-1 downto 0);

begin

  -- Insert Bit Products into Reduction Matrix
  genProducts : for i in 0 to N-1 generate
    -- Extracted Factors
    signal cc : std_logic_vector(WC-1 downto 0);
    signal dd : std_logic_vector(WD-1 downto 0);
  begin
    cc <= c((i+1)*WC-1 downto i*WC);
    dd <= d((i+1)*WD-1 downto i*WD);
    genRows : for cx in cc'range generate
      genCols : for dx in dd'range generate
        constant COL  : natural := cx + dx;
      begin
        genAssign: if COL < WA generate
          constant TRGT : natural := N*sum(PRODUCT_HEIGHTS(COL-1 downto 0)) + i*PRODUCT_HEIGHTS(COL) + cx - MAXIMUM(0, 1+COL-WD);
        begin
          ma(TRGT) <= cc(cx) and dd(dx);
          mb(TRGT) <= cc(cx) and dd(dx);
        end generate genAssign;
      end generate genCols;
    end generate genRows;
  end generate genProducts;

  -- Generate pre-compressed Matrix
  genPre: for i in PRECOMPRESSION(0)+1 to PRECOMPRESSION'high generate
    constant TAG : integer := PRECOMPRESSION(i);
  begin
    genComp2P: if TAG = TAG2P generate
    --  signal x0, x1, x2 : std_logic;
    --begin
    --  -- TODO: ONE LUT6_2
    --  x0 <= A(PRECOMPRESSION(i+1));
    --  x1 <= ma(PRECOMPRESSION(i+2)) and mb(PRECOMPRESSION(i+2));
    --  x2 <= ma(PRECOMPRESSION(i+3)) and mb(PRECOMPRESSION(i+3));
    --  mr(PRECOMPRESSION(i+4)) <= x0 xor x1 xor x2;
    --  mr(PRECOMPRESSION(i+5)) <= (x0 and x1) or (x0 and x2) or (x1 and x2);

      lut: LUT6_2
        generic map (
          INIT => x"EA80_8080" & x"956A_6A6A"
        )
        port map (
          O6 => mr(PRECOMPRESSION(i+5)),
          O5 => mr(PRECOMPRESSION(i+4)),
          I5 => '1',
          I4 => mb(PRECOMPRESSION(i+3)),
          I3 => ma(PRECOMPRESSION(i+3)),
          I2 => mb(PRECOMPRESSION(i+2)),
          I1 => ma(PRECOMPRESSION(i+2)),
          I0 => A(PRECOMPRESSION(i+1))
        );
    end generate;
    genComp3: if TAG = TAG3 generate
      signal x0, x1, x2 : std_logic;
    begin
      -- TODO?: TWO LUT6?
      x0 <= ma(PRECOMPRESSION(i+1)) and mb(PRECOMPRESSION(i+1));
      x1 <= ma(PRECOMPRESSION(i+2)) and mb(PRECOMPRESSION(i+2));
      x2 <= ma(PRECOMPRESSION(i+3)) and mb(PRECOMPRESSION(i+3));
      mr(PRECOMPRESSION(i+4)) <= x0 xor x1 xor x2;
      mr(PRECOMPRESSION(i+5)) <= (x0 and x1) or (x0 and x2) or (x1 and x2);
    end generate;
    genAnd: if TAG = TAG_AND generate
      -- TODO?: MERGE TWO for LUT6_2?
      mr(PRECOMPRESSION(i+2)) <= ma(PRECOMPRESSION(i+1)) and mb(PRECOMPRESSION(i+1));
    end generate;
    genCopy: if TAG = TAG_COPY generate
      mr(PRECOMPRESSION(i+2)) <= A(PRECOMPRESSION(i+1));
    end generate;
  end generate genPre;

  blkCompress: block
    constant LAYOUT : natural_vector := natural_vector(PRECOMPRESSION(PRECOMPRESSION(0) downto 1));
    signal y : std_logic_vector(clog2_maxweight(LAYOUT)-1 downto 0);
  begin
    compressor: entity work.compress
      generic map (
        INPUT_LAYOUT => LAYOUT,
        DEPTH => DEPTH
      )
      port map (
        clk => clk,
        rst => '0',
        x => mr,
        y => y
      );
      process(y)
      begin
        r <= (others => '0');
        r(MINIMUM(y'left, r'left) downto 0) <= y(MINIMUM(y'left, r'left) downto 0);
      end process;
  end block blkCompress;

end rtl;
