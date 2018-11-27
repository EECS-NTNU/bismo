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

use work.utils.all;
use std.textio.all;

entity compress is
  generic (
    INPUT_LAYOUT : natural_vector;
    DEPTH : natural := 0
  );
  port (
    clk : in std_logic;
    rst : in std_logic;
    x : in  std_logic_vector(sum(INPUT_LAYOUT)-1 downto 0);
    y : out std_logic_vector(clog2_maxweight(INPUT_LAYOUT)-1 downto 0)
  );
end entity compress;


library UNISIM;
use UNISIM.vcomponents.all;

use work.counters.all;

architecture xil of compress is

  -- Compression Schedule
  constant S : integer_vector := schedule(INPUT_LAYOUT);
  constant D : positive := count(S, -STAGE_BUF-1);
  signal nn  : std_logic_vector(MAXIMUM(S) downto 0);  -- used bit signals

begin

  assert DEPTH <= D
    report "DEPTH larger than number of compression levels not yet supported."
    severity failure;

  -- Feed Inputs
  nn(x'length downto 0) <= x & '0';
  
  -- Implement Schedule
  genSchedule : for i in S'range generate
    constant TAG : integer := -(S(i)+1);
  begin
    genTag : if TAG >= 0 generate
      genCounter : if TAG < COUNTERS'length generate
        constant comp        : counter := COUNTERS(tag);
        constant INPUT_BITS  : positive   := sum(comp.inputs);
        constant OUTPUT_BITS : positive   := sum(comp.outputs);

        signal bits_in  : std_logic_vector(INPUT_BITS-1 downto 0);
        signal bits_out : std_logic_vector(OUTPUT_BITS-1 downto 0);
      begin

        -- Extract Inputs
        genExtrIn: for j in bits_in'range generate
          bits_in(j) <= nn(S(i+INPUT_BITS-j));
        end generate genExtrIn;

        -- Map Outputs
        genMapOut: for j in bits_out'range generate
          nn(S(i+INPUT_BITS+OUTPUT_BITS-j)) <= bits_out(j);
        end generate genMapOut;

        -- Generate Counters that utilize a CARRY4 primitive
        genChain : if TAG < 10 generate

          -- CARRY4 Connectivity
          signal cin  : std_logic;
          signal d, p : std_logic_vector(3 downto 0);

        begin
          genAtoms: if TAG < 9 generate
            constant KIND_HI : natural range 0 to 2 := TAG/3;
            constant KIND_LO : natural range 0 to 2 := TAG mod 3;

            constant HI_BITS : natural := 6 - KIND_HI;
            constant LO_BITS : natural := 6 - KIND_LO/2;

            -- Extracted Signals
            signal bits_hi : std_logic_vector(HI_BITS-1 downto 0);
            signal bits_lo : std_logic_vector(LO_BITS-1 downto 0);

          begin

            -- Extract Inputs
            bits_hi <= bits_in(bits_in'left downto LO_BITS);
            bits_lo <= bits_in(LO_BITS-1 downto 0);

            -- Instantiate HI LUTs
            genHi06: if KIND_HI = 0 generate
              atom: entity work.atom06
                port map (
                  x0 => bits_hi,
                  d  => d(3 downto 2),
                  s  => p(3 downto 2)
                );
            end generate genHi06;
            genHi14: if KIND_HI = 1 generate
              atom: entity work.atom14
                port map (
                  x1 => bits_hi(bits_hi'left),
                  x0 => bits_hi(bits_hi'left-1 downto 0),
                  d  => d(3 downto 2),
                  s  => p(3 downto 2)
                );
            end generate genHi14;
            genHi22: if KIND_HI = 2 generate
              atom: entity work.atom22
                port map (
                  x1 => bits_hi(3 downto 2),
                  x0 => bits_hi(1 downto 0),
                  d  => d(3 downto 2),
                  s  => p(3 downto 2)
                );
            end generate genHi22;

            -- Instantiate LO LUTs
            genLo06: if KIND_LO = 0 generate
              atom: entity work.atom06
                port map (
                  x0 => bits_lo,
                  d  => d(1 downto 0),
                  s  => p(1 downto 0)
                );
              cin <= '0';
            end generate genLo06;
            genLo14: if KIND_LO = 1 generate
              atom: entity work.atom14
                port map (
                  x1 => bits_lo(bits_lo'left),
                  x0 => bits_lo(bits_lo'left-1 downto 1),
                  d  => d(1 downto 0),
                  s  => p(1 downto 0)
                );
              cin <= bits_lo(0);
            end generate genLo14;
            genLo22: if KIND_LO = 2 generate
              atom: entity work.atom22
                port map (
                  x1 => bits_lo(4 downto 3),
                  x0 => bits_lo(2 downto 1),
                  d  => d(1 downto 0),
                  s  => p(1 downto 0)
                );
              cin <= bits_lo(0);
            end generate genLo22;

          end generate genAtoms;

          -- Instantiate full LUT block
          genBlock: if TAG = 9 generate
            block1324_1: entity work.block1324
              port map (
                x0 => bits_in(4 downto 1),
                x1 => bits_in(6 downto 5),
                x2 => bits_in(bits_in'left-1 downto 7),
                x3 => bits_in(bits_in'left),
                d  => d,
                s  => p
              );
            cin <= bits_in(0);
          end generate genBlock;

          -- Instantiate Chain
          genCC: block
            signal co : std_logic_vector(4 downto 1);
          begin
            cc : CARRY4
              port map (
                CO     => co,
                O      => bits_out(3 downto 0),
                CI     => '0',
                CYINIT => cin,
                DI     => d,
                S      => p
              );
            bits_out(4) <= co(4);
          end block genCC;

        end generate genChain;

        -- Generate floating instances
        gen25: if TAG = 10 generate
          comp25_1: entity work.comp25
            port map (
              x0 => bits_in(4 downto 0),
              x1 => bits_in(bits_in'left downto 5),
              o0 => bits_out(1 downto 0),
              o1 => bits_out(bits_out'left downto 2)
            );
        end generate gen25;
        gen63: if TAG = 11 generate
          comp63_1: entity work.comp63
            port map (
              x => bits_in,
              o => bits_out
            );
        end generate gen63;
        genFA: if TAG = 12 generate
          fa_1: entity work.fa
            port map (
              x => bits_in,
              o => bits_out
            );
        end generate genFA;

      end generate genCounter;

      -- Conclusive Chain Construction
      genSumCP: if TAG = RESULT_CP generate
        y(S(i-1)) <= nn(S(i+1));
      end generate genSumCP;

      genSumFA : if TAG = RESULT_FA generate
        signal cout : std_logic;
      begin
        fa : entity work.fa_cc
          port map (
            a    => nn(S(i+1)),
            b    => nn(S(i+2)),
            cin  => nn(S(i+3)),
            s    => y(S(i-1)),
            cout => cout
          );
        genCout : if S(i+4) /= 0 generate
          nn(S(i+4)) <= cout;
        end generate;
      end generate genSumFA;

      genSum42 : if TAG = RESULT_42 generate
        signal gout, cout : std_logic;
      begin
        a42 : entity work.add42_cc
          port map (
            a    => nn(S(i+1)),
            b    => nn(S(i+2)),
            c    => nn(S(i+3)),
            gin  => nn(S(i+4)),
            cin  => nn(S(i+5)),
            s    => y(S(i-1)),
            gout => gout,
            cout => cout
          );
        genGout : if S(i+6) /= 0 generate
          nn(S(i+6)) <= gout;
        end generate;
        genCout : if S(i+7) /= 0 generate
          nn(S(i+7)) <= cout;
        end generate;
      end generate genSum42;

      genBuf : if TAG = STAGE_BUF generate
        constant L : positive := count(S(0 to i), -STAGE_BUF-1);
        constant N : positive := S(i+1); 
        constant R : boolean := (L*DEPTH/D) > ((L-1)*DEPTH/D);
      begin
          genStage: for j in 1 to N generate
-- !!! The Vivado synthesis is running extremely long for a behavioral
--  description of these registers within the compressor. It really hurts
--  badly for DEPTH > 1 and D > 5.
--     Things turn a lot nicer if we just instantiate the register. This
--  is a very frustrating tool deficiency if not even a bug.
--            genReg: if (L*DEPTH/D) > ((L-1)*DEPTH/D) generate
--              process(clk)
--              begin
--                if rising_edge(clk) then
--                  if rst = '1' then
--                    nn(S(i+2*j+1)) <= '0';
--                  else
--                    nn(S(i+2*j+1)) <= nn(S(i+2*j));
--                  end if ;
--                end if;
--              end process;
            ff : FDRE
              port map (
                Q  => nn(S(i+2*j+1)),
                C  => clk,
                CE => '1',
                R  => rst,
                D  => nn(S(i+2*j))
              );
--            end generate genReg;
--            genComb: if (L*DEPTH/D) = ((L-1)*DEPTH/D) generate 
--              nn(S(i+2*j + 1 )) <= nn(S(i+2*j));
            genComb: if not R generate 
              nn(S(i+2*j+1)) <= nn(S(i+2*j));
            end generate genComb;
               
          end generate genStage;
      end generate genBuf;

    end generate genTag;
  end generate genSchedule;

end xil;
