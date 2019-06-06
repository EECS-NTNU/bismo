library ieee;
use ieee.std_logic_1164.all;

entity reg is
	generic(
		DataWidth		: positive := 8
		);
	port(
		clk				: in std_logic;
		rst				: in std_logic;
		data_in			: in std_logic_vector(DataWidth - 1 downto 0);
		data_out 		: out std_logic_vector(DataWidth - 1 downto 0)
		);
end reg;

architecture behav of reg is

begin
	process (clk, rst)
	begin
		if rst = '1' then
			data_out <= (others => '0');
		elsif rising_edge(clk) then
			data_out <= data_in;
		end if;
	end process;
end behav;
