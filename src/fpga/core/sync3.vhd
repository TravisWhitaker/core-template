library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sync3 is

generic (
    width : natural := 1
);

port (
    i : in std_logic_vector(width-1 downto 0);
    o : out std_logic_vector(width-1 downto 0);
    clk : in std_logic;
    rise : out std_logic;
    fall : out std_logic
);

end sync3;

architecture rtl of sync3 is

signal s1 : std_logic_vector(width-1 downto 0);
signal s2 : std_logic_vector(width-1 downto 0);
signal s3 : std_logic_vector(width-1 downto 0);
signal ooo : std_logic_vector(width-1 downto 0);

begin

process(clk)

begin

if rising_edge(clk)
then
  s3 <= ooo;
  ooo <= s2;
  s2 <= s1;
  s1 <= i;

end if;

end process;

o <= ooo;
rise <= '1' when width = 1 and ooo(0) = '1' and s3(0) = '0' else '0';
fall <= '1' when width = 1 and ooo(0) = '0' and s3(0) = '1' else '0';

end rtl;
