library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity rom_storage is 
port(
  wr_en : in std_logic;
  
  addr : in std_logic_vector( 16 downto 0);
  din : in std_logic_vector( 15 downto 0);
  
  dout : out std_logic_vector( 15 downto 0);

  -- sram bus parameters
  sram_a : out std_logic_vector( 16 downto 0);
  sram_dq : inout std_logic_vector( 15 downto 0); 
  sram_oe_n : out std_logic;
  sram_we_n : out std_logic;
  sram_ub_n : out std_logic; 
  sram_lb_n : out std_logic
);
end rom_storage;

architecture struct of rom_storage is
begin

sram_oe_n <= '0'; -- Output always enabled
sram_we_n <= '0' when wr_en = '1' else '1'; -- write enable only applies when chip select is held
sram_ub_n <= '0';
sram_lb_n <= '0';

sram_a <= addr; --  Address is lower-15 address bits plus 2 bank bits.
sram_dq <= din when wr_en = '1' else "ZZZZZZZZZZZZZZZZ";

dout <= sram_dq(15 downto 0);

end struct;
