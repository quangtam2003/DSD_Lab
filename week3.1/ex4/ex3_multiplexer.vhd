LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY ex3_multiplexer IS	
	PORT (SW: IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		  LEDR: OUT STD_LOGIC_VECTOR(15 DOWNTO 0); -- red LEDs
		  LEDG: OUT STD_LOGIC_VECTOR(7 DOWNTO 0)); -- Green LEDs
END ex3_multiplexer;

ARCHITECTURE Structure OF ex3_multiplexer IS
	COMPONENT test
		PORT (	I0: in std_logic;
					I1: in std_logic;
					Sel: in std_logic;
					Y: out std_logic
					
				 );
	END COMPONENT;

	BEGIN
		LEDR <= SW;
		DUT: test PORT MAP (SW(2),SW(1),SW(0),LEDG(0));
		
END Structure;

library ieee;
use ieee.std_logic_1164.all;

entity test is
port(	

I0: in std_logic;
I1: in std_logic;
Sel: in std_logic;
Y: out std_logic
);
end test;
  
architecture dataflow of test is
begin
   Y <= (I0 and ( not Sel )) or (I1 and Sel);
end dataflow;