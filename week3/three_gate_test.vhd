LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY and_test IS
	PORT (SW: IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		  LEDR: OUT STD_LOGIC_VECTOR(15 DOWNTO 0); -- red LEDs
		  LEDG: OUT STD_LOGIC_VECTOR(7 DOWNTO 0) -- Green LEDs
	);
END and_test;

ARCHITECTURE Structure OF and_test IS
	COMPONENT my_gates
		PORT port(	A: in std_logic;
					B: in std_logic;
					Y_AND: out std_logic;
					Y_OR: out std_logic;
					Y_NOT_A: out std_logic
				 );
	END COMPONENT;

	BEGIN
		LEDR <= SW;
		DUT: my_gates PORT MAP (SW(1),SW(0), LEDG(2),LEDG(1),LEDG(0));
		
END Structure;



library ieee;
use ieee.std_logic_1164.all;

entity my_gates is
port(	A: in std_logic;
	    B: in std_logic;
	    Y_AND: out std_logic;
		Y_OR: out std_logic;
		Y_NOT_A: out std_logic
);
end my_gates;
  
architecture dataflow of my_gates is
begin
    Y_AND <= A and B;
	Y_OR<= A or B;
	Y_NOT_A= not A;
end dataflow;
