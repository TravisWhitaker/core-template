library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bridge_cmd is

port (

clk : in std_logic;
reset_n : out std_logic;

-- Only the host reads/writes the address, so the target relies on host polling
-- to communicate. Read/write are named from the host's perspective, which is
-- why the read side of the bus is an output.

bridge_addr : in std_logic_vector(31 downto 0);
bridge_read : in std_logic;
bridge_read_data : out std_logic_vector(31 downto 0);
bridge_write : in std_logic;
bridge_write_data : out std_logic_vector(31 downto 0);

status_booted : in std_logic;
status_setup_done : in std_logic;
status_running : in std_logic;

);

end bridge_cmd;

architecture rtl of bridge_cmd is

signal host_0 : std_logic_vector(31 downto 0);

signal host_20 : std_logic_vector(31 downto 0);
signal host_24 : std_logic_vector(31 downto 0);
signal host_28 : std_logic_vector(31 downto 0);
signal host_2c : std_logic_vector(31 downto 0);

signal host_40 : std_logic_vector(31 downto 0);
signal host_44 : std_logic_vector(31 downto 0);
signal host_48 : std_logic_vector(31 downto 0);
signal host_4c : std_logic_vector(31 downto 0);

signal host_cmd_start : std_logic;
signal host_cmd_init_val : std_logic_vector(15 downto 0);
signal host_cmd : std_logic_vector(15 downto 0);
signal host_result_code : std_logic_vector(15 downto 0);

type host_state_type is
  ( host_idle
  , host_parse
  , host_work
  , host_done
  );

signal host_state : host_state_type;

signal target_0 : std_logic_vector(31 downto 0);

signal target_20 : std_logic_vector(31 downto 0);
signal target_24 : std_logic_vector(31 downto 0);
signal target_28 : std_logic_vector(31 downto 0);
signal target_2c : std_logic_vector(31 downto 0);

signal target_40 : std_logic_vector(31 downto 0);
signal target_44 : std_logic_vector(31 downto 0);
signal target_48 : std_logic_vector(31 downto 0);
signal target_4c : std_logic_vector(31 downto 0);

type target_state_type is
  ( target_idle
  , target_ready
  , target_dataslot
  , target_wait_rtr
  , target_wait_dso
  );

signal target_state : target_state_type;

begin

bridge_io : process(clk) is
begin

if rising_edge(clk)
then

-- writing seems to take precedence over reading
if bridge_write = '1'
then
  case bridge_addr(31 downto 8) is
  when x"f80000" =>
    -- host command write
    case bridge_addr(7 downto 0) is
    when x"00" =>
      host_0 <= bridge_write_data;
      if bridge_write_data(31 downto 16) = x"434d"
      then
        host_cmd_init_val <= bridge_write_data(15 downto 0);
        host_cmd_start <= '1';
      end if;
    when x"20" => host_20 <= bridge_write_data;
    when x"24" => host_24 <= bridge_write_data;
    when x"28" => host_28 <= bridge_write_data;
    when x"2c" => host_2c <= bridge_write_data;
    end case;
  when x"f80010" =>
    -- host is writing to target region
    case bridge_addr(7 downto 0) is
    when x"00" => target_0 <= bridge_write_data;
    when x"40" => target_40 <= bridge_write_data;
    when x"44" => target_44 <= bridge_write_data;
    when x"48" => target_48 <= bridge_write_data;
    when x"4c" => target_4c <= bridge_write_data;
    end case;
  when x"f80020" =>
    -- host is doing data slot things, ignore for now
    null;
  end case;
elsif bridge_read = '1'
then
  case bridge_addr(31 downto 8) is
  when x"f80000" =>
    case bridge_addr(7 downto 0) is
    when x"00" => bridge_read_data <= host_0;
    when x"04" => bridge_read_data <= x"00000020"
    when x"08" => bridge_read_data <= x"00000040"
    when x"40" => bridge_read_data <= host_40;
    when x"44" => bridge_read_data <= host_44;
    when x"48" => bridge_read_data <= host_48;
    when x"4c" => bridge_read_data <= host_4c;
    end case;
  when x"f80010" =>
    case bridge_addr(7 downto 0) is
    when x"00" => bridge_read_data <= target_0;
    when x"04" => bridge_read_data <= x"00000020"
    when x"08" => bridge_read_data <= x"00000040"
    when x"20" => bridge_read_data <= target_20;
    when x"24" => bridge_read_data <= target_24;
    when x"28" => bridge_read_data <= target_28;
    when x"2c" => bridge_read_data <= target_2c;
    end case;
  when x"f80020" =>
    -- host is doing data slot things, ignore for now
    null;
  end case;
end if;

case host_state is
when host_idle =>
  if host_cmd_start = '1'
  then
    host_cmd_start <= '0';
    host_cmd <= host_cmd_init_val;
    host_state <= host_parse;
  end if;
when host_parse =>
  -- set busy signal
  host_0 <= x"4255" & host_cmd;
  case host_cmd is
  when x"0000" =>
    -- request status
    with std_logic_vector'(status_booted & status_setup_done & status_running) select
      host_result_code <= x"0001" when "000",
                          x"0002" when "100",
                          x"0003" when "110",
                          x"0004" when "111",
                          x"0000" when others;
    host_state <= host_done;
  when x"0010" =>
    -- reset enter
    reset_n <= '0';
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"0011" =>
    -- reset exit
    reset_n <= '1';
    host_result_code <= x"0000";
    host_state <= host_done:
  end case;
  when x"0080" =>
    -- data slot request read, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"0082" =>
    -- data slot request write, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"008a" =>
    -- data slot update, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"008f" =>
    -- data slot access complete, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"0090" =>
    -- rtc report, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"00a0" =>
    -- start save state, nothing for now
    host_40 <= x"00000000";
    host_44 <= x"00000000";
    host_48 <= x"00000000";
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"00a4" =>
    -- load save state, nothing for now
    host_40 <= x"00000000";
    host_44 <= x"00000000";
    host_48 <= x"00000000";
    host_result_code <= x"0000";
    host_state <= host_done;
  when x"00b0" =>
    -- menu is open, nothing for now
    host_result_code <= x"0000";
    host_state <= host_done;
  end case;
when host_work =>
  host_state <= host_idle;
when host_done =>
  host_0 <= x"4f4b" & host_result_code
  host_state <= host_idle;
end case;

case target_state is
when target_idle =>
  if status_setup_done = '1'
  then
    target_state <= target_ready;
  end if;
when target_ready =>
  target_0 <= x"636d0140";
  target_state <= target_wait_rtr;
when target_dataslot =>
  -- nothing for now
  target_state <= target_idle;
when target_wait_rtr =>
  if target_0(31 downto 16) = x"6f6b"
  then
    target_state <= target_idle;
  end if;
when target_wait_dso =>
  -- nothing for now
  target_state <= target_idle;
end case;

-- end if clock edge
end if;


end rtl;
