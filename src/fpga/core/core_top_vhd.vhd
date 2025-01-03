library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- use ieee.std_logic_misc.or_reduce;

entity core_top_vhd is

port (
---------------------------------------------------
-- clock ins 74.25mhz. not phase aligned, so treat these domains as asynchronous

clk_74a : in std_logic;
clk_74b : in std_logic;

---------------------------------------------------
-- cartridge interface
-- switches between 3.3v and 5v mechanically
-- out enable for multibit translators controlled by pic32

-- GBA AD[15:8]
cart_tran_bank2 : inout std_logic_vector(7 downto 0);
cart_tran_bank2_dir : out std_logic;

-- GBA AD[7:0]
cart_tran_bank3 : inout std_logic_vector(7 downto 0);
cart_tran_bank3_dir : out std_logic;

-- GBA A[23:16]
cart_tran_bank1 : inout std_logic_vector(7 downto 0);
cart_tran_bank1_dir : out std_logic;

-- GBA [7] PHI#
-- GBA [6] WR#
-- GBA [5] RD#
-- GBA [4] CS1#-CS#
--     [3:0] und
cart_tran_bank0 : inout std_logic_vector(7 downto 4);
cart_tran_bank0_dir : out std_logic;

-- GBA CS2#-RES#
cart_tran_pin30 : inout std_logic;
cart_tran_pin30_dir : out std_logic;
-- when GBC cart is inserted, this signal when low or weak will pull GBC -RES low with a special circuit
-- the goal is that when unconfigured, the FPGA weak pullups won't interfere.
-- thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
-- and general IO drive this pin.
cart_pin30_pwroff_reset : out std_logic;

-- GBA IRQ-DRQ
cart_tran_pin31 : inout std_logic;
cart_tran_pin31_dir : out std_logic;

-- infrared
port_ir_rx : in std_logic;
port_ir_tx : out std_logic;
port_ir_rx_disable : out std_logic;

-- GBA link port
port_tran_si : inout std_logic;
port_tran_si_dir : out std_logic;
port_tran_so : inout std_logic;
port_tran_so_dir : out std_logic;
port_tran_sck : inout std_logic;
port_tran_sck_dir : out std_logic;
port_tran_sd : inout std_logic;
port_tran_sd_dir : out std_logic;
 
---------------------------------------------------
-- cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

cram0_a : out std_logic_vector(21 downto 16);
cram0_dq : inout std_logic_vector(15 downto 0);
cram0_wait : in std_logic;
cram0_clk : out std_logic;
cram0_adv_n : out std_logic;
cram0_cre : out std_logic;
cram0_ce0_n : out std_logic;
cram0_ce1_n : out std_logic;
cram0_oe_n : out std_logic;
cram0_we_n : out std_logic;
cram0_ub_n : out std_logic;
cram0_lb_n : out std_logic;

cram1_a : out std_logic_vector(21 downto 16);
cram1_dq : inout std_logic_vector(15 downto 0);
cram1_wait : in std_logic;
cram1_clk : out std_logic;
cram1_adv_n : out std_logic;
cram1_cre : out std_logic;
cram1_ce0_n : out std_logic;
cram1_ce1_n : out std_logic;
cram1_oe_n : out std_logic;
cram1_we_n : out std_logic;
cram1_ub_n : out std_logic;
cram1_lb_n : out std_logic;

---------------------------------------------------
-- sdram, 512mbit 16bit

dram_a : out std_logic_vector(12 downto 0);
dram_ba : out std_logic_vector(1 downto 0);
dram_dq : inout std_logic_vector(15 downto 0);
dram_dqm : out std_logic_vector(1 downto 0);
dram_clk : out std_logic;
dram_cke : out std_logic;
dram_ras_n : out std_logic;
dram_cas_n : out std_logic;
dram_we_n : out std_logic;

---------------------------------------------------
-- sram, 1mbit 16bit

sram_a : out std_logic_vector(16 downto 0);
sram_dq : inout std_logic_vector(15 downto 0);
sram_oe_n : out std_logic;
sram_we_n : out std_logic;
sram_ub_n : out std_logic;
sram_lb_n : out std_logic;

---------------------------------------------------
-- vblank driven by dock for sync in a certain mode

vblank : in std_logic;

---------------------------------------------------
-- i-o to 6515D breakout usb uart

dbg_tx : out std_logic;
dbg_rx : in std_logic;

---------------------------------------------------
-- i-o pads near jtag connector user can solder to

user1 : out std_logic;
user2 : in std_logic;

---------------------------------------------------
-- RFU internal i2c bus 

aux_sda : inout std_logic;
aux_scl : out std_logic;

---------------------------------------------------
-- RFU, do not use
vpll_feed : out std_logic;


--
-- logical connections
--

---------------------------------------------------
-- video, audio out to scaler
video_rgb : out std_logic_vector(23 downto 0);
video_rgb_clock : out std_logic;
video_rgb_clock_90 : out std_logic;
video_de : out std_logic;
video_skip : out std_logic;
video_vs : out std_logic;
video_hs : out std_logic;
    
audio_mclk : out std_logic;
audio_adc : in std_logic;
audio_dac : out std_logic;
audio_lrck : out std_logic;

---------------------------------------------------
-- bridge bus connection
-- synchronous to clk_74a
bridge_endian_little : out std_logic;
bridge_addr : in std_logic_vector(31 downto 0);
bridge_rd : in std_logic;
bridge_rd_data : out std_logic_vector(31 downto 0);
bridge_wr : in std_logic;
bridge_wr_data : in std_logic_vector(31 downto 0);

---------------------------------------------------
-- controller data
-- 
-- key bitmap:
--   [0]    dpad_up
--   [1]    dpad_down
--   [2]    dpad_left
--   [3]    dpad_right
--   [4]    face_a
--   [5]    face_b
--   [6]    face_x
--   [7]    face_y
--   [8]    trig_l1
--   [9]    trig_r1
--   [10]   trig_l2
--   [11]   trig_r2
--   [12]   trig_l3
--   [13]   trig_r3
--   [14]   face_select
--   [15]   face_start
--   [31:28] type
-- joy values - unsigned
--   [ 7: 0] lstick_x
--   [15: 8] lstick_y
--   [23:16] rstick_x
--   [31:24] rstick_y
-- trigger values - unsigned
--   [ 7: 0] ltrig
--   [15: 8] rtrig
--
cont1_key : in std_logic_vector(31 downto 0);
cont2_key : in std_logic_vector(31 downto 0);
cont3_key : in std_logic_vector(31 downto 0);
cont4_key : in std_logic_vector(31 downto 0);
cont1_joy : in std_logic_vector(31 downto 0);
cont2_joy : in std_logic_vector(31 downto 0);
cont3_joy : in std_logic_vector(31 downto 0);
cont4_joy : in std_logic_vector(31 downto 0);
cont1_trig : in std_logic_vector(15 downto 0);
cont2_trig : in std_logic_vector(15 downto 0);
cont3_trig : in std_logic_vector(15 downto 0);
cont4_trig : in std_logic_vector(15 downto 0)

);

end core_top_vhd;

architecture rtl of core_top_vhd is

-- Quartus doesn't seem to support direct entity instantiation of Verilog
-- components...
component core_bridge_cmd port
(

    clk : in std_logic;
    reset_n : out std_logic;

    bridge_endian_little : in std_logic;
    bridge_addr : in std_logic_vector(31 downto 0);
    bridge_rd : in std_logic;
    bridge_rd_data : out std_logic_vector(31 downto 0);
    bridge_wr : in std_logic;
    bridge_wr_data : in std_logic_vector(31 downto 0);

    -- all these signals should be synchronous to clk
    -- add synchronizers if these need to be used in other clock domains
    status_boot_done : in std_logic;           -- assert when PLLs lock and logic is ready
    status_setup_done : in std_logic;          -- assert when core is happy with what's been loaded into it
    status_running : in std_logic;             -- assert when pocket's taken core out of reset and is running

    dataslot_requestread : out std_logic;
    dataslot_requestread_id : out std_logic_vector(15 downto 0);
    dataslot_requestread_ack : in std_logic;
    dataslot_requestread_ok : in std_logic;

    dataslot_requestwrite : out std_logic;
    dataslot_requestwrite_id : out std_logic_vector(15 downto 0);
    dataslot_requestwrite_size : out std_logic_vector(31 downto 0);
    dataslot_requestwrite_ack : in std_logic;
    dataslot_requestwrite_ok : in std_logic;

    dataslot_update : out std_logic;
    dataslot_update_id : out std_logic_vector(15 downto 0);
    dataslot_update_size : out std_logic_vector(31 downto 0);

    dataslot_allcomplete : out std_logic;

    rtc_epoch_seconds : out std_logic_vector(31 downto 0);
    rtc_date_bcd : out std_logic_vector(31 downto 0);
    rtc_time_bcd : out std_logic_vector(31 downto 0);
    rtc_valid : out std_logic;

    savestate_supported : in std_logic;
    savestate_addr : in std_logic_vector(31 downto 0);
    savestate_size : in std_logic_vector(31 downto 0);
    savestate_maxloadsize : in std_logic_vector(31 downto 0);

    osnotify_inmenu : out std_logic;

    savestate_start : out std_logic;        -- core should detect rising edge on this;
    savestate_start_ack : in std_logic;    -- and then assert ack for at least 1 cycle
    savestate_start_busy : in std_logic;   -- assert constantly while in progress after ack
    savestate_start_ok : in std_logic;     -- assert continuously when done; and clear when new process is started
    savestate_start_err : in std_logic;    -- assert continuously on error; and clear when new process is started

    savestate_load : out std_logic;
    savestate_load_ack : in std_logic;
    savestate_load_busy : in std_logic;
    savestate_load_ok : in std_logic;
    savestate_load_err : in std_logic;

    target_dataslot_read : in std_logic;       -- rising edge triggered
    target_dataslot_write : in std_logic;
    target_dataslot_getfile : in std_logic;
    target_dataslot_openfile : in std_logic;

    target_dataslot_ack : out std_logic;        -- asserted upon command start until completion
    target_dataslot_done : out std_logic;       -- asserted upon command finish until next command is issued    
    target_dataslot_err : out std_logic_vector(2 downto 0);        -- contains result of command execution. zero is OK

    target_dataslot_id : in std_logic_vector(15 downto 0);         -- parameters for each of the read-reload-write commands
    target_dataslot_slotoffset : in std_logic_vector(31 downto 0);
    target_dataslot_bridgeaddr : in std_logic_vector(31 downto 0);
    target_dataslot_length : in std_logic_vector(31 downto 0);

    target_buffer_param_struct : in std_logic_vector(31 downto 0); -- bus address of the memory region APF will fetch additional parameter struct from
    target_buffer_resp_struct : in std_logic_vector(31 downto 0);  -- bus address of the memory region APF will write its response struct to

    datatable_addr : in std_logic_vector(9 downto 0);
    datatable_wren : in std_logic;
    datatable_data : in std_logic_vector(31 downto 0);
    datatable_q : out std_logic_vector(31 downto 0)

);
end component;

signal bridge_endian_little_int : std_logic;

component synch_3 generic
(
    WIDTH : natural := 1
);
port
(
    i    : in  std_logic_vector(WIDTH-1 downto 0);
    o    : out std_logic_vector(WIDTH-1 downto 0);
    clk  : in  std_logic;
    rise : out std_logic;
    fall : out std_logic
);
end component;

component mf_pllbase port
(
    refclk : in std_logic;
    rst : in std_logic;
    outclk_0 : out std_logic;
    outclk_1 : out std_logic;
    outclk_2 : out std_logic;
    outclk_3 : out std_logic;
    outclk_4 : out std_logic;
    locked : out std_logic
);
end component;
-- host-target command handler
--
signal reset_n : std_logic; -- driven by host commands, can be used as core-wide reset
signal cmd_bridge_rd_data : std_logic_vector(31 downto 0);

-- bridge host commands
-- synchronous to clk_74a
signal status_boot_done : std_logic; 
signal status_setup_done : std_logic; -- rising edge triggers a target command
signal status_running : std_logic; -- we are running as soon as reset_n goes high

signal dataslot_requestread : std_logic;
signal dataslot_requestread_id : std_logic_vector(15 downto 0);
signal dataslot_requestread_ack : std_logic;
signal dataslot_requestread_ok : std_logic;

signal dataslot_requestwrite : std_logic;
signal dataslot_requestwrite_id : std_logic_vector(15 downto 0);
signal dataslot_requestwrite_size : std_logic_vector(31 downto 0);
signal dataslot_requestwrite_ack : std_logic;
signal dataslot_requestwrite_ok : std_logic;

signal dataslot_update : std_logic;
signal dataslot_update_id : std_logic_vector(15 downto 0);
signal dataslot_update_size : std_logic_vector(31 downto 0);
    
signal dataslot_allcomplete : std_logic;

signal rtc_epoch_seconds : std_logic_vector(31 downto 0);
signal rtc_date_bcd : std_logic_vector(31 downto 0);
signal rtc_time_bcd : std_logic_vector(31 downto 0);
signal rtc_valid : std_logic;

signal savestate_supported : std_logic;
signal savestate_addr : std_logic_vector(31 downto 0);
signal savestate_size : std_logic_vector(31 downto 0);
signal savestate_maxloadsize : std_logic_vector(31 downto 0);

signal savestate_start : std_logic;
signal savestate_start_ack : std_logic;
signal savestate_start_busy : std_logic;
signal savestate_start_ok : std_logic;
signal savestate_start_err : std_logic;

signal savestate_load : std_logic;
signal savestate_load_ack : std_logic;
signal savestate_load_busy : std_logic;
signal savestate_load_ok : std_logic;
signal savestate_load_err : std_logic;
    
signal osnotify_inmenu : std_logic;

-- bridge target commands
-- synchronous to clk_74a
signal target_dataslot_read : std_logic;       
signal target_dataslot_write : std_logic;
signal target_dataslot_getfile : std_logic;  -- require additional param-resp structs to be mapped
signal target_dataslot_openfile : std_logic; -- require additional param-resp structs to be mapped
    
signal target_dataslot_ack : std_logic;        
signal target_dataslot_done : std_logic;
signal target_dataslot_err : std_logic_vector(2 downto 0);

signal target_dataslot_id : std_logic_vector(15 downto 0);
signal target_dataslot_slotoffset : std_logic_vector(31 downto 0);
signal target_dataslot_bridgeaddr : std_logic_vector(31 downto 0);
signal target_dataslot_length : std_logic_vector(31 downto 0);
    
signal target_buffer_param_struct : std_logic_vector(31 downto 0); -- to be mapped-implemented when using some Target commands
signal target_buffer_resp_struct : std_logic_vector(31 downto 0);  -- to be mapped-implemented when using some Target commands

-- bridge data slot access
-- synchronous to clk_74a
signal datatable_addr : std_logic_vector(9 downto 0);
signal datatable_wren : std_logic;
signal datatable_data : std_logic_vector(31 downto 0);
signal datatable_q : std_logic_vector(31 downto 0);

-- video constants
constant VID_V_BPORCH : natural := 10;
constant VID_V_ACTIVE : natural := 240;
constant VID_V_TOTAL : natural := 512;
constant VID_H_BPORCH : natural := 10;
constant VID_H_ACTIVE : natural := 320;
constant VID_H_TOTAL : natural := 400;

-- video gen sigs

signal frame_count : unsigned(15 downto 0);
    
signal x_count : unsigned(9 downto 0);
signal y_count : unsigned(9 downto 0);
    
-- signal visible_x : std_logic_vector(9 downto 0); = x_count - VID_H_BPORCH;
-- signal visible_y : std_logic_vector(9 downto 0); = y_count - VID_V_BPORCH;

signal vidout_rgb : std_logic_vector(23 downto 0);
signal vidout_de, vidout_de_1 : std_logic;
signal vidout_skip : std_logic;
signal vidout_vs : std_logic;
signal vidout_hs, vidout_hs_1 : std_logic;
    
-- signal square_x : std_logic_vector(9 downto 0); = 'd135;
-- signal square_y : std_logic_vector(9 downto 0); = 'd95;

-- audio gen constants/sigs

constant CYCLE_48KHZ : natural := 245760;

signal audgen_accum : unsigned(21 downto 0);
signal audgen_mclk : std_logic;

signal aud_mclk_divider : unsigned(1 downto 0);
signal audgen_sclk : std_logic;
signal audgen_lrck_1 : std_logic;

signal audgen_lrck_cnt : unsigned(4 downto 0);    
signal audgen_lrck : std_logic;
signal audgen_dac : std_logic;

-- pll sigs

signal clk_core_12288 : std_logic;
signal clk_core_12288_90deg : std_logic;
signal pll_core_locked : std_logic;
signal pll_core_locked_s : std_logic;

begin

-- not using the IR port, so turn off both the LED, and
-- disable the receive circuit to save power
port_ir_tx <= '0';
port_ir_rx_disable <= '1';

-- bridge endianness
bridge_endian_little_int <= '0';
bridge_endian_little <= bridge_endian_little_int;

-- cart is unused, so set all level translators accordingly
-- directions are 0:IN, 1:OUT
cart_tran_bank3 <= (others => 'Z');
cart_tran_bank3_dir <= '0';
cart_tran_bank2 <= (others => 'Z');
cart_tran_bank2_dir <= '0';
cart_tran_bank1 <= (others => 'Z');
cart_tran_bank1_dir <= '0';
cart_tran_bank0 <= (others => '1');
cart_tran_bank0_dir <= '1';
cart_tran_pin30 <= '0';      -- reset or cs2, we let the hw control it by itself
cart_tran_pin30_dir <= 'Z';
cart_pin30_pwroff_reset <= '0';  -- hardware can control this
cart_tran_pin31 <= 'Z';      -- input
cart_tran_pin31_dir <= '0';  -- input

-- link port is unused, set to input only to be safe
-- each bit may be bidirectional in some applications
port_tran_so <= 'Z';
port_tran_so_dir <= '0';     -- SO is output only
port_tran_si <= 'Z';
port_tran_si_dir <= '0';     -- SI is input only
port_tran_sck <= 'Z';
port_tran_sck_dir <= '0';    -- clock direction can change
port_tran_sd <= 'Z';
port_tran_sd_dir <= '0';     -- SD is input and not used


-- tie off the rest of the pins we are not using
cram0_a <= (others => '0');
cram0_dq <= (others => 'Z');
cram0_clk <= '0';
cram0_adv_n <= '1';
cram0_cre <= '0';
cram0_ce0_n <= '1';
cram0_ce1_n <= '1';
cram0_oe_n <= '1';
cram0_we_n <= '1';
cram0_ub_n <= '1';
cram0_lb_n <= '1';

cram1_a <= (others => '0');
cram1_dq <= (others => 'Z');
cram1_clk <= '0';
cram1_adv_n <= '1';
cram1_cre <= '0';
cram1_ce0_n <= '1';
cram1_ce1_n <= '1';
cram1_oe_n <= '1';
cram1_we_n <= '1';
cram1_ub_n <= '1';
cram1_lb_n <= '1';

dram_a <= (others => '0');
dram_ba <= (others => '0');
dram_dq <= (others => 'Z');
dram_dqm <= (others => '0');
dram_clk <= '0';
dram_cke <= '0';
dram_ras_n <= '1';
dram_cas_n <= '1';
dram_we_n <= '1';

sram_a <= (others => '0');
sram_dq <= (others => 'Z');
sram_oe_n  <= '1';
sram_we_n  <= '1';
sram_ub_n  <= '1';
sram_lb_n  <= '1';

dbg_tx <= 'Z';
user1 <= 'Z';
aux_scl <= 'Z';
vpll_feed <= 'Z';

-- for bridge write data, we just broadcast it to all bus devices
-- for bridge read data, we have to mux it
-- add your own devices here
with bridge_addr(31 downto 24) select
    bridge_rd_data <=
        cmd_bridge_rd_data when "11111000",  -- 32'hF8xxxxxx
        (others => '0')    when "00010000",  -- 32'h10xxxxxx
        (others => '0')    when others;

status_boot_done <= pll_core_locked_s; 
status_setup_done <= pll_core_locked_s; -- rising edge triggers a target command
status_running <= reset_n; -- we are running as soon as reset_n goes high

dataslot_requestread_ack <= '1';
dataslot_requestread_ok <= '1';

dataslot_requestwrite_ack <= '1';
dataslot_requestwrite_ok <= '1';

icb : core_bridge_cmd port map (
    clk => clk_74a,
    reset_n => reset_n,

    bridge_endian_little => bridge_endian_little_int,
    bridge_addr => bridge_addr,
    bridge_rd => bridge_rd,
    bridge_rd_data => cmd_bridge_rd_data,
    bridge_wr => bridge_wr,
    bridge_wr_data => bridge_wr_data,
    
    status_boot_done => status_boot_done,
    status_setup_done => status_setup_done,
    status_running => status_running,

    dataslot_requestread => dataslot_requestread,
    dataslot_requestread_id => dataslot_requestread_id,
    dataslot_requestread_ack => dataslot_requestread_ack,
    dataslot_requestread_ok => dataslot_requestread_ok,

    dataslot_requestwrite => dataslot_requestwrite,
    dataslot_requestwrite_id => dataslot_requestwrite_id,
    dataslot_requestwrite_size => dataslot_requestwrite_size,
    dataslot_requestwrite_ack => dataslot_requestwrite_ack,
    dataslot_requestwrite_ok => dataslot_requestwrite_ok,

    dataslot_update => dataslot_update,
    dataslot_update_id => dataslot_update_id,
    dataslot_update_size => dataslot_update_size,
    
    dataslot_allcomplete => dataslot_allcomplete,

    rtc_epoch_seconds => rtc_epoch_seconds,
    rtc_date_bcd => rtc_date_bcd,
    rtc_time_bcd => rtc_time_bcd,
    rtc_valid => rtc_valid,
    
    savestate_supported => savestate_supported,
    savestate_addr => savestate_addr,
    savestate_size => savestate_size,
    savestate_maxloadsize => savestate_maxloadsize,

    savestate_start => savestate_start,
    savestate_start_ack => savestate_start_ack,
    savestate_start_busy => savestate_start_busy,
    savestate_start_ok => savestate_start_ok,
    savestate_start_err => savestate_start_err,

    savestate_load => savestate_load,
    savestate_load_ack => savestate_load_ack,
    savestate_load_busy => savestate_load_busy,
    savestate_load_ok => savestate_load_ok,
    savestate_load_err => savestate_load_err,

    osnotify_inmenu => osnotify_inmenu,
    
    target_dataslot_read => target_dataslot_read,
    target_dataslot_write => target_dataslot_write,
    target_dataslot_getfile => target_dataslot_getfile,
    target_dataslot_openfile => target_dataslot_openfile,
    
    target_dataslot_ack => target_dataslot_ack,
    target_dataslot_done => target_dataslot_done,
    target_dataslot_err => target_dataslot_err,

    target_dataslot_id => target_dataslot_id,
    target_dataslot_slotoffset => target_dataslot_slotoffset,
    target_dataslot_bridgeaddr => target_dataslot_bridgeaddr,
    target_dataslot_length => target_dataslot_length,

    target_buffer_param_struct => target_buffer_param_struct,
    target_buffer_resp_struct => target_buffer_resp_struct,
    
    datatable_addr => datatable_addr,
    datatable_wren => datatable_wren,
    datatable_data => datatable_data,
    datatable_q => datatable_q
);

-- video generation
-- ~12,288,000 hz pixel clock
--
-- we want our video mode of 320x240 @ 60hz, this results in 204800 clocks per frame
-- we need to add hblank and vblank times to this, so there will be a nondisplay area. 
-- it can be thought of as a border around the visible area.
-- to make numbers simple, we can have 400 total clocks per line, and 320 visible.
-- dividing 204800 by 400 results in 512 total lines per frame, and 240 visible.
-- this pixel clock is fairly high for the relatively low resolution, but that's fine.
-- PLL output has a minimum output frequency anyway.

video_rgb_clock <= clk_core_12288;
video_rgb_clock_90 <= clk_core_12288_90deg;
video_rgb <= vidout_rgb;
video_de <= vidout_de;
video_skip <= vidout_skip;
video_vs <= vidout_vs;
video_hs <= vidout_hs;

vid : process(clk_core_12288, reset_n) is
begin

if reset_n = '0'
then
  x_count <= (others => '0');
  y_count <= (others => '0');
elsif rising_edge(clk_core_12288)
then
  vidout_de <= '0';
  vidout_skip <= '0';
  vidout_vs <= '0';
  vidout_hs <= '0';

  vidout_hs_1 <= vidout_hs;
  vidout_de_1 <= vidout_de;

-- x and y counters
  x_count <= x_count + 1;
  if x_count = (VID_H_TOTAL-1)
  then
    x_count <= (others => '0');
    
    y_count <= y_count + 1;
    if y_count = (VID_V_TOTAL-1)
    then
      y_count <= (others => '0');
    end if;
  end if;

-- generate sync 
  if x_count = 0 and y_count = 0
  then
      -- sync signal in back porch
      -- new frame
      vidout_vs <= '1';
      frame_count <= frame_count + 1;
  end if;

-- we want HS to occur a bit after VS, not on the same cycle
  if x_count = 3
  then
    vidout_hs <= '1';
  end if;

-- inactive areas are black:
  vidout_rgb <= (others => '0');
-- else some random color:
  if x_count >= VID_H_BPORCH and x_count < (VID_H_ACTIVE+VID_H_BPORCH)
  then
    if y_count >= VID_V_BPORCH and y_count < (VID_V_ACTIVE+VID_H_BPORCH)
    then
      -- data enable
      vidout_de <= '1';

      vidout_rgb <= "111111110101010100001111";
    end if;
  end if;

end if;

end process;

-- audio i2s silence generator
-- see other examples for actual audio generation

audio_mclk <= audgen_mclk;
audio_dac <= audgen_dac;
audio_lrck <= audgen_lrck;

audgen_sclk <= aud_mclk_divider(1);

aud_mclk : process(clk_74a) is
begin
if rising_edge(clk_74a)
then
  audgen_accum <= audgen_accum + CYCLE_48KHZ;
  if audgen_accum >= 742500
  then
    audgen_mclk <= not audgen_mclk;
    audgen_accum <= audgen_accum - 742500 + CYCLE_48KHZ;
  end if;
end if;
end process;

aud_sclk : process(audgen_mclk) is
begin
if rising_edge(audgen_mclk)
then
  aud_mclk_divider <= aud_mclk_divider + 1;
end if;
end process;

aud_i2s : process(audgen_sclk) is
begin
if rising_edge(audgen_sclk)
then
  audgen_dac <= '0';
  audgen_lrck_cnt <= audgen_lrck_cnt + 1;
  if audgen_lrck_cnt = 31
  then
    audgen_lrck <= not audgen_lrck;
  end if;
end if;
end process;

-- PLL

s01 : synch_3 generic map
(
    WIDTH => 1
)
port map
(
    i(0) => pll_core_locked,
    o(0) => pll_core_locked_s,
    clk => clk_74a
);

mp1 : mf_pllbase port map
(
    refclk => clk_74a,
    rst => '0',
    outclk_0 => clk_core_12288,
    outclk_1 => clk_core_12288_90deg,
    locked => pll_core_locked
);

end rtl;
