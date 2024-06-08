//
//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [15:0]  cont1_key,
input   wire    [15:0]  cont2_key,
input   wire    [15:0]  cont3_key,
input   wire    [15:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is input only
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;

assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

//assign dram_a = 'h0;
//assign dram_ba = 'h0;
//assign dram_dq = {16{1'bZ}};
//assign dram_dqm = 'h0;
//assign dram_clk = 'h0;
//assign dram_cke = 'h0;
//assign dram_ras_n = 'h1;
//assign dram_cas_n = 'h1;
//assign dram_we_n = 'h1;

//assign sram_a = 'h0;
//assign sram_dq = {16{1'bZ}};
//assign sram_oe_n  = 1;
//assign sram_we_n  = 1;
//assign sram_ub_n  = 1;
//assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;

// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        bridge_rd_data <= 0;
    end
    32'h10xxxxxx: begin
        // example
        // bridge_rd_data <= example_device_data;
        bridge_rd_data <= 0;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked; 
    wire            status_setup_done = pll_core_locked; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_allcomplete;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a


// bridge data slot access

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q ),

);

// Game type defines
parameter [1:0] GAME_ID_GAUNTLET_4P 	 = 0;
parameter [1:0] GAME_ID_GAUNTLET_2P 	 = 1;
parameter [1:0] GAME_ID_GAUNTLET2 		 = 2;
parameter [1:0] GAME_ID_VINDICATORS_PT2  = 3;

reg [1:0] game_id;
reg service_mode;

parameter [1:0] MUX_ID_WARRIOR  = 0;
parameter [1:0] MUX_ID_VALKYRIE = 1;
parameter [1:0] MUX_ID_WIZARD   = 2;
parameter [1:0] MUX_ID_ELF      = 3;

reg [1:0] p1_mux = MUX_ID_WARRIOR;
reg [1:0] p2_mux = MUX_ID_VALKYRIE;
reg [1:0] p3_mux = MUX_ID_WIZARD;
reg [1:0] p4_mux = MUX_ID_ELF;

always @(posedge clk_74a) begin
  if(bridge_wr) begin
    casex(bridge_addr)
       32'h80000000: begin 
			game_id		 	<= bridge_wr_data[2:0];  
	   end
	   32'h90000000: begin 
            p1_mux  <= bridge_wr_data[1:0];
            p2_mux  <= bridge_wr_data[3:2];
            p3_mux  <= bridge_wr_data[5:4];
            p4_mux  <= bridge_wr_data[7:6];
		end
	   32'hA0000000: begin 
			service_mode 	<= bridge_wr_data[0];
	   end
       32'hB0000000: begin
            do_reset <= ~do_reset;
       end
    endcase
  end
end

///////////////////////////////////////////////
// System
///////////////////////////////////////////////

wire osnotify_inmenu_s;

synch_3 OSD_S (osnotify_inmenu, osnotify_inmenu_s, clk_sys);

///////////////////////////////////////////////
// ROM
///////////////////////////////////////////////

reg         ioctl_download = 0;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;

always @(posedge clk_74a) begin
    if (dataslot_requestwrite)     ioctl_download <= 1;
    else if (dataslot_allcomplete) ioctl_download <= 0;
end

data_loader #(
    .ADDRESS_MASK_UPPER_4(0),
    .ADDRESS_SIZE(25),
	 .WRITE_MEM_CLOCK_DELAY(13),
	 .WRITE_MEM_EN_CYCLE_LENGTH(1)
) rom_loader (
    .clk_74a(clk_74a),
    .clk_memory(clk_sys),

    .bridge_wr(bridge_wr),
    .bridge_endian_little(bridge_endian_little),
    .bridge_addr(bridge_addr),
    .bridge_wr_data(bridge_wr_data),

    .write_en(ioctl_wr),
    .write_addr(ioctl_addr),
    .write_data(ioctl_dout)
);

reg [23:0] acc_bytes = 0;

wire [22:0] sdram_addr;
reg  [31:0] sdram_data = 32'b0;
reg         sdram_we = 1'b0;
wire 	      sdram_ready;
wire        ap_en;
wire        gp_en;
wire        mp_en;

// the order in which the files are listed in the .mra file determines the order in which they appear here on the HPS bus
// some files are interleaved as DWORD, some are interleaved as WORD and some are not interleaved and appear as BYTEs
// acc_bytes collects previous bytes so that when a WORD or DWORD is complete it is written to the RAM as appropriate
always @(posedge clk_sys)
	if (ioctl_wr && ioctl_download )
		acc_bytes<={acc_bytes[15:0],ioctl_dout}; // accumulate previous bytes

always @(posedge clk_sys)
begin
	sdram_we <= 1'b0;
	if (ioctl_wr && ioctl_download && ioctl_addr[1] && ioctl_addr[0])
	begin
		sdram_data <= {acc_bytes,ioctl_dout};
		sdram_we <= 1'b1;
	end
end

// Synchronize the loading of the sram on each WORD
reg [15:0] sram_latched;
reg [16:0] sram_addr_latched;
reg sram_en_latched;

always @(posedge clk_sys)
begin
   if (ioctl_wr && ioctl_download && ioctl_addr[0])
   begin
      sram_latched <= {acc_bytes[7:0], ioctl_dout};
		sram_addr_latched <= ioctl_addr[17:1];		
		sram_en_latched <= mp_wr_3_5_6_7;
   end  
end

assign sdram_addr = ioctl_download ? ioctl_addr[24:2] : {5'd0,gp_addr};

reg do_reset;
reg last_do_reset;
reg manual_reset = 1'b0;
reg [24:0] reset_count = 25'b0;

always @(posedge clk_14) begin
	last_do_reset <= do_reset;
	if (~manual_reset && (do_reset != last_do_reset)) begin
		reset_count <= 25'd1;
		manual_reset <= 1'b1;
	end
	else begin
		if (reset_count == 25'b0001111111111111111111111) begin
			reset_count <= 25'b0;
			manual_reset <= 1'b0;
		end
		else begin
			reset_count <= reset_count + 25'd1;
		end
	end
end

sdram #(.fCK_Mhz(93.06817)) sdram
(
	.I_RST(~pll_core_locked),
	.I_CLK(clk_93),

	// controller interface
	.I_ADDR(sdram_addr),
	.I_DATA(sdram_data),
	.I_WE(sdram_we),
	.O_RDY(sdram_ready),
	.O_DATA(gp_data),

	// SDRAM interface
	.SDRAM_DQ(dram_dq),
	.SDRAM_A(dram_a),
	.SDRAM_BA(dram_ba),
	.SDRAM_DQML(dram_dqm[0]),
	.SDRAM_DQMH(dram_dqm[1]),
	//.SDRAM_CLK(dram_clk),
	.SDRAM_CKE(dram_cke),
	//.SDRAM_nCS(), // always enabled
	.SDRAM_nRAS(dram_ras_n),
	.SDRAM_nCAS(dram_cas_n),
	.SDRAM_nWE(dram_we_n)
);

wire [15:0] sram_in;
wire [16:0] sram_addr;

rom_storage rom_storage (
  .wr_en(sram_en_latched), 	 
  
  .addr(sram_addr), 		 
  .din(sram_in), 		 
  
  .dout(mp_data_3_5_6_7), 		 

  // -- sram bus parameters
  .sram_a(sram_a), 	 
  .sram_dq(sram_dq), 
  .sram_oe_n(sram_oe_n), 
  .sram_we_n(sram_we_n), 
  .sram_ub_n(sram_ub_n), 
  .sram_lb_n(sram_lb_n), 
);

wire sram_wr_en;

wire gp_wr, mp_wr_9A_9B, mp_wr_10A_10B, mp_wr_7A_7B, mp_wr_6A_6B, mp_wr_5A_5B, mp_wr_3A_3B, ap_wr_16R, ap_wr_16S, cp_wr_6P;
wire mp_wr_3_5_6_7;

wire [15:0] ap_addr;
wire [ 7:0] ap_data, ap_data_16R, ap_data_16S;

wire [17:0] gp_addr;
wire [31:0] gp_data;

wire [18:0] mp_addr;
wire [15:0] mp_data, mp_data_9A_9B, mp_data_10A_10B, mp_data_3_5_6_7; 
wire [13:0] cp_addr;
wire [ 7:0] cp_data;

wire [ 7:0] r4_addr;
wire [ 3:0] r4_data, r4_data_G1, r4_data_G2, r4_data_V2;

// ioctl_addr 000000..01FFFF = video ROMs 2L  2A  1L  1A  (4*32KB)
// ioctl_addr 020000..03FFFF = video ROMs 2MN 2B  1MN 1B  (4*32KB)
// ioctl_addr 040000..05FFFF = video ROMs 2P  2C  1P  1C  (4*32KB)
// ioctl_addr 060000..07FFFF = video ROMs 2R  2D  1R  1D  (4*32KB)
// ioctl_addr 080000..09FFFF = video ROMs 2ST 2EF 1ST 1EF (4*32KB)
// ioctl_addr 0A0000..0BFFFF = video ROMs 2U  2J  1U  1J  (4*32KB)
assign gp_wr         = (ioctl_wr && ioctl_addr[24:18] < 7'h03);
// ioctl_addr 0C0000..0CFFFF = CPU ROMS 9A 9B (2*32KB)
assign mp_wr_9A_9B   = (ioctl_wr && ioctl_addr[24:16]== 9'h0C && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0;
// ioctl_addr 0D0000..0DFFFF = NO ROM (2*32KB filler)
// ioctl_addr 0E0000..0EFFFF = NO ROM (2*32KB filler)
// ioctl_addr 0F0000..0F7FFF = CPU ROMS 10A 10B (2*16KB)
assign mp_wr_10A_10B = (ioctl_wr && ioctl_addr[24:15]==10'h1E && ioctl_addr[0]==1'b1) ? 1'b1 : 1'b0;
// 
// ioctl_addr 0F8000..0FFFFF = NO ROMS (2*16K filler)
// ioctl_addr 100000..10FFFF = CPU ROMS 7A 7B (2*32KB)
// ioctl_addr 110000..11FFFF = CPU ROMS 6A 6B (2*32KB)
// ioctl_addr 120000..12FFFF = CPU ROMS 5A 5B (2*32KB)
// ioctl_addr 130000..13FFFF = CPU ROMS 3A 3B (2*32KB)
assign mp_wr_7A_7B   = (ioctl_wr && ioctl_addr[24:16]== 9'h10);
assign mp_wr_6A_6B   = (ioctl_wr && ioctl_addr[24:16]== 9'h11);
assign mp_wr_5A_5B   = (ioctl_wr && ioctl_addr[24:16]== 9'h12);
assign mp_wr_3A_3B   = (ioctl_wr && ioctl_addr[24:16]== 9'h13);


// ioctl_addr 140000..147FFF = AUDIO ROM 16S (32KB)
assign ap_wr_16S     = (ioctl_wr && ioctl_addr[24:15]==10'h28 ) ? 1'b1 : 1'b0;
// ioctl_addr 148000..14BFFF = AUDIO ROM 16R (16KB)
assign ap_wr_16R     = (ioctl_wr && ioctl_addr[24:14]==11'h52 ) ? 1'b1 : 1'b0;
// ioctl_addr 14C000..14FFFF = CHAR ROM 6P (16KB)
assign cp_wr_6P      = (ioctl_wr && ioctl_addr[24:14]==11'h53 ) ? 1'b1 : 1'b0;

assign mp_data =
	mp_addr[18:15] == 4'b0000  ? mp_data_9A_9B :
	mp_addr[18:14] == 5'b00110 ? mp_data_10A_10B :
	mp_addr[18:17] == 2'b01    ? mp_data_3_5_6_7 :
	{ 16'h0 };
	
assign mp_wr_3_5_6_7 = mp_wr_3A_3B | mp_wr_5A_5B | mp_wr_6A_6B | mp_wr_7A_7B;

assign ap_data = ap_addr[15] ? ap_data_16S : ap_data_16R;

assign sram_addr = ioctl_download ? {sram_addr_latched} : mp_addr[16:0];
assign sram_in = sram_latched;

/*************************************************************/
// ** Moved to SDRAM **
// 256 M10K blocks
//	dpram #(16,32) gp_ram
//	(.clock_a(clk_sys    ), .enable_a(), .wren_a(gp_wr        ), .address_a(ioctl_addr[17:2]), .data_a({acc_bytes[23:0],ioctl_dout}), .q_a(               ),
//	 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   gp_addr[15:0]), .data_b(                            ), .q_b(gp_data        ));

// 64 M10K blocks
dpram #(15,16) mp_ram_9A_9B
(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_9A_9B  ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[14:0]), .data_b(                            ), .q_b(mp_data_9A_9B   ));

// 32 M10K blocks
dpram #(14,16) mp_ram_10A_10B
(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_10A_10B), .address_a(ioctl_addr[14:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[13:0]), .data_b(                            ), .q_b(mp_data_10A_10B));

// ** MOVED TO SRAM **
// 64 M10K blocks
//dpram #(15,16) mp_ram_7A_7B
//(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_7A_7B  ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
// .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[14:0]), .data_b(                            ), .q_b(mp_data_7A_7B  ));

// 64 M10K blocks
//dpram #(15,16) mp_ram_6A_6B
//(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_6A_6B  ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
// .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[14:0]), .data_b(                            ), .q_b(mp_data_6A_6B  ));

// 64 M10K blocks
//dpram #(15,16) mp_ram_5A_5B
//(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_5A_5B  ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
// .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[14:0]), .data_b(                            ), .q_b(mp_data_5A_5B  ));

// 64 M10K blocks
//dpram #(15,16) mp_ram_3A_3B
//(.clock_a(clk_sys    ), .enable_a(), .wren_a(mp_wr_3A_3B  ), .address_a(ioctl_addr[15:1]), .data_a({acc_bytes[ 7:0],ioctl_dout}), .q_a(               ),
//.clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   mp_addr[14:0]), .data_b(                            ), .q_b(mp_data_3A_3B  ));

// 32 M10K blocks
dpram #(15, 8) ap_ram_16S
(.clock_a(clk_sys    ), .enable_a(), .wren_a(ap_wr_16S    ), .address_a(ioctl_addr[14:0]), .data_a(                 ioctl_dout ), .q_a(               ),
 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   ap_addr[14:0]), .data_b(                            ), .q_b(ap_data_16S    ));

// 16 M10K blocks
dpram #(14, 8) ap_ram_16R
(.clock_a(clk_sys    ), .enable_a(), .wren_a(ap_wr_16R    ), .address_a(ioctl_addr[13:0]), .data_a(                 ioctl_dout ), .q_a(               ),
 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   ap_addr[13:0]), .data_b(                            ), .q_b(ap_data_16R    ));

// 16 M10K blocks
dpram  #(14,8) cp_ram_6P
(.clock_a(clk_sys    ), .enable_a(), .wren_a(cp_wr_6P     ), .address_a(ioctl_addr[13:0]), .data_a(                 ioctl_dout ), .q_a(               ),
 .clock_b(clk_sys    ), .enable_b(), .wren_b(             ), .address_b(   cp_addr[13:0]), .data_b(                            ), .q_b(cp_data        ));

// total game dpram uses 152 of 308 M10K blocks, + 2MBit DRAM
PROM_4R_G1 PROM_4R_G1(.CLK(clk_sys), .ADDR(r4_addr), .DATA(r4_data_G1) );
PROM_4R_G2 PROM_4R_G2(.CLK(clk_sys), .ADDR(r4_addr), .DATA(r4_data_G2) );
PROM_4R_V2 PROM_4R_V2(.CLK(clk_sys), .ADDR(r4_addr), .DATA(r4_data_V2) );

assign r4_data = 
	game_id == GAME_ID_VINDICATORS_PT2 ? r4_data_V2 :
	game_id == GAME_ID_GAUNTLET2 	   ? r4_data_G2 :
	r4_data_G1;

///////////////////////////////////////////////
// Video
///////////////////////////////////////////////

reg hblank_core, vblank_core;
wire hs_core, vs_core;
wire [3:0] r;
wire [3:0] g;
wire [3:0] b;
wire [3:0] i;

wire [3:0] r_conv;
wire [3:0] g_conv;
wire [3:0] b_conv;

// convert input video from 16bit IRGB to 12 bit RGB
RGBI RCONV (.ADDR({i,r}), .DATA(r_conv));
RGBI GCONV (.ADDR({i,g}), .DATA(g_conv));
RGBI BCONV (.ADDR({i,b}), .DATA(b_conv));

reg video_de_reg;
reg video_hs_reg;
reg video_vs_reg;
reg [23:0] video_rgb_reg;

reg hs_prev;
reg vs_prev;

assign video_rgb_clock = clk_core_7;
assign video_rgb_clock_90 = clk_core_7_90deg;

assign video_de = video_de_reg;
assign video_hs = video_hs_reg;
assign video_vs = video_vs_reg;
assign video_rgb = video_rgb_reg;
assign video_skip = 0;

always @(posedge clk_core_7) begin
    video_de_reg <= 0;
    
    video_rgb_reg <= 24'd0;
    if (~(~vblank_core || ~hblank_core)) begin
        video_de_reg <= 1;
        video_rgb_reg[23:16]  <= {2{r_conv}};
        video_rgb_reg[15:8]   <= {2{g_conv}};        
        video_rgb_reg[7:0]    <= {2{b_conv}};
	end    
	
    video_hs_reg <= ~hs_prev && ~hs_core;
    video_vs_reg <= ~vs_prev && ~vs_core;
    hs_prev <= ~hs_core;
    vs_prev <= ~vs_core;
end


///////////////////////////////////////////////
// Audio
///////////////////////////////////////////////

wire [15:0] audio_l;
wire [15:0] audio_r;

sound_i2s #(
    .CHANNEL_WIDTH(16),
    .SIGNED_INPUT(1)
) sound_i2s (
    .clk_74a(clk_74a),
    .clk_audio(clk_14),
    
    .audio_l(audio_l),
    .audio_r(audio_r),

    .audio_mclk(audio_mclk),
    .audio_lrck(audio_lrck),
    .audio_dac(audio_dac)
);


///////////////////////////////////////////////
// Control
///////////////////////////////////////////////

wire [15:0] joy;
wire [15:0] joy2;
wire [15:0] joy3;
wire [15:0] joy4;

synch_3 #(
    .WIDTH(16)
) cont1_key_s (
    cont1_key,
    joy,
    clk_14
);

synch_3 #(
    .WIDTH(16)
) cont2_key_s (
    cont2_key,
    joy2,
    clk_14
);

synch_3 #(
    .WIDTH(16)
) cont3_key_s (
    cont3_key,
    joy3,
    clk_14
);

synch_3 #(
    .WIDTH(16)
) cont4_key_s (
    cont4_key,
    joy4,
    clk_14
);

// Gauntlet Controls (Covers Gauntlet 1/2, both 2 + 4 player variants)

wire player1_up;
wire player1_down;
wire player1_left;
wire player1_right;
wire player1_fire;
wire player1_magic;

wire player2_up;
wire player2_down;
wire player2_left;
wire player2_right;
wire player2_fire;
wire player2_magic;

wire player3_up;
wire player3_down;
wire player3_left;
wire player3_right;
wire player3_fire;
wire player3_magic;

wire player4_up;
wire player4_down;
wire player4_left;
wire player4_right;
wire player4_fire;
wire player4_magic;

assign player1_up    = joy[0];
assign player1_down  = joy[1];
assign player1_left  = joy[2];
assign player1_right = joy[3];
assign player1_fire  = joy[5];
assign player1_magic = joy[4];

assign player2_up    = joy2[0];
assign player2_down  = joy2[1];
assign player2_left  = joy2[2];
assign player2_right = joy2[3];
assign player2_fire  = joy2[5];
assign player2_magic = joy2[4];

assign player3_up    = joy3[0];
assign player3_down  = joy3[1];
assign player3_left  = joy3[2];
assign player3_right = joy3[3];
assign player3_fire  = joy3[5];
assign player3_magic = joy3[4];

assign player4_up    = joy4[0];
assign player4_down  = joy4[1];
assign player4_left  = joy4[2];
assign player4_right = joy4[3];
assign player4_fire  = joy4[5];
assign player4_magic = joy4[4];

wire [7:0] player1_controls;
wire [7:0] player2_controls;
wire [7:0] player3_controls;
wire [7:0] player4_controls;

assign player1_controls = {~player1_up, ~player1_down, ~player1_left, ~player1_right, 1'b1, 1'b1, ~player1_fire, ~player1_magic};
assign player2_controls = {~player2_up, ~player2_down, ~player2_left, ~player2_right, 1'b1, 1'b1, ~player2_fire, ~player2_magic};
assign player3_controls = {~player3_up, ~player3_down, ~player3_left, ~player3_right, 1'b1, 1'b1, ~player3_fire, ~player3_magic};
assign player4_controls = {~player4_up, ~player4_down, ~player4_left, ~player4_right, 1'b1, 1'b1, ~player4_fire, ~player4_magic};

// Vindicators Part II Tank Controls (Player 1 and 2)
wire tank1_back_r;
wire tank1_back_l;
wire tank1_forward_r;
wire tank1_forward_l;
wire tank1_thumb_r;
wire tank1_thumb_l;
wire tank1_trig_r;
wire tank1_trig_l;

wire tank2_back_r;
wire tank2_back_l;
wire tank2_forward_r;
wire tank2_forward_l;
wire tank2_thumb_r;
wire tank2_thumb_l;
wire tank2_trig_r;
wire tank2_trig_l;

assign tank1_back_r    = joy[5]; // B
assign tank1_back_l    = joy[1]; // down
assign tank1_forward_r = joy[6]; // X
assign tank1_forward_l = joy[0]; // up
assign tank1_thumb_r   = joy[7]; // Y
assign tank1_thumb_l   = joy[3]; // left
assign tank1_trig_r    = joy[9]; // R trig
assign tank1_trig_l    = joy[8]; // L trig

assign tank2_back_r    = joy2[5]; // B
assign tank2_back_l    = joy2[1]; // down
assign tank2_forward_r = joy2[6]; // X
assign tank2_forward_l = joy2[0]; // up
assign tank2_thumb_r   = joy2[7]; // Y
assign tank2_thumb_l   = joy2[3]; // left
assign tank2_trig_r    = joy2[9]; // R trig
assign tank2_trig_l    = joy2[8]; // L trig

wire [7:0] tank1_controls;
wire [7:0] tank2_controls;

reg analog_stick1_detected = 1'b0;
reg analog_tank1_back_r;
reg analog_tank1_back_l;
reg analog_tank1_forward_r;
reg analog_tank1_forward_l;
reg analog_tank1_thumb_r;
reg analog_tank1_thumb_l;
reg analog_tank1_trig_r;
reg analog_tank1_trig_l;

reg analog_stick2_detected = 1'b0;
reg analog_tank2_back_r;
reg analog_tank2_back_l;
reg analog_tank2_forward_r;
reg analog_tank2_forward_l;
reg analog_tank2_thumb_r;
reg analog_tank2_thumb_l;
reg analog_tank2_trig_r;
reg analog_tank2_trig_l;

always @(posedge clk_14) begin
   analog_stick1_detected <= cont1_joy[15:0] != 0;
   analog_stick2_detected <= cont2_joy[15:0] != 0;
	
	if (game_id == GAME_ID_VINDICATORS_PT2) begin
        analog_tank1_back_r <= analog_stick1_detected ? cont1_joy[31:24] > 192 : 1'b0;
        analog_tank1_back_l <= analog_stick1_detected ? cont1_joy[15:8] > 192 : 1'b0;
        analog_tank1_forward_r <= analog_stick1_detected ? cont1_joy[31:24] < 64 : 1'b0;
        analog_tank1_forward_l <= analog_stick1_detected ? cont1_joy[15:8] < 64 : 1'b0;
        analog_tank1_thumb_r <= analog_stick1_detected ? cont1_key[9] : 1'b0;
        analog_tank1_thumb_l <= analog_stick1_detected ? cont1_key[8] : 1'b0;
        analog_tank1_trig_r <= analog_stick1_detected ? cont1_key[11] : 1'b0;
        analog_tank1_trig_l <= analog_stick1_detected ? cont1_key[10] : 1'b0;

        analog_tank2_back_r <= analog_stick2_detected ? cont2_joy[31:24] > 192 : 1'b0;
        analog_tank2_back_l <= analog_stick2_detected ? cont2_joy[15:8] > 192 : 1'b0;
        analog_tank2_forward_r <= analog_stick2_detected ? cont2_joy[31:24] < 64 : 1'b0;
        analog_tank2_forward_l <= analog_stick2_detected ? cont2_joy[15:8] < 64 : 1'b0;
        analog_tank2_thumb_r <= analog_stick2_detected ? cont2_key[9] : 1'b0;
        analog_tank2_thumb_l <= analog_stick2_detected ? cont2_key[8] : 1'b0;
        analog_tank2_trig_r <= analog_stick2_detected ? cont2_key[11] : 1'b0;
        analog_tank2_trig_l <= analog_stick2_detected ? cont2_key[10] : 1'b0;
    end
end

assign tank1_controls = { 
    ~(analog_stick1_detected ? analog_tank1_back_r : tank1_back_r), 
    ~(analog_stick1_detected ? analog_tank1_back_l : tank1_back_l), 
    ~(analog_stick1_detected ? analog_tank1_forward_r : tank1_forward_r), 
    ~(analog_stick1_detected ? analog_tank1_forward_l : tank1_forward_l), 
    ~(analog_stick1_detected ? analog_tank1_thumb_r : tank1_thumb_r),  
    ~(analog_stick1_detected ? analog_tank1_thumb_l : tank1_thumb_l), 
    ~(analog_stick1_detected ? analog_tank1_trig_r : tank1_trig_r), 
    ~(analog_stick1_detected ? analog_tank1_trig_l : tank1_trig_l)};
    
assign tank2_controls = { 
    ~(analog_stick2_detected ? analog_tank2_back_r : tank2_back_r), 
    ~(analog_stick2_detected ? analog_tank2_back_l : tank2_back_l ), 
    ~(analog_stick2_detected ? analog_tank2_forward_r : tank2_forward_r), 
    ~(analog_stick2_detected ? analog_tank2_forward_l : tank2_forward_l), 
    ~(analog_stick2_detected ? analog_tank2_thumb_r : tank2_thumb_r),  
    ~(analog_stick2_detected ? analog_tank2_thumb_l : tank2_thumb_l), 
    ~(analog_stick2_detected ? analog_tank2_trig_r : tank2_trig_r) , 
    ~(analog_stick2_detected ? analog_tank2_trig_l : tank2_trig_l)};
        
// Control multiplexing -- allows for dynamic mapping of controllers for 4P Guantlet 1,
// static controller mapping for 2P Gauntlet 1 and All Gauntlet 2 variants, and tank 
// controls for Vindicators Part II.

wire [7:0] player1_muxed_controls;
wire [7:0] player2_muxed_controls;
wire [7:0] player3_muxed_controls;
wire [7:0] player4_muxed_controls;

assign player1_muxed_controls = 
    game_id == GAME_ID_GAUNTLET_4P ? 
        ( p1_mux == MUX_ID_WARRIOR ? player1_controls :
          p2_mux == MUX_ID_WARRIOR ? player2_controls :
          p3_mux == MUX_ID_WARRIOR ? player3_controls :
          p4_mux == MUX_ID_WARRIOR ? player4_controls : 
			 8'b11111111) :
    game_id == GAME_ID_VINDICATORS_PT2 ? tank1_controls :
		player1_controls;

assign player2_muxed_controls = 
    game_id == GAME_ID_GAUNTLET_4P ? 
        ( p1_mux == MUX_ID_VALKYRIE ? player1_controls :
          p2_mux == MUX_ID_VALKYRIE ? player2_controls :
          p3_mux == MUX_ID_VALKYRIE ? player3_controls :
          p4_mux == MUX_ID_VALKYRIE ? player4_controls : 
			 8'b11111111) :
    game_id == GAME_ID_VINDICATORS_PT2 ? tank2_controls :
          player2_controls;

assign player3_muxed_controls = 
    game_id == GAME_ID_GAUNTLET_4P ? 
        ( p1_mux == MUX_ID_WIZARD ? player1_controls :
          p2_mux == MUX_ID_WIZARD ? player2_controls :
          p3_mux == MUX_ID_WIZARD ? player3_controls :
          p4_mux == MUX_ID_WIZARD ? player4_controls : 
			 8'b11111111) :
	 game_id == GAME_ID_VINDICATORS_PT2 ? { 6'b111111, ~joy2[15], ~joy[15] } : // start buttons.
          player3_controls;

assign player4_muxed_controls = 
    game_id == GAME_ID_GAUNTLET_4P ? 
        ( p1_mux == MUX_ID_ELF ? player1_controls :
          p2_mux == MUX_ID_ELF ? player2_controls :
          p3_mux == MUX_ID_ELF ? player3_controls :
          p4_mux == MUX_ID_ELF ? player4_controls : 
			 8'b11111111) :
          player4_controls;

wire coin1 = joy[14];
wire coin2 = joy2[14];
wire coin3 = joy3[14];
wire coin4 = joy4[14];
wire coin1_muxed;
wire coin2_muxed;
wire coin3_muxed;
wire coin4_muxed;

assign coin1_muxed =
    game_id == GAME_ID_GAUNTLET_4P ?
        ( p1_mux == MUX_ID_WARRIOR ? coin1 :
          p2_mux == MUX_ID_WARRIOR ? coin2 :
          p3_mux == MUX_ID_WARRIOR ? coin3 :
          p4_mux == MUX_ID_WARRIOR ? coin4 :
			 1'b0
        ) : coin1 ;

assign coin2_muxed =
    game_id == GAME_ID_GAUNTLET_4P ?
        ( p1_mux == MUX_ID_VALKYRIE ? coin1 :
          p2_mux == MUX_ID_VALKYRIE ? coin2 :
          p3_mux == MUX_ID_VALKYRIE ? coin3 :
          p4_mux == MUX_ID_VALKYRIE ? coin4 :
			 1'b0
        ) : coin2 ;

assign coin3_muxed =
    game_id == GAME_ID_GAUNTLET_4P ?
        ( p1_mux == MUX_ID_WIZARD ? coin1 :
          p2_mux == MUX_ID_WIZARD ? coin2 :
          p3_mux == MUX_ID_WIZARD ? coin3 :
          p4_mux == MUX_ID_WIZARD ? coin4 :
			 1'b0
        ) : coin3 ;

assign coin4_muxed =
    game_id == GAME_ID_GAUNTLET_4P ?
        ( p1_mux == MUX_ID_ELF ? coin1 :
          p2_mux == MUX_ID_ELF ? coin2 :
          p3_mux == MUX_ID_ELF ? coin3 :
          p4_mux == MUX_ID_ELF ? coin4 :
			 1'b0
        ) : coin4 ;

wire [4:0] misc_inputs;
assign misc_inputs = {~service_mode, ~coin1_muxed, ~coin2_muxed, ~coin3_muxed, ~coin4_muxed};

//---------------------------------------------
FPGA_GAUNTLET fpga_gauntlet (

		.I_CLK_14M(clk_14),  //			: in	std_logic;
		.I_CLK_7M(clk_core_7), 	//			: in	std_logic;

		// -- Active high reset
		.I_RESET(~reset_n || manual_reset), 	//			: in	std_logic;

		// -- Player controls, active low
		.I_P1(player1_muxed_controls),			//		: in	std_logic_vector(7 downto 0);
		.I_P2(player2_muxed_controls),			//		: in	std_logic_vector(7 downto 0);
		.I_P3(player3_muxed_controls),			//		: in	std_logic_vector(7 downto 0);
		.I_P4(player4_muxed_controls),			//		: in	std_logic_vector(7 downto 0);

		// -- System inputs
		.I_SYS(misc_inputs),
		.I_SLAP_TYPE(
			game_id == GAME_ID_GAUNTLET_4P 		 ? 104 :
			game_id == GAME_ID_GAUNTLET_2P 		 ? 107 :
			game_id == GAME_ID_GAUNTLET2	 	 ? 106 :
			game_id == GAME_ID_VINDICATORS_PT2	 ? 118 :
			104
		),	

		//.O_LEDS(),			//	: out	std_logic_vector(4 downto 1);

		// -- Audio out
		.O_AUDIO_L(audio_l),		//	: out	std_logic_vector(15 downto 0) := (others=>'0');
		.O_AUDIO_R(audio_r),		//	: out	std_logic_vector(15 downto 0) := (others=>'0');

		// -- Monitor output
		.O_VIDEO_I(i),		//	: out	std_logic_vector(3 downto 0);
		.O_VIDEO_R(r),		//	: out	std_logic_vector(3 downto 0);
		.O_VIDEO_G(g),		//	: out	std_logic_vector(3 downto 0);
		.O_VIDEO_B(b),		//	: out	std_logic_vector(3 downto 0);
		
		.O_HSYNC(hs_core),			//	: out	std_logic;
		.O_VSYNC(vs_core),			//	: out	std_logic;
		
		// .O_CSYNC(),					//	: out	std_logic;
		
		.O_HBLANK(hblank_core),		//	: out	std_logic;
		.O_VBLANK(vblank_core),		//	: out	std_logic;

		// -- GFX ROMs, read from non existent ROMs MUST return FFFFFFFF
		.O_GP_EN(gp_en),
		.O_GP_ADDR(gp_addr),
		.I_GP_DATA(gp_data),

		.O_CP_ADDR(cp_addr),
		.I_CP_DATA(cp_data),

		.O_MP_EN(mp_en),
		.O_MP_ADDR(mp_addr),
		.I_MP_DATA(mp_data),

		.O_4R_ADDR(r4_addr),
		.I_4R_DATA(r4_data),

		.O_AP_EN(ap_en),
		.O_AP_ADDR(ap_addr),
		.I_AP_DATA(ap_data)
);



///////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////

wire    clk_core_7;
wire    clk_core_7_90deg;
wire    clk_sys;  
wire 	  clk_93;	
wire    clk_14;
wire    pll_core_locked;
assign clk_sys = clk_93;

mf_pllbase mp1 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),

	 
	 .outclk_4		 ( dram_clk ),
    .outclk_3	    ( clk_93 ), // 180 degrees
    .outclk_2       ( clk_core_7_90deg ),
	 .outclk_1       ( clk_core_7 ),
    .outclk_0       ( clk_14 ),

    .locked         ( pll_core_locked )
);

endmodule
