//
// RC2014, ACIA, 
// CLK_CPU 7.37280000 MHZ 
// CLK_VDP 21.477MHZ
//

`default_nettype none

module rc2014
(
	input         CLOCK_27,
`ifdef USE_CLOCK_50
   input         CLOCK_50,
`endif
	output        LED,
	output [VGA_BITS-1:0] VGA_R,
	output [VGA_BITS-1:0] VGA_G,
	output [VGA_BITS-1:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,

`ifdef USE_HDMI
	output        HDMI_RST,
	output  [7:0] HDMI_R,
	output  [7:0] HDMI_G,
	output  [7:0] HDMI_B,
	output        HDMI_HS,
	output        HDMI_VS,
	output        HDMI_PCLK,
	output        HDMI_DE,
	inout         HDMI_SDA,
	inout         HDMI_SCL,
	input         HDMI_INT,
`endif

	input         SPI_SCK,
	inout         SPI_DO,
	input         SPI_DI,
	input         SPI_SS2,    // data_io
	input         SPI_SS3,    // OSD
	input         CONF_DATA0, // SPI_SS for user_io

`ifdef USE_QSPI
	input         QSCK,
	input         QCSn,
	inout   [3:0] QDAT,
`endif
`ifndef NO_DIRECT_UPLOAD
	input         SPI_SS4,
`endif

	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,

`ifdef DUAL_SDRAM
	output [12:0] SDRAM2_A,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_DQML,
	output        SDRAM2_DQMH,
	output        SDRAM2_nWE,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nCS,
	output  [1:0] SDRAM2_BA,
	output        SDRAM2_CLK,
	output        SDRAM2_CKE,
`endif

	output        AUDIO_L,
	output        AUDIO_R,
`ifdef I2S_AUDIO
	output        I2S_BCK,
	output        I2S_LRCK,
	output        I2S_DATA,
`endif
`ifdef SPDIF_AUDIO
	output        SPDIF,
`endif
`ifdef USE_AUDIO_IN
	input         AUDIO_IN,
`endif
	
`ifdef USE_EXTBUS	
	output [23:0]  BUS_A = 24'b0,
	inout  [15:0]  BUS_D = 16'b0,
	inout          USER1,
	inout          USER2,
	inout          USER3,
	inout          USER5,
	inout          USER6,
	inout          USER7,
	inout          N41,
	inout          N42,
	inout          N43,
	inout          N44,
	inout          N45,
	inout          N46,
	inout          N47,
	inout          N48,
	output         BUS_nRESET,
	output         BUS_nM1,
	output         BUS_nMREQ,
	output         BUS_nIORQ,
	output         BUS_nRD,
	output         BUS_nWR,
	output         BUS_nRFSH,
	output         BUS_nHALT,
	output         BUS_nBUSAK,
	output reg     BUS_CLK,
`endif
   input         UART_RX,
	output        UART_TX
);

`ifdef NO_DIRECT_UPLOAD
localparam bit DIRECT_UPLOAD = 0;
wire SPI_SS4 = 1;
`else
localparam bit DIRECT_UPLOAD = 1;
`endif

`ifdef USE_QSPI
localparam bit QSPI = 1;
assign QDAT = 4'hZ;
`else
localparam bit QSPI = 0;
`endif

`ifdef VGA_8BIT
localparam VGA_BITS = 8;
`else
localparam VGA_BITS = 6;
`endif

`ifdef USE_HDMI
localparam bit HDMI = 1;
assign HDMI_RST = 1'b1;
`else
localparam bit HDMI = 0;
`endif

`ifdef BIG_OSD
localparam bit BIG_OSD = 1;
`define SEP "-;",
`else
localparam bit BIG_OSD = 0;
`define SEP
`endif

// remove this if the 2nd chip is actually used
`ifdef DUAL_SDRAM
assign SDRAM2_A = 13'hZZZZ;
assign SDRAM2_BA = 0;
assign SDRAM2_DQML = 0;
assign SDRAM2_DQMH = 0;
assign SDRAM2_CKE = 0;
assign SDRAM2_CLK = 0;
assign SDRAM2_nCS = 1;
assign SDRAM2_DQ = 16'hZZZZ;
assign SDRAM2_nCAS = 1;
assign SDRAM2_nRAS = 1;
assign SDRAM2_nWE = 1;
`endif

`include "build_id.v"

`default_nettype none

`ifdef USE_EXTBUS	
assign {N41,N42,N43,N44,N45,N46,N47,N48,USER1,USER2,USER3,USER5,USER6,USER7}=0;
`endif

`include "build_id.v"
localparam CONF_STR = {
	"RC2014;;",
	"F0,ROM,Load ROM;",
	"S0U,IMGVHD,Load Disk;",
	`SEP
	"P1,Boot Switches;",
	"P1O13,Slice,0,1,2,3,4,5,6,7;",
	"P1O4,Device,Hard,Floppy;",
	"P1O5,Unit,ROM,Disk;",
	"P1O6,Boot,Menu,Auto;",
	"P1O7,Console,Primary,Secondary;",
	"P1O8,Device,Serial,CRT;",
	`SEP
	"P1T0,Reset;",
	"P2,Hardware options;",	
	"P2O9,AY-3-8910,External,Internal;",
	"P2OA,2nd ACIA,External,Internal;",
	"P2OB,Z80 CTC,External,Internal;",	
	`SEP
	"P2T0,Reset;",
	`SEP
	"T0,Reset;",
	"V,v1.0.",`BUILD_DATE
};

////////////////////   CLOCKS   ///////////////////
wire clk_sys;
wire clk_vt;
wire clk_7;
wire clk_25,clk_50,clk_100;
wire locked;



pll pll
(
`ifdef USE_CLOCK_50
   .inclk0(CLOCK_50),
`else
   .inclk0(CLOCK_27),
`endif	
	.c0(clk_sys),
	.c1(),
	.c2(clk_7),
	.locked(locked)
);
pll1 pll1
(
`ifdef USE_CLOCK_50
   .inclk0(CLOCK_50),
`else
   .inclk0(CLOCK_27),
`endif
	.c0(clk_vt),
	.c1(clk_50),
	.c2(clk_100),
	.c3(clk_25)
);

reg ne7M3, pe7M3;
reg ne1M8, pe1M8;

reg[6:0] ce = 1;

always @(negedge clk_sys) if(!reset) begin
	ce <= ce+1'd1;
	ne7M3 <= ~ce[0] & ~ce[1] & ~ce[2];
	pe7M3 <= ~ce[0] & ~ce[1] &  ce[2];
	ne1M8 <= ~ce[0] & ~ce[1] & ~ce[2] & ~ce[3] & ~ce[4] ;
	pe1M8 <= ~ce[0] & ~ce[1] & ~ce[2] & ~ce[3] &  ce[4];
end


//////////////////   MIST ARM I/O   ///////////////////
wire  [7:0] joystick_0;
wire  [7:0] joystick_1;

wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire        ypbpr;
wire        no_csync;
wire [63:0] status;

wire [31:0] sd_lba;
wire  sd_rd;
wire  sd_wr;

wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        img_mounted;
wire [63:0] img_size;

wire        sd_ack_conf;
wire        sd_conf;
wire        sd_sdhc;

wire        key_strobe;
wire        key_pressed;
wire        key_extended;
wire  [7:0] key_code;

wire  [8:0] mouse_x;
wire  [8:0] mouse_y;
wire  [7:0] mouse_flags;
wire        mouse_strobe;


user_io #(.STRLEN($size(CONF_STR)>>3), .PS2DIV(2000), .SD_IMAGES(1), .FEATURES(32'h0 | (BIG_OSD << 13) | (HDMI << 14))) user_io
(
	.clk_sys(clk_sys),
	.clk_sd(clk_sys),
	.conf_str(CONF_STR),

	.SPI_CLK(SPI_SCK),
	.SPI_SS_IO(CONF_DATA0),
	.SPI_MOSI(SPI_DI),
	.SPI_MISO(SPI_DO),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.sd_conf(sd_conf),
	.sd_ack_conf(sd_ack_conf),
	.sd_sdhc(sd_sdhc),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_din(sd_buff_din),
	.sd_dout(sd_buff_dout),
	.sd_dout_strobe(sd_buff_wr),

	.key_strobe(key_strobe),
	.key_code(key_code),
	.key_pressed(key_pressed),
	.key_extended(key_extended),
	.ps2_kbd_clk(ps2k_c),
	.ps2_kbd_data(ps2k_d),
	.mouse_x(mouse_x),
	.mouse_y(mouse_y),
	.mouse_flags(mouse_flags),
	.mouse_strobe(mouse_strobe),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),


	.buttons(buttons),
	.status(status),
	.scandoubler_disable(scandoubler_disable),
	.ypbpr(ypbpr),
	.no_csync(no_csync)

);

wire reset=status[0] | buttons[1] | ioctl_download |~locked ;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [5:0] ioctl_index;
wire  [1:0] ioctl_ext_index;

data_io data_io
(
	.clk_sys(clk_7),

	.SPI_SCK(SPI_SCK),
	.SPI_SS2(SPI_SS2),
	.SPI_DI(SPI_DI),
	.SPI_DO(SPI_DO),

	.clkref_n(1'b0),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index({ioctl_ext_index, ioctl_index})
);

wire sd_miso,sd_mosi,sd_clk,sd_cs;

sd_card sd_card (
	.clk_sys         ( clk_sys       ),   // at least 2xsd_sck
	// connection to io controller
	.sd_lba          ( sd_lba         ),
	.sd_rd           ( sd_rd          ),
	.sd_wr           ( sd_wr          ),
	.sd_ack          ( sd_ack         ),
	.sd_conf         ( sd_conf        ),
	.sd_ack_conf     ( sd_ack_conf    ),
	.sd_sdhc         ( sd_sdhc        ),
	.allow_sdhc      (1'b1            ),
	.sd_buff_dout    ( sd_buff_dout   ),
	.sd_buff_wr      ( sd_buff_wr     ),
	.sd_buff_din     ( sd_buff_din    ),
	.sd_buff_addr    ( sd_buff_addr   ),

   .img_mounted   (img_mounted),
	.img_size      (img_size),
	// connection to local CPU
	.sd_cs   		( sd_cs ),
	.sd_sck  		( sd_clk          ),
	.sd_sdi  		( sd_mosi         ),
	.sd_sdo  		( sd_miso        )
);
////////////////////   VT ////////////////////////
wire vga_fb,vga_ht;
wire txtx,rxrx;
wire rtsrts,ctscts;
wire ps2k_c,ps2k_d;

wire vt_hsync;
wire vt_vsync;

   vt10x vt10x (
      .vga_hsync  (vt_hsync),
      .vga_vsync  (vt_vsync),
      .vga_fb     (vga_fb),
      .vga_ht     (vga_ht),

      .rx (txtx),
      .tx (rxrx),
		.cts (~rtsrts),
		.rts (ctscts),
		.rtscts (1'b0),

      .ps2k_c (is_crt? 8'b1:ps2k_c),
      .ps2k_d (is_crt? 8'b1:ps2k_d),
      .teste (1'b0),
      .testf (1'b0),
      .vga_cursor_block (1'b1),
      .vga_cursor_blink (1'b1),
      .have_act_seconds(0),
      .vga_bl (),

      .vttype (105),
		.bps(115200),

      .cpuclk (clk_vt),
	   .clk50mhz (clk_50), // BRG clock
	   .clk25mhz (clk_25), // VGA clock
      .reset (reset)
   );


////////////////////   COMPUTER   ///////////////////

//////////// Internal devices //////////////
wire psgInt        = status[9];
wire aciaInt       = status[10];
wire ctcInt        = status[11];

`ifndef USE_EXTBUS
wire [7:0] BUS_D   = 8'hff;
`endif
////////////////////////////////////////////
wire nM1,nMREQ,nIORQ,nRD,nWR,nRFSH,nBUSACK;
wire [15:0] cpu_a;
wire [7:0]  cpu_din;
wire [7:0]  cpu_dout;

//////////////////////
wand nINT;
assign nINT=1'b1;
assign nINT= uart1_nINT;
assign nINT= aciaInt? UART2_CS ? uart2_nINT : 1'b1 :1'b1;
assign nINT= ctcInt?  ctc_nINT: 1'b1;
assign nINT= vdp_nINT;

//////////////////////
wand nBUSRQ=1'b1;
//////////////////////
wand nNMI=1'b1;
//////////////////////
wand nWAIT=1'b1;
//////////////////////
wand nHALT;

T80pa cpu
(
	.RESET_n(~reset),

	.CLK(clk_sys),
	.CEN_n(ne7M3),
	.CEN_p(pe7M3),
	
	.WAIT_n(nWAIT),
	.INT_n(nINT),
	.NMI_n(nNMI),
	.BUSRQ_n(nBUSRQ),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(nHALT),
	.BUSAK_n(nBUSACK),
	.OUT0   (1'b0   ),
	.A(cpu_a),
	.DO(cpu_dout),
	.DI(cpu_din)
);

wire UART1_CS = cpu_a[7:1] == 7'b1000000 && !nIORQ && nM1;    //
wire UART2_CS = cpu_a[7:1] == 7'b0100000 && !nIORQ && nM1;    //
wire LED_CS   = cpu_a[7:0] == 8'h00      && !nIORQ && nM1;    //
wire CTC_CS   = cpu_a[7:2] == 6'b100010  && !nIORQ && nM1;    // $88-8B
wire PSG_CS   = cpu_a[7:2] == 6'b101000  && !nIORQ && nM1;    // $A0-A1
wire SD_CS    = cpu_a[7:0] == 8'h69      && !nIORQ && nM1;    // $69
wire TMS_CS   = cpu_a[7:2] == 6'b100110  && !nIORQ && nM1;    // 
wire PPI_CS   = cpu_a[7:2] == 6'b101010  && !nIORQ && nM1;    // $A8-AB

wire MEM_nRD = nRD | nMREQ;
wire MEM_nWR = nWR | nMREQ;
wire IO_nRD  = nRD | nIORQ;
wire IO_nWR  = nWR | nIORQ;

assign mem_rd = !MEM_nRD;
assign mem_we = !MEM_nWR ;


assign cpu_din = LED_CS   ? status[8:1] :
                 TMS_CS   ? vdp_dout   :
					  PPI_CS   ? ppi_dout   :
                 UART1_CS ? uart1_dout :
  					  SD_CS    ? {sd_miso,7'b0}:
					  mem_rd   ? mem_q:
				     UART2_CS ? aciaInt?uart2_dout : BUS_D:
					  CTC_CS   ? ctcInt? ctc_dout   : BUS_D: 
					  PSG_CS   ? psgInt? psg_dout   : BUS_D:
					  BUS_D;

assign mem_d = mem_we ? cpu_dout : 8'bZZZZZZZZ;

wire [7:0] uart1_dout;
wire [7:0] uart2_dout;

wire uart1_nINT,uart2_nINT;

acia6850 uart1
(
			.clk      (clk_sys),        // System Clock
			.rst      (reset),          // Reset input (active high)
			.cs       (UART1_CS) ,      // miniUART Chip Select
			.addr     (cpu_a[0]),       // Register Select
			.rw       (nWR),            // Read / Not Write  1 - Read, 0 - Write
			.data_in  (cpu_dout),       // Data Bus In 
			.data_out (uart1_dout),        // Data Bus Out
			.irq      (uart1_nINT),     // Interrupt Request out.

			.RxC      (ne7M3),          // Receive Baud Clock
			.TxC      (ne7M3),          // Transmit Baud Clock
			.RxD      (rxrx),           // Receive Data
			.TxD      (txtx),           // Transmit Data
			.DCD_n    (1'b0),           // Data Carrier Detect
			.CTS_n    (1'b0),           // Clear To Send
			.RTS_n    (rtsrts)          // Request To send
		);

acia6850 uart2
(
			.clk      (clk_sys),        // System Clock
			.rst      (reset),          // Reset input (active high)
			.cs       (UART2_CS) ,       // miniUART Chip Select
			.addr     (cpu_a[0]),       // Register Select
			.rw       (nWR),            // Read / Not Write  1 - Read, 0 - Write
			.data_in  (cpu_dout),       // Data Bus In 
			.data_out (uart2_dout),         // Data Bus Out
			.irq      (uart2_nINT),      // Interrupt Request out.

			.RxC      (ne7M3),          // Receive Baud Clock
			.TxC      (ne7M3),          // Transmit Baud Clock
			.RxD      (UART_RX ),           // Receive Data
			.TxD      (UART_TX ),           // Transmit Data
			.DCD_n    (1'b0),           // Data Carrier Detect
			.CTS_n    (1'b0),           // Clear To Send
			.RTS_n    ()                // Request To send
		);



wire [7:0] ctc_dout;
wire ctc_counter_0_to;
wire ctc_counter_1_to;
wire ctc_counter_2_to;
wire ctc_nINT;

		
z80ctc_top z80ctc
(
	.clock(clk_sys),
	.clock_ena(pe7M3),
	.reset(reset),
	.din(cpu_dout),
	.cpu_din(cpu_din),
	.dout(ctc_dout),
	.ce_n(~CTC_CS),
	.cs(cpu_a[1:0]),
	.m1_n(nM1),
	.iorq_n(nIORQ),
	.rd_n(nRD),
	.int_n(ctc_nINT),
	.trg0(pe7M3),
	.to0(ctc_counter_0_to),
	.trg1(pe7M3),
	.to1(ctc_counter_1_to),
	.trg2(ne7M3),
	.to2(ctc_counter_2_to),
	.trg3(ctc_counter_2_to)
);

wire [7:0] psg_dout;
wire [14:0] mix;
wire [7:0] io;
wire bc1   = psgInt? PSG_CS  && !cpu_a[0] : 1'bZ;
wire bdir  = psgInt? PSG_CS  && !nWR      : 1'bZ;

wire [15:0] audio = {mix,1'b0};

psg ay8910
(
	.clock  (clk_sys),
	.sel    (1'b1   ),
	.ce     (pe1M8  ),
	.reset  (~reset ),
	.bdir   (bdir   ),
	.bc1    (bc1    ),
	.d      (psgInt ? cpu_dout: 8'bZZZZZZZZ),
	.q      (psg_dout),
	.mix    (mix    ),
	.ioad   (io     )
);
///////////////////TMS9958/////////////////////////////


wire        vram_we,vram_rd;
wire 			vram_we_h,vram_we_l;

wire [16:0] vram_a;
wire [7:0]  vram_q_h,vram_q_l;
wire [7:0]  vram_d;

wire [7:0] vdp_dout;
wire [5:0] vdp_r;
wire [5:0] vdp_g;
wire [5:0] vdp_b;
wire       vdp_hsync,vdp_vsync;
wire       vdp_hblank,vdp_vblank;

wire csw_n_i = !TMS_CS | nWR;
wire csr_n_i = !TMS_CS | nRD;
wire vdp_nINT;

f18a_core VDP (
    .clk_100m0_i(clk_100),
    .clk_25m0_i(clk_25),
    .reset_n_i(~reset),
    .mode_i(cpu_a[0]),
    .csw_n_i(csw_n_i),
    .csr_n_i(csr_n_i),
    .int_n_o(vdp_nINT),
    .cd_i(cpu_dout),
    .cd_o(vdp_dout),
    .blank_o(),
    .hsync_o(vdp_hsync),
    .vsync_o(vdp_vsync),
    .red_o(vdp_r),
    .grn_o(vdp_g),
    .blu_o(vdp_b),
    .sprite_max_i(1'b0),
    .scanlines_i(1'b0)
);

///////////////////TMS9958/////////////////////////////
// Instanciado del componente jt8255

wire [7:0] ppi_dout;
wire [7:0] porta_din,porta_dout;
wire [7:0] portb_din,portb_dout;
wire [7:0] portc_din,portc_dout;

    jt8255 jt8255_inst (
        .rst(reset),
        .clk(clk_sys),
        .addr(cpu_a[1:0]),
        .din(cpu_dout),
        .dout(ppi_dout),
        .rdn(nRD),
        .wrn(nWR),
        .csn(~PPI_CS),
        .porta_din(8'h0),
        .portb_din(kbd_dout),
        .portc_din(8'h0),
        .porta_dout(porta_dout),
        .portb_dout(),
        .portc_dout(portc_dout)
    );

wire [7:0] kbd_dout;

keyboard keyboard
(
   .clk		  (clk_sys),
	.reset	  (reset),
   .key_strobe(key_strobe),
   .key_pressed(key_pressed),
   .key_extended(key_extended),
   .key_code  (key_code),
   .kb_row    (portc_dout[3:0]),
   .kb_data   (kbd_dout)
);

////////////////////   RAM/ROM   ///////////////////

wire[19:0] mem_a;
wire [7:0] mem_q;
wire [7:0] mem_d;
wire mem_we;
wire mem_rd;

wire isram = mem_a[19];
wire mem_ready;

wire [7:0] PAGSEL0,PAGSEL1,PAGSEL2,PAGSEL3;
wire nPage;

always @(posedge clk_sys) begin
	if(reset) begin
		PAGSEL0 <= 8'h00;
		PAGSEL1 <= 8'h01;
		PAGSEL2 <= 8'h3e;
		PAGSEL3 <= 8'h3f;
		nPage   <= 1'b1;
		DEBUG   <= 8'b0;
	end else begin
	 if (!IO_nWR)
		case (cpu_a[7:0])
			8'h00: DEBUG <= cpu_dout;
			8'h69: begin
			          sd_mosi <=cpu_dout[0];
						 sd_clk  <=cpu_dout[4];
						 sd_cs   <=cpu_dout[3];
					 end
			8'h78: PAGSEL0 <= cpu_dout;
			8'h79: PAGSEL1 <= cpu_dout;
			8'h7a: PAGSEL2 <= cpu_dout;
			8'h7b: PAGSEL3 <= cpu_dout;
			8'h7c: nPage <= cpu_dout[0];
		endcase
	end
end

always @(posedge clk_sys) begin
  case (cpu_a[15:14])
    2'b00 : mem_a <= nPage?{PAGSEL0[5:0],cpu_a[13:0]} : {6'b0,cpu_a[13:0]};
    2'b01 : mem_a <= nPage?{PAGSEL1[5:0],cpu_a[13:0]} : {6'b0,cpu_a[13:0]};
    2'b10 : mem_a <= nPage?{PAGSEL2[5:0],cpu_a[13:0]} : {6'b0,cpu_a[13:0]};
    2'b11 : mem_a <= nPage?{PAGSEL3[5:0],cpu_a[13:0]} : {6'b0,cpu_a[13:0]};
  endcase
end

// RAM & ROM

sdram sdram
(
	.*,
	.init(~locked),
	.clk(clk_sys),

   .wtbt(0),
   .addr(ioctl_download?ioctl_addr:mem_a),
   .rd(mem_rd),
   .dout(mem_q),
   .din(ioctl_download?ioctl_dout:mem_d),
   .we(ioctl_download?ioctl_wr:isram?mem_we:1'b0),
   .ready(mem_ready)
);

assign SDRAM_CLK = ~clk_sys; 

////////////////////   AUDIO   ///////////////////
wire [15:0] audiomix;

`ifdef I2S_AUDIO
i2s i2s (
	.reset(1'b0),
	.clk(clk_sys),
	.clk_rate(32'd58_982_400),

	.sclk(I2S_BCK),
	.lrclk(I2S_LRCK),
	.sdata(I2S_DATA),

	.left_chan ({~audio[15],audio[14:0]}),
	.right_chan({~audio[15],audio[14:0]})
);
`endif

dac #(
   .c_bits      (16))
audiodac_l(
   .clk_i       (clk_sys ),
   .res_n_i     (1      ),
   .dac_i       (audio),
   .dac_o       (AUDIO_L)
  );

dac #(
   .c_bits      (16))
audiodac_r(
   .clk_i       (clk_sys ),
   .res_n_i     (1      ),
   .dac_i       (audio),
   .dac_o       (AUDIO_R)
  );

////////////////////   VIDEO   ///////////////////

wire HSync,VSync;
wire HBlank,VBlank;
wire [5:0] Rx,Gx,Bx;
wire is_crt = status[8];


always @(posedge clk_sys) begin
	Rx<= is_crt? vdp_r : 4'b0;
	Gx<= is_crt? vdp_g : vga_fb ? 4'b1111 : vga_ht? 4'b1000: 4'b0000;
	Bx<= is_crt? vdp_b : 4'b0;
	HSync <= is_crt? vdp_hsync : vt_hsync;
	VSync <= is_crt? vdp_vsync : vt_vsync;
end

mist_video #(.COLOR_DEPTH(4), .OUT_COLOR_DEPTH(VGA_BITS),.BIG_OSD(BIG_OSD)) mist_video (
	
   //.clk_sys ( is_crt ? clk_25 : clk_50),
	.clk_sys(clk_25),
	// OSD SPI interface
	.SPI_SCK     ( SPI_SCK    ),
	.SPI_SS3     ( SPI_SS3    ),
	.SPI_DI      ( SPI_DI     ),

	.scanlines   ( status[16:15] ),

	.ce_divider  ( 3'd1       ),

	.scandoubler_disable ( 1'b1 ),
	.no_csync    ( 1'b1       ),
	.ypbpr       ( ypbpr      ),
	.rotate      ( 2'b00      ),
	.blend       ( 1'b0       ),

	// video in
	.R           ( Rx ),
	.G           ( Gx ),
	.B           ( Bx ),
 
   .HSync       (HSync),
	.VSync       (VSync),

	// MiST video output signals
	.VGA_R       ( VGA_R      ),
	.VGA_G       ( VGA_G      ),
	.VGA_B       ( VGA_B      ),
	.VGA_VS      ( VGA_VS     ),
	.VGA_HS      ( VGA_HS     )
);

// External BUS

wire [7:0] DEBUG;

assign LED= !key_strobe;
`ifdef USE_EXTBUS
always @(posedge clk_sys) begin
	if(reset) begin
		BUS_A<=16'hffff;
		BUS_D<=8'hff;
		BUS_nMREQ <=1;
      BUS_nIORQ <=1;
      BUS_nRD   <=1;
      BUS_nWR   <=1;
		BUS_nM1   <=1;
   end else begin
	   BUS_A[15:0]  <= cpu_a;
		BUS_A[23:16] <= DEBUG;
	   BUS_D <= !nWR? cpu_dout : 8'bZ;
	   BUS_nMREQ <= nMREQ;
	   BUS_nIORQ <= nIORQ;
	   BUS_nRD <= nRD;
	   BUS_nWR <= nWR;
		BUS_nRFSH <= nRFSH;
		BUS_nBUSAK <= nBUSACK;
		BUS_nHALT <= nHALT;
		BUS_nM1 <= nM1;
		BUS_CLK <= clk_7;
	end
   BUS_nRESET <= ~reset;
end

`endif
endmodule
