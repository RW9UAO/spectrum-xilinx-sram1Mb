`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
module top_module(

	//----------------------------
	input  wire       CLK_50M,        // 100MHz system clock signal
	//----------------------------
	input  wire reset,					// CPU reset
	//input  wire [3:0] key,			// if want all 4 key
	//input	 wire key,					// if need only one button
	output wire [3:0]LED,			// onboard LEDs
	output wire [15:0] usb_dbg,		// Cypress FX2LP chip for logic analyzer
	//----------------------------	// VGA signals
	output wire VGA_VSYNC,
	output wire VGA_HSYNC,
	output wire [4:0] VGA_R,
	output wire [5:0] VGA_G,
	output wire [4:0] VGA_B,
	//-------------------------
	output wire BUZZER,					// std ZX sound
	output wire tape_out,				// to tape recorder
	input wire tape_in,				// from tape recorder
	//-------------------------		// PS2
	input wire PS2_KBCLK,
	input wire PS2_KBDAT,
	//sram control
	output wire [18:0]SRAM_A,
	inout wire [7:0]SRAM_D,
	output wire SRAM_WE,
	output wire SRAM_OE,
	output wire CStop,
	output wire CSdown,
	// sdcard
	output wire SD_clk,
	output wire SD_cs,
	output wire SD_datain,
	input wire SD_dataout,
	// sound
	output wire OUT_L,
	output wire OUT_R
);
//===========================================================
wire clk_100mhz;
wire clk_140mhz;
wire clk_28mhz;
wire clock;
wire clk_cpu;
wire ena_1m75hz;
reg [2:0]clk_cnt;
reg [4:0]flash;
reg [8:0]hcnt;
reg [9:0]vcnt;
reg hsync;
reg vsync;
reg screen;
reg screen1;
reg blank;
reg [7:0]vid_0_reg;
reg [7:0]vid_1_reg;
reg [7:0]vid_b_reg;
reg [7:0]vid_c_reg;
wire vid_dot;
reg r,rb;
reg g,gb;
reg b,bb;

wire [15:0]cpu_a_bus;
wire [7:0]cpu_d_bus;
wire cpu_mreq_n;
wire cpu_iorq_n;
wire cpu_wr_n;
wire cpu_rd_n;
reg cpu_int_n;
reg int_signal;
wire cpu_m1_n;

wire rom_sel;
wire [7:0]rom_do;
reg [7:0]sram_do;
wire [14:0]rom_addr;
wire [7:0]divmmcrom_do;

wire [14:0]video_addr_cpu;
wire [7:0]video_wr_data_cpu;
wire [7:0]video_rd_data_cpu;
reg video_wr_cpu;
wire [14:0]video_addr_vc;
wire [7:0]video_rd_data_vc;

reg port_1f_sel;
reg port_eb_sel;
reg port_fe_sel;
reg port_ff_sel;
reg port_7ffd_sel;
reg port_fffd_sel;
wire [7:0]port_e3;
wire [7:0]port_eb;
reg [4:0]port_fe;
reg [7:0]port_ff;
reg [7:0]port_7ffd;
reg [7:0]port_fffd;

wire [7:0]kb_a_bus;
wire [4:0]kb_do_bus;
wire [4:0]kempston;
wire [12:1]f_key;


reg divmmc_cs;

wire VCC;
wire GND;

//==================================================================================
main_clock PLL14(
	.RESET( GND ),
	.LOCKED(),
	.CLK_IN1(CLK_50M),		// input main clock
	.CLK_OUT1(clock),			// 14 MHz pixel clock
	.CLK_OUT2(clk_28mhz),
	.CLK_OUT3(clk_100mhz),
	.CLK_OUT4(clk_140mhz)
);
//==================================================================================
z80_top_direct_n Z80(
	.CLK(clk_cpu),
	.nM1(cpu_m1_n),
	.nMREQ(cpu_mreq_n),
	.nIORQ(cpu_iorq_n),
	.nRD(cpu_rd_n),
	.nWR(cpu_wr_n),
	.nRFSH(),
	.nHALT(),
	.nBUSACK(),
	.nWAIT(VCC),
	.nINT(cpu_int_n),
	.nNMI( f_key[5] ),
	.nRESET(reset),
	.nBUSRQ(VCC),
	.A(cpu_a_bus),
	.D(cpu_d_bus)
);
//==================================================================================

divmmcrom ROM1(
  .clka(clk_140mhz),
  .addra(cpu_a_bus),
  .douta(divmmcrom_do)
);

//bios128 ROM0(		// 128kb 32k image
bios48 ROM0(		// 48kb 16k image
//test_128k ROM2(		// t_rom512
//test128k_2 ROM2(	// test from DivMMC speccy proj
  .clka(clk_140mhz),
  .addra(rom_addr),
  .douta(rom_do)
);
//==================================================================================
bram_32kb videomem(
	// CPU part
  .clka(clk_140mhz),
  .wea(video_wr_cpu),
  .addra(video_addr_cpu),
  .dina(video_wr_data_cpu),
  .douta(video_rd_data_cpu),
  // video controller part
  .clkb(clk_140mhz),
  .web(GND),
  .addrb(video_addr_vc),
  .dinb({VCC,VCC,VCC,VCC,VCC,VCC,VCC,VCC}),
  .doutb(video_rd_data_vc)
);
//==================================================================================
zxkbd kbd(
		.clk(clock),
		.reset( ~reset),
		.res_k(),
		.ps2_clk(PS2_KBCLK),
		.ps2_data(PS2_KBDAT),
		.zx_kb_scan(kb_a_bus),
		.zx_kb_out(kb_do_bus),
		.k_joy(kempston),
		.f(f_key),   
		.num_joy()
);
//==================================================================================
divmmc sdcard(
	.I_CLK(clock),
	.I_CS(divmmc_cs),
	.I_RESET( ~reset ),
	.I_ADDR(cpu_a_bus),
	.I_DATA(cpu_d_bus),
	.O_DATA(port_eb),
	.I_WR_N(cpu_wr_n),
	.I_RD_N(cpu_rd_n),
	.I_IORQ_N(cpu_iorq_n),
	.I_MREQ_N(cpu_mreq_n),
	.I_M1_N(cpu_m1_n),
	.O_E3REG(port_e3),
	.O_AMAP(AUTOMAP),
	.O_CS_N(SD_cs),
	.O_SCLK(SD_clk),
	.O_MOSI(SD_datain),
	.I_MISO(SD_dataout)
);
//==================================================================================
wire [7:0]ssg0_do;
wire [7:0]ssg0_a;
wire [7:0]ssg0_b;
wire [7:0]ssg0_c;

turbosound sound(
		.I_CLK(clk_28mhz),//		28.00 MHz
		.I_ENA(ena_1m75hz),
		.I_ADDR(cpu_a_bus),
		.I_DATA(cpu_d_bus),
		.I_WR_N(cpu_wr_n),
		.I_IORQ_N(cpu_iorq_n),
		.I_M1_N(cpu_m1_n),
		.I_RESET_N(reset),
		.O_SEL(),
		// ssg0
		.O_SSG0_DA(ssg0_do),
		.O_SSG0_AUDIO_A(ssg0_a),
		.O_SSG0_AUDIO_B(ssg0_b),
		.O_SSG0_AUDIO_C(ssg0_c)
);
//==================================================================================
wire [15:0]audio_r;
wire [15:0]audio_l;
assign audio_l	= { 3'b000, ssg0_a, 5'b00000 } + { 3'b000, ssg0_b, 5'b00000 };
assign audio_r	= { 3'b000, ssg0_c, 5'b00000 } + { 3'b000, ssg0_b, 5'b00000 };

dac dacL(
	.I_CLK(clk_100mhz),
	.I_RESET( ~reset ),
	.I_DATA(audio_l),
	.O_DAC(OUT_L)
);
dac dacR(
	.I_CLK(clk_100mhz),
	.I_RESET( ~reset ),
	.I_DATA(audio_r),
	.O_DAC(OUT_R)
);
//==================================================================================
assign VCC = 1'b1;	// std logic
assign GND = 1'b0;	// std logic

// CPU 3,5 MHz clock
assign clk_cpu = clk_cnt[1];
assign ena_1m75hz = clk_cnt[2];

// CPU want lower 16kb
assign rom_sel = ( cpu_a_bus[15:14] == 2'b00 ) ? 1'b1 : 1'b0;
assign rom_addr = { port_7ffd[4], cpu_a_bus[13:0] };
//==================================================================================
// video RAM multiplexor

wire video_bank5;
assign video_bank5 = ( port_7ffd[7:5] == 3'b000 && port_7ffd[2:0] == 5) ? 1'b1 : 1'b0;
wire video_bank7;
assign video_bank7 = ( port_7ffd[7:5] == 3'b000 && port_7ffd[2:0] == 7) ? 1'b1 : 1'b0;

assign video_addr_cpu =
	// write to x4000
	( cpu_a_bus[15:14] == 2'b01 ) ? { port_7ffd[3], cpu_a_bus[13:0]} :
	// or write to xc000 and bank5 or bank7
	( cpu_a_bus[15:14] == 2'b11 && port_7ffd[2:0] == 5 ) ? { 1'b0, cpu_a_bus[13:0]}
	: { 1'b1, cpu_a_bus[13:0]};
	
assign video_wr_data_cpu = cpu_d_bus;

always @(posedge clock) begin
	if ( cpu_mreq_n == 0 && cpu_wr_n == 0 &&
	( ( cpu_a_bus[15:14] == 2'b01 ) || 
	// or write to xc000 and bank5 or bank7
	( cpu_a_bus[15:14] == 2'b11 && ( video_bank5 || video_bank7 ) ) ) ) begin
		video_wr_cpu <= 1'b1;
	end else begin
		video_wr_cpu <= 1'b0;
	end
end

assign video_addr_vc = 
	// check for pixels or attribute
	(hcnt[0] == 1'b0) ? 				
	// pixel
			{ port_7ffd[3], 1'b0, vcnt[8:7], vcnt[3:1], vcnt[6:4], hcnt[7:3]} :
	// attr
			{ port_7ffd[3], 4'b0110, vcnt[8:4], hcnt[7:3]};

// store pixels and attr from video ram to temp buffers
always @(posedge clock) begin
	case ( hcnt[2:0] )
		3'b100: begin
					vid_0_reg <= video_rd_data_vc;
			end
		3'b101:  begin
					vid_1_reg <= video_rd_data_vc;
					port_ff <= video_rd_data_vc;
			end
		3'b111: begin
			vid_b_reg <= vid_0_reg;
			vid_c_reg <= vid_1_reg;
			screen1 <= screen;
			end
	endcase
end

//==================================================================================
// CPU want port
always @(*)begin
	if (cpu_a_bus[7:0] == 8'h1F && cpu_iorq_n == 0) begin
		port_1f_sel <= 1'b1;
	end else begin
		port_1f_sel <= 1'b0;
	end
end
always @(*)begin
	if (cpu_a_bus[7:0] == 8'hEB && cpu_iorq_n == 0) begin
		port_eb_sel <= 1'b1;
	end else begin
		port_eb_sel <= 1'b0;
	end
end
always @(*)begin
	if (cpu_a_bus[7:0] == 8'hFE && cpu_iorq_n == 0) begin
		port_fe_sel <= 1'b1;
	end else begin
		port_fe_sel <= 1'b0;
	end
end
always @(*)begin
	if (cpu_a_bus[7:0] == 8'hFF && cpu_iorq_n == 0) begin
		port_ff_sel <= 1'b1;
	end else begin
		port_ff_sel <= 1'b0;
	end
end
always @(*)begin
	if (cpu_a_bus == 16'h7ffd && cpu_iorq_n == 0) begin
		port_7ffd_sel <= 1'b1;
	end else begin
		port_7ffd_sel <= 1'b0;
	end
end
always @(*)begin
	if (cpu_a_bus == 16'hfffd && cpu_iorq_n == 0) begin
		port_fffd_sel <= 1'b1;
	end else begin
		port_fffd_sel <= 1'b0;
	end
end
//========================================================
always @(posedge f_key[6] /*or negedge reset*/) begin
//	if( reset == 0) begin
//		divmmc_cs <= 0;
//	end else begin
		divmmc_cs <= ~ divmmc_cs;
//	end
end
always @(posedge clock) begin

	if( reset == 0) begin
		port_fe <= 8'h00;
		port_7ffd <= 8'h00;
//		port_eff7 <= 8'h00;
	end else
	if ( port_fe_sel == 1 && cpu_wr_n == 0) begin
		port_fe <= cpu_d_bus[4:0];
	end else
	if (port_7ffd_sel == 1 && cpu_wr_n == 0) begin
		port_7ffd <= cpu_d_bus;
	end
//	if (port_eff7_sel == 1 && cpu_wr_n == 0) begin
//		port_eff7 <= cpu_d_bus;
//	end
end


//==================================================================================
// sram multiplexor
wire CONMEM;
wire MAPRAM;
wire AUTOMAP;
assign CONMEM = port_e3[7];
assign MAPRAM = port_e3[6];

assign SRAM_A = 
	// divmmc ESXDOS RAM 2000-3FFF
	( (CONMEM || AUTOMAP) & cpu_a_bus[15:13] == 1'b001 ) ? { port_e3[5:0], cpu_a_bus[12:0] } :
	// x8000 aka Bank2
	( cpu_a_bus[15:14] == 2'b10 ) ? {2'b00 , 3'b010, cpu_a_bus[13:0]} :
	// Bank5 - from video memory
	// Bank7 - from video memory
	// xC000
	//( cpu_a_bus[15:14] == 2'b11 ) ? 
	//{2'b00, port_7ffd[2:0], cpu_a_bus[13:0]} ;	// 128k ver
	{port_7ffd[6:5], port_7ffd[2:0], cpu_a_bus[13:0]} ;	// 512kb ver

// RAM write, protect for lower 16kb and from video controller
assign SRAM_WE = ( cpu_mreq_n == 0 && cpu_wr_n == 0 && (cpu_a_bus[15] == 1'b1 || (CONMEM || AUTOMAP) )) ? 1'b0 : 1'b1;
assign SRAM_OE = ( cpu_mreq_n == 0 && cpu_rd_n == 0 && (cpu_a_bus[15] == 1'b1 || (CONMEM || AUTOMAP) )) ? 1'b0 : 1'b1;
// duap chip for 1024kb
//assign CStop = port_7ffd[7];//VCC;
//assign CSdown = ~ port_7ffd[7];//GND;
assign CStop = ( (CONMEM || AUTOMAP) && cpu_a_bus[15:13] == 1'b001 ) ? 1'b0 : 1'b1;
assign CSdown = ~ CStop;

// input RAM data alwas connect to CPU data bus
assign SRAM_D = ( SRAM_WE == 0 ) ? cpu_d_bus : 8'hZZ;

//==================================================================================
// keyboard decoder use top of addr bus
assign kb_a_bus = cpu_a_bus[15:8];

//==================================================================================
// CPU data bus multiplexer
wire mem_rd;
wire io_rd;
assign mem_rd = (cpu_iorq_n == 1 && cpu_mreq_n == 0 && cpu_rd_n == 0) ? 1'b1 : 1'b0;
assign io_rd = (cpu_iorq_n == 0 && cpu_mreq_n == 1 && cpu_rd_n == 0) ? 1'b1 : 1'b0;

assign cpu_d_bus =
// read x0000
	( mem_rd && rom_sel == 1 && CONMEM == 0 && AUTOMAP == 0 ) ? rom_do :
// divMMC ROM
	( mem_rd && cpu_a_bus[15:13] == 3'b000 && (CONMEM || AUTOMAP) ) ? divmmcrom_do:
// divMMC RAM
	( mem_rd && cpu_a_bus[15:13] == 3'b001 && (CONMEM || AUTOMAP) ) ? SRAM_D:
// read x4000
	( mem_rd && cpu_a_bus[15:14] == 2'b01 ) ? video_rd_data_cpu :
// read x8000
	( mem_rd && cpu_a_bus[15:14] == 2'b10 ) ? SRAM_D :
// read xC000, bank5
	( mem_rd && cpu_a_bus[15:14] == 2'b11 
	&& port_7ffd[7:5] == 3'b000 && port_7ffd[2:0] == 5) ? video_rd_data_cpu :
// read xC000, bank7
	( mem_rd && cpu_a_bus[15:14] == 2'b11 
	&& port_7ffd[7:5] == 3'b000 && port_7ffd[2:0] == 7) ? video_rd_data_cpu :
// read xC000, sram
	( mem_rd && cpu_a_bus[15:14] == 2'b11 ) ? SRAM_D :
// read read from xFE port
	( io_rd && port_fe_sel == 1) ? 
	{ 1'b1, tape_in, 1'b1, kb_do_bus} :
// read from kempston joy
	( io_rd && port_1f_sel == 1) ? 
	{ 3'b111, kempston} :
// read read from xFF port
	( io_rd && port_ff_sel == 1) ? 
	port_ff :
// read read from x7FFD port
	( io_rd && port_7ffd_sel == 1) ? 
	port_7ffd :
// read read from xEFF7 port
//	( io_rd && port_eff7_sel == 1) ? 
//	port_eff7 :
// divMMC
	( io_rd && port_eb_sel && divmmc_cs ) ? 
	port_eb :
// sound
	( cpu_mreq_n == 1 && cpu_rd_n == 0 && port_fffd_sel == 1 ) ?
	ssg0_do:
// read from unised ports, always read xFF
	( io_rd && cpu_rd_n == 0 ) ?
	8'b11111111 :
// CPU in write mode
	8'bZZZZZZZZ;
//==================================================================================
// VGA signals
assign VGA_R = {r,rb,rb,rb,rb};
assign VGA_G = {g,gb,gb,gb,gb,gb};
assign VGA_B = {b,bb,bb,bb,bb};
assign VGA_HSYNC = ( hsync == 0 ) ? 1'b0 : 1'b1;
assign VGA_VSYNC = ( vsync == 0 ) ? 1'b0 : 1'b1; 

// std ZX beeper
assign BUZZER = port_fe[4];	// not phisically connected

// tape recorder
assign tape_out = port_fe[3];	// not phisically connected
//==================================================================================
assign usb_dbg[7:0] = cpu_d_bus[7:0];
assign usb_dbg[8] = cpu_m1_n;
assign usb_dbg[9] = cpu_rd_n;
assign usb_dbg[10] = cpu_wr_n;
assign usb_dbg[11] = cpu_mreq_n;
assign usb_dbg[12] = cpu_iorq_n;
assign usb_dbg[13] = SRAM_WE;
assign usb_dbg[14] = SRAM_OE;
assign usb_dbg[15] = int_signal;


assign LED[3] = ~AUTOMAP;
assign LED[2] = ~MAPRAM;
assign LED[1] = ~CONMEM;
assign LED[0] = ~divmmc_cs;
//======================================================
// generate 3,5 MHz CPU clock from 14 Mhz pixel clock
always @(negedge clock) begin
	clk_cnt <= clk_cnt + 1'b1;
end

//======================================================
// INT inpulse generation
always @(posedge clock) begin
	// for one INT impulse, work fine
	if ( vcnt == 478 && hcnt == 316) begin 
		int_signal <= 1'b1;
	end else begin
		int_signal <= 1'b0;
	end
end

always @(posedge clk_100mhz) begin
	if( int_signal == 1'b1) begin
		cpu_int_n <= 1'b0;
	end
	// 428-316=112 pix; 32/448 * 112 = 8uS as original
	if ( hcnt == 428 ) begin
		cpu_int_n <= 1'b1;
	end
end

//======================================================
// horiz sync counter
always @(negedge clock) begin
	
	hcnt <= hcnt + 1'b1;
	
	if ( hcnt == 448 ) begin
		hcnt <= 0;
	end
end

//======================================================
// vertical sync counter
always @(negedge clock) begin

	if ( hcnt == 328 ) begin
	
		vcnt <= vcnt + 1'b1;
	
		if ( vcnt[9:1] == 312 ) begin
			vcnt[9:1] <= 0;
		end
		// flash	signal
		if ( vcnt == 10'b1000000000) begin
			flash <= flash + 1'b1;
		end
	end
end

//==================================================
// VGA sync
always @(posedge clock) begin
	if ( hcnt == 328 ) begin
		hsync <= 1'b0;
	end else 
	if ( hcnt == 381 ) begin
		hsync <= 1'b1;
	end
end

//==================================================
// VGA sync
always @(posedge clock) begin
	if ( vcnt[9:1] == 256 ) begin
		vsync <= 1'b0;
	end else
	if ( vcnt[9:1] == 260 ) begin
		vsync <= 1'b1;
	end
end

//==================================================
// signal for VGA picture
always @(posedge clock) begin
	if ( (hcnt > 301 && hcnt < 417) || (vcnt[9:1] > 224 && vcnt[9:1] < 285) ) begin
		blank <= 1'b1;
	end else begin
		blank <= 1'b0;
	end
end

//==================================================
// enable video output
always @(posedge clock) begin
	if ( hcnt < 256 && vcnt[9:1] < 192 ) begin
		screen <= 1'b1;
	end else begin
		screen <= 1'b0;
	end
end

//==================================================
// use multipexor as pixel shift
assign vid_dot = 	( hcnt[2:0] == 3'b000) ? vid_b_reg[7] :
						( hcnt[2:0] == 3'b001) ? vid_b_reg[6] :
						( hcnt[2:0] == 3'b010) ? vid_b_reg[5] :
						( hcnt[2:0] == 3'b011) ? vid_b_reg[4] :
						( hcnt[2:0] == 3'b100) ? vid_b_reg[3] :
						( hcnt[2:0] == 3'b101) ? vid_b_reg[2] :
						( hcnt[2:0] == 3'b110) ? vid_b_reg[1] :
						vid_b_reg[0];

//==================================================
// picture magic
wire [2:0]selector;
assign selector = {vid_dot, flash[4], vid_c_reg[7]};

always @(posedge clock) begin
	
	if( blank == 0 ) begin
		if (screen1 == 1) begin
			if ( selector == 3'b000 || selector == 3'b010 ||
				  selector == 3'b011 || selector == 3'b101 ) begin
						b <= vid_c_reg[3:3];
						bb <= ( vid_c_reg[3:3] && vid_c_reg[6:6] );
						r <= vid_c_reg[4:4];
						rb <= ( vid_c_reg[4:4] && vid_c_reg[6:6] );
						g <= vid_c_reg[5:5];
						gb <= ( vid_c_reg[5:5] && vid_c_reg[6:6] );
			end else begin
						b <= vid_c_reg[0:0];
						bb <= ( vid_c_reg[0:0] && vid_c_reg[6:6] );
						r <= vid_c_reg[1:1];
						rb <= ( vid_c_reg[1:1] && vid_c_reg[6:6] );
						g <= vid_c_reg[2:2];
						gb <= ( vid_c_reg[2:2] && vid_c_reg[6:6] );
			end
		end else begin // screen1 == 0
						b <= port_fe[0];
						r <= port_fe[1];
						g <= port_fe[2];
						rb <= 0;
						gb <= 0;
						bb <= 0;
		end
	end else begin	//blank == 1
						b <= 0;
						r <= 0;
						g <= 0;
						rb <= 0;
						gb <= 0;
						bb <= 0;
	end
end


endmodule
