/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Change the name of this module to something that reflects its functionality and includes your name for uniqueness
// For example tqvp_yourname_spi for an SPI peripheral.
// Then edit tt_wrapper.v line 41 and change tqvp_example to your chosen module name.
module tqvp_adder (
    input         clk,          // Clock - the TinyQV project clock is normally set to 64MHz.
    input         rst_n,        // Reset_n - low to reset.

    input  [7:0]  ui_in,        // The input PMOD, always available.  Note that ui_in[7] is normally used for UART RX.
                                // The inputs are synchronized to the clock, note this will introduce 2 cycles of delay on the inputs.

    output [7:0]  uo_out,       // The output PMOD.  Each wire is only connected if this peripheral is selected.
                                // Note that uo_out[0] is normally used for UART TX.

    input [5:0]   address,      // Address within this peripheral's address space
    input [31:0]  data_in,      // Data in to the peripheral, bottom 8, 16 or all 32 bits are valid on write.

    // Data read and write requests from the TinyQV core.
    input [1:0]   data_write_n, // 11 = no write, 00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    input [1:0]   data_read_n,  // 11 = no read,  00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    
    output [31:0] data_out,     // Data out from the peripheral, bottom 8, 16 or all 32 bits are valid on read when data_ready is high.
    output        data_ready,

    output        user_interrupt  // Dedicated interrupt request for this peripheral
);

    // Implement a 32-bit read/write register at address 0
    // reg [31:0] ctrl;
    // always @(posedge clk) begin
    //     if (!rst_n) begin
    //         ctrl <= 0;
    //     end else begin
    //         if (address == 6'h0 ) begin
    //             if (data_write_n != 2'b11)              ctrl[7:0]   <= data_in[7:0];
    //             if (data_write_n[1] != data_write_n[0]) ctrl[15:8]  <= data_in[15:8];
    //             if (data_write_n == 2'b10)              ctrl[31:16] <= data_in[31:16];
    //         end
    //     end
    // end

    // The bottom 8 bits of the stored data are added to ui_in and output to uo_out.
    
    reg hsync;
    reg vsync;
    wire visible;
    reg [9:0] pix_x;
    reg [9:0] pix_y;
    
    wire [1:0] R,G,B ;

    //wire start = ctrl[0] ;

    localparam OBJ_BYTES     = 4;
    localparam OBJ_REGION_SZ = OBJ_BYTES * MAX_SPRITES; // bytes used for object table
    localparam BITMAP_BASE   = OBJ_REGION_SZ;           // start address for bitmap storage (auto-adjusted)
    localparam BITMAP_BYTES  = 63 - OBJ_REGION_SZ;      // remaining bytes up to CONTROL_ADDR
    localparam CONTROL_ADDR  = 63;                      // last byte reserved for control/status

    // --- internal memories ---
    reg [7:0] active_obj_ram  [0:OBJ_REGION_SZ]; // support full 6-bit address
    reg [7:0] stage_obj_ram   [0:OBJ_REGION_SZ]; // support full 6-bit address
    reg [7:0] bitmap_ram      [0:BITMAP_BYTES-1]; // packed 1bpp bitmaps; host writes here only when enabled

    // --- control register bits (in CONTROL_ADDR) ---
    // bit0 = BITMAP_WRITE_EN  : when 1, host may write bitmap region
    // bit1 = STAGING_READY    : host writes 1 to indicate staging table is ready
    // bit2 = (reserved)
    reg [7:0] control_reg;

    // --- host read/write interface ---
    // Writes go to:
    //  - addresses < OBJ_REGION_SZ => stage_obj_ram (staging update)
    //  - BITMAP_BASE.. => bitmap_ram (only when BITMAP_WRITE_EN set)
    //  - CONTROL_ADDR => control_reg (byte writes only)
    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            // reset memories & control
            for (i = 0; i < OBJ_REGION_SZ; i = i + 1) begin
                active_obj_ram[i] <= 8'b0;
                stage_obj_ram[i]  <= 8'b0;
            end
            for (i = 0; i < BITMAP_BYTES; i = i + 1) begin
                bitmap_ram[i] <= 8'b0;
            end
            control_reg <= 8'b0;
        end else begin
            // Host writes (synchronous)
            if (data_write_n != 2'b11) begin
                // compute write width and perform writes (byte/half/word) at address
                // only perform writes if address and region valid and allowed by control_reg
                if (data_write_n == 2'b00) begin
                    // byte write
                    if (address < OBJ_REGION_SZ) begin
                        stage_obj_ram[address[5:0]] <= data_in[7:0];
                    end else if ((address >= BITMAP_BASE) && (address < BITMAP_BASE + BITMAP_BYTES)) begin
                        if (control_reg[0]) // BITMAP_WRITE_EN
                            bitmap_ram[address - BITMAP_BASE] <= data_in[7:0];
                    end else if (address == CONTROL_ADDR) begin
                        control_reg <= data_in[7:0];
                    end
                end else if (data_write_n == 2'b01) begin
                    // halfword (16-bit) write - address must be <= 62
                    if ((address + 1) < OBJ_REGION_SZ) begin
                        stage_obj_ram[address[5:0]]   <= data_in[7:0];
                        stage_obj_ram[(address+1)[5:0]] <= data_in[15:8];
                    end else if ((address >= BITMAP_BASE) && ((address + 1) < BITMAP_BASE + BITMAP_BYTES)) begin
                        if (control_reg[0]) begin
                            bitmap_ram[address - BITMAP_BASE]     <= data_in[7:0];
                            bitmap_ram[address+1 - BITMAP_BASE] <= data_in[15:8];
                        end
                    end else if (address == CONTROL_ADDR - 1) begin
                        // allow halfword write that ends at CONTROL_ADDR
                        control_reg <= data_in[15:8];
                    end
                end else if (data_write_n == 2'b10) begin
                    // word (32-bit) write - address must be <= 60
                    if ((address + 3) < OBJ_REGION_SZ) begin
                        stage_obj_ram[address[5:0]]   <= data_in[7:0];
                        stage_obj_ram[(address+1)[5:0]] <= data_in[15:8];
                        stage_obj_ram[(address+2)[5:0]] <= data_in[23:16];
                        stage_obj_ram[(address+3)[5:0]] <= data_in[31:24];
                    end else if ((address >= BITMAP_BASE) && ((address + 3) < BITMAP_BASE + BITMAP_BYTES)) begin
                        if (control_reg[0]) begin
                            bitmap_ram[address - BITMAP_BASE]     <= data_in[7:0];
                            bitmap_ram[address+1 - BITMAP_BASE] <= data_in[15:8];
                            bitmap_ram[address+2 - BITMAP_BASE] <= data_in[23:16];
                            bitmap_ram[address+3 - BITMAP_BASE] <= data_in[31:24];
                        end
                    end else if (address == CONTROL_ADDR - 3) begin
                        control_reg <= data_in[31:24];
                    end
                end
            end
            // Note: we keep reading/writing staging only. Active table is swapped later at vsync.
        end
    end

reg [1:0] data_read_n_reg;
reg [5:0] address_reg;

always @(posedge clk ) begin
    if (!rst_n) begin
        data_ready <= 1'b0;
        data_out   <= 32'b0;
        data_read_n_reg <= 2'b11;
        address_reg     <= 6'b0;
    end else begin
        // Latch the read request
        data_read_n_reg <= data_read_n;
        address_reg     <= address;

        data_ready <= (data_read_n_reg != 2'b11);
        data_out   <= 32'b0;
        if (data_read_n_reg != 2'b11) begin
            if (data_read_n_reg == 2'b00) begin
                // byte read
                if (address_reg < OBJ_REGION_SZ) data_out[7:0] <= active_obj_ram[address_reg[5:0]];
                else if ((address_reg >= BITMAP_BASE) && (address_reg < BITMAP_BASE + BITMAP_BYTES))
                    data_out[7:0] <= bitmap_ram[address_reg - BITMAP_BASE];
                else if (address_reg == CONTROL_ADDR) data_out[7:0] <= control_reg;
            end else if (data_read_n_reg == 2'b01) begin
                // halfword
                if ((address_reg + 1) < OBJ_REGION_SZ) begin
                    data_out[15:0] <= { active_obj_ram[(address_reg+1)[5:0]], active_obj_ram[address_reg[5:0]] };
                end else if ((address_reg >= BITMAP_BASE) && ((address_reg + 1) < BITMAP_BASE + BITMAP_BYTES)) begin
                    data_out[15:0] <= { bitmap_ram[address_reg+1 - BITMAP_BASE], bitmap_ram[address_reg - BITMAP_BASE] };
                end else if (address_reg == CONTROL_ADDR - 1) begin
                    data_out[15:0] <= { 8'b0, control_reg };
                end
            end else if (data_read_n_reg == 2'b10) begin
                // word
                if ((address_reg + 3) < OBJ_REGION_SZ) begin
                    data_out[31:0] <= { active_obj_ram[(address_reg+3)[5:0]], active_obj_ram[(address_reg+2)[5:0]],
                                        active_obj_ram[(address_reg+1)[5:0]], active_obj_ram[address_reg[5:0]] };
                end else if ((address_reg >= BITMAP_BASE) && ((address_reg + 3) < BITMAP_BASE + BITMAP_BYTES)) begin
                    data_out[31:0] <= { bitmap_ram[address_reg+3 - BITMAP_BASE], bitmap_ram[address_reg+2 - BITMAP_BASE],
                                        bitmap_ram[address_reg+1 - BITMAP_BASE], bitmap_ram[address_reg - BITMAP_BASE] };
                end else if (address_reg == CONTROL_ADDR - 3) begin
                    data_out[31:0] <= { 24'b0, control_reg };
                end
            end
        end
    end
end

    // ---- vsync / staging swap / user_interrupt protocol ----
    // On each rising edge of vsync:
    //   if (STAGING_READY == 0) => pulse user_interrupt for 1 cycle to ask host to provide staging data.
    //   if (STAGING_READY == 1) => copy staging_obj_ram -> active_obj_ram and clear STAGING_READY.
    reg vsync_d;
    always @(posedge clk) vsync_d <= vsync;

    // user_interrupt pulse generator (1 clock) on vsync rising when staging not ready
    always @(posedge clk) begin
        if (!rst_n) user_interrupt <= 1'b0;
        else begin
            user_interrupt <= 1'b0; // default
            if (vsync && !vsync_d) begin // rising edge
                if (!control_reg[1]) begin
                    // request new staging data for next frame
                    user_interrupt <= 1'b1;
                end else begin
                    // staging ready -> commit it now (copy to active)
                    for (i = 0; i < OBJ_REGION_SZ; i = i + 1) begin
                        active_obj_ram[i] <= stage_obj_ram[i];
                    end
                    // clear STAGING_READY bit (host must set it again next frame when new staging is ready)
                    control_reg[1] <= 1'b0;
                    user_interrupt <= 1'b0;
                end
            end
        end
    end
    
    video_controller u_video_controller(
        .clk      	(clk       ),
        .reset    	(rst_n     ),
        .polarity 	(1'b1      ), // 0 = negative polarity (VGA, SVGA), 1 = positive polarity (XGA, SXGA)
        .hsync    	(hsync     ),
        .vsync    	(vsync     ),
        .visible  	(visible   ),
        .pix_x    	(pix_x     ),
        .pix_y    	(pix_y     )
    );
    

    bg background (
        .clk(clk),
        .rst_n(rst_n),
        .video_active(visible),
        .pix_x(pix_x),
        .pix_y(pix_y),
        .vsync(vsync),
        .R(R),
        .G(G),
        .B(B)
        //.start      (start     )
    );


    // Address 0 reads the example data register.  
    // Address 4 reads ui_in
    // All other addresses read 0.
    assign data_out = (address == 6'h0) ? ctrl :
        (address == 6'h4) ? {22'h0,pix_x } :
        (address == 6'h8) ? {22'h0,pix_y } :
        (address == 6'hc) ? {24'h0,uo_out} :
                      32'h0;

    assign uo_out = {vsync, hsync, B, G, R}; 

    // All reads complete in 1 clock
    assign data_ready = 1;
    
    // User interrupt is generated on rising edge of ui_in[6], and cleared by writing a 1 to the low bit of address 8.
    reg example_interrupt;
    reg last_ui_in_6;

    always @(posedge clk) begin
        if (!rst_n) begin
            example_interrupt <= 0;
        end

        if (ui_in[6] && !last_ui_in_6) begin
            example_interrupt <= 1;
        end else if (address == 6'h8 && data_write_n != 2'b11 && data_in[0]) begin
            example_interrupt <= 0;
        end

        last_ui_in_6 <= ui_in[6];
    end

    assign user_interrupt = example_interrupt;

    // List all unused inputs to prevent warnings
    // data_read_n is unused as none of our behaviour depends on whether
    // registers are being read.
    wire _unused = &{data_read_n, 1'b0};

endmodule
