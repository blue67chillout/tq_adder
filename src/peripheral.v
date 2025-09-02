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
    //         if (address == 6'h50 ) begin
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
    
    wire [1:0]R,G,B ;

    wire [1:0]bg_R,bg_G,bg_B ;
    

    wire start = control_reg[4] ;

    integer   spr_idx;
    reg       pix_hit;
    reg [7:0] x, y, bitmap_offset, size_byte;
    reg [3:0] width, height;
    reg [3:0] spr_x, spr_y;
    integer   bit_offset;
    integer   byte_addr;
    integer   bit_in_byte;
    reg [7:0] bmp_byte;
    reg       bmp_bit;
    
  
   //reg [5:0] temp_addr_p1, temp_addr_p2, temp_addr_p3;
   reg [7:0] temp_logic_x, temp_logic_y;
    
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
        .R(bg_R),
        .G(bg_G),
        .B(bg_B),
        .start      (start     )
    );


    // Address 0 reads the example data register.  
    // Address 4 reads ui_in
    // All other addresses read 0.

    
    // User interrupt is generated on rising edge of ui_in[6], and cleared by writing a 1 to the low bit of address 8.
    // reg example_interrupt;
    // reg last_ui_in_6;

    // always @(posedge clk) begin
    //     if (!rst_n) begin
    //         example_interrupt <= 0;
    //     end

    //     if (ui_in[6] && !last_ui_in_6) begin
    //         example_interrupt <= 1;
    //     end else if (address == 6'h40 && data_write_n != 2'b11 && data_in[0]) begin
    //         example_interrupt <= 0;
    //     end

    //     last_ui_in_6 <= ui_in[6];
    // end

    // assign user_interrupt = example_interrupt;

    

    localparam MAX_SPRITES = 1;
    localparam OBJ_BYTES     = 4;
    localparam OBJ_REGION_SZ = OBJ_BYTES * MAX_SPRITES;
    localparam BITMAP_BASE   = OBJ_REGION_SZ;
    localparam BITMAP_BYTES  = 31 - OBJ_REGION_SZ;
    localparam CONTROL_ADDR  = 32;

    reg [7:0] active_obj_ram [0:OBJ_REGION_SZ - 1];
    reg [7:0] stage_obj_ram  [0:OBJ_REGION_SZ - 1];
    reg [7:0] bitmap_ram     [0:BITMAP_BYTES - 1];
    reg [7:0] control_reg;

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
                        stage_obj_ram[address] <= data_in[7:0];
                    end else if ((address >= BITMAP_BASE) && (address < BITMAP_BASE + BITMAP_BYTES)) begin
                        if (control_reg[0]) // BITMAP_WRITE_EN
                            bitmap_ram[address - BITMAP_BASE] <= data_in[7:0];
                    end else if (address == CONTROL_ADDR) begin
                        control_reg <= data_in[7:0];
                    end
                end else if (data_write_n == 2'b01) begin
                    // halfword (16-bit) write - address must be <= 62
                    if ((address + 1) < OBJ_REGION_SZ) begin
                        stage_obj_ram[address]   <= data_in[7:0];
                        stage_obj_ram[address+1] <= data_in[15:8];
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
                        stage_obj_ram[address]   <= data_in[7:0];
                        stage_obj_ram[address+1] <= data_in[15:8];
                        stage_obj_ram[address+2] <= data_in[23:16];
                        stage_obj_ram[address+3] <= data_in[31:24];
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
            
            // **FIX**: Vsync logic moved here from the second always block.
            // This logic will be overridden by a CPU write on the same clock cycle,
            // which is the desired priority.
            if (vsync && !vsync_d) begin // rising edge of vsync
                if (control_reg[1]) begin // STAGING_READY is 1
                    // Copy staged object data to active memory
                    for (i = 0; i < OBJ_REGION_SZ; i = i + 1) begin
                        active_obj_ram[i] <= stage_obj_ram[i];
                    end
                    // Clear the staging ready flag automatically
                    control_reg[1] <= 1'b0;
                end
            end
        end
    end

   reg vsync_d;
    always @(posedge clk) vsync_d <= vsync;
    reg frame_interrupt ;
    always @(posedge clk) begin
        if (!rst_n) begin
            frame_interrupt <= 1'b0;
        end else begin
            frame_interrupt <= 1'b0; // Default to clearing the interrupt each cycle
            if (vsync && !vsync_d) begin // Rising edge of vsync
                // If staging wasn't ready at the start of the frame, generate an interrupt
                if (!control_reg[1]) begin // STAGING_READY is 0
                    frame_interrupt <= 1'b1;
                end
                // The transfer logic for active_obj_ram and control_reg has been moved
            end
        end
    end
    assign user_interrupt = frame_interrupt ;


    wire [7:0] logic_x = pix_x[9:2];
    wire [7:0] logic_y = pix_y[9:2];

    always @(*) begin
            pix_hit = 1'b0; // Default value for the cycle
            temp_logic_x = 0;
            temp_logic_y = 0;
            spr_x        = 0;
            spr_y        = 0;
            bit_offset   = 0;
            byte_addr    = 0;
            bit_in_byte  = 0;
            bmp_byte     = 0;
            bmp_bit      = 0;
            for (spr_idx = 0; spr_idx < MAX_SPRITES; spr_idx = spr_idx + 1) begin
                // Read sprite attributes for this iteration
                x             = active_obj_ram[spr_idx*OBJ_BYTES + 0];
                y             = active_obj_ram[spr_idx*OBJ_BYTES + 1];
                bitmap_offset = active_obj_ram[spr_idx*OBJ_BYTES + 2];
                size_byte     = active_obj_ram[spr_idx*OBJ_BYTES + 3];
                width         = size_byte[7:4] + 1;
                height        = size_byte[3:0] + 1;

                if (visible && (logic_x >= x) && (logic_x < x + width) && (logic_y >= y) && (logic_y < y + height)) begin
                    temp_logic_x  = logic_x - x;
                    temp_logic_y  = logic_y - y;
                    spr_x         = temp_logic_x[3:0];
                    spr_y         = temp_logic_y[3:0];
                    bit_offset    = spr_y * width + spr_x;
                    byte_addr     = bitmap_offset + (bit_offset >> 3);
                    bit_in_byte   = bit_offset & 3'h7;
                    if ((byte_addr >= 0) && (byte_addr < BITMAP_BYTES)) begin
                        bmp_byte = bitmap_ram[byte_addr];
                        bmp_bit  = bmp_byte[bit_in_byte];
                        if(bmp_bit) begin
                            pix_hit = 1'b1;
                        end
                    end
                end
            end
    end
    

    // --- Final Output Assignments ---
   assign sprite_pixel_on = pix_hit;

    assign R = sprite_pixel_on ? 2'b11 : bg_R;
    assign G = sprite_pixel_on ? 2'b11 : bg_G;
    assign B = sprite_pixel_on ? 2'b11 : bg_B;

        assign data_out = (address == 6'h0) ? control_reg :
        (address == 6'h4) ? {22'h0,pix_x } :
        (address == 6'h8) ? {22'h0,pix_y } :
        (address == 6'hc) ? {24'h0,uo_out} :
                      32'h0;

    assign uo_out = {vsync, hsync, B, G, R}; 

    // All reads complete in 1 clock
    assign data_ready = 1;

    // List all unused inputs to prevent warnings
    // data_read_n is unused as none of our behaviour depends on whether
    // registers are being read.
    wire _unused = &{data_read_n, 1'b0};

endmodule

