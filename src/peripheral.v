/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tqvp_adder (
    input  wire       clk,            // Clock - the TinyQV project clock is normally set to 64MHz.
    input  wire       rst_n,          // Reset_n - low to reset.

    input  wire [7:0] ui_in,          // The input PMOD
    output wire [7:0] uo_out,         // The output PMOD

    input  wire [5:0] address,        // Address within this peripheral's address space
    input  wire [31:0] data_in,        // Data in to the peripheral

    input  wire [1:0] data_write_n,   // 11 = no write, 00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    input  wire [1:0] data_read_n,    // 11 = no read,  00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    
    output            [31:0] data_out,       // Data out from the peripheral
    output            data_ready,
    output reg        user_interrupt  // Dedicated interrupt request for this peripheral
);

    // --- Signal Declarations ---
   
    reg         hsync;
    reg         vsync;
    wire        visible;
    reg  [9:0]  pix_x;
    reg  [9:0]  pix_y;
    wire [1:0]  R,G,B;
    wire        sprite_pixel_on;
    

    // --- Sprite Engine Parameters ---
    parameter MAX_SPRITES = 2;
    localparam OBJ_BYTES     = 4;
    localparam OBJ_REGION_SZ = OBJ_BYTES * MAX_SPRITES;
    localparam BITMAP_BASE   = OBJ_REGION_SZ;
    localparam BITMAP_BYTES  = 63 - OBJ_REGION_SZ;
    localparam CONTROL_ADDR  = 63;

    // --- Memories and Control ---
    reg [7:0] active_obj_ram [0:OBJ_REGION_SZ - 1];
    reg [7:0] stage_obj_ram  [0:OBJ_REGION_SZ - 1];
    reg [7:0] bitmap_ram     [0:BITMAP_BYTES - 1];
    reg [7:0] control_reg;

    // --- Rendering Logic & Temp Variables ---
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
    
    // --- Temporary variables for Verilog-1995 compatibility ---
    // FIX: All declarations moved here from inside always blocks.
   // reg [5:0] temp_addr_p1, temp_addr_p2, temp_addr_p3;
    //reg [7:0] temp_logic_x, temp_logic_y;


    // --- Host Read/Write Interface ---
    // FIX: Merged read and write logic into a single block to avoid multiple drivers and re-declarations.
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
            // Note: we keep reading/writing staging only. Active table is swapped later at vsync.
        end
    end
    // --- VSYNC and Staging Logic ---
    reg vsync_d;
    always @(posedge clk) vsync_d <= vsync;

    always @(posedge clk) begin
        if (!rst_n) user_interrupt <= 1'b0;
        else begin
            user_interrupt <= 1'b0; // default
            if (vsync && !vsync_d) begin // rising edge
                if (!control_reg[1]) begin // STAGING_READY is 0
                    user_interrupt <= 1'b1;
                end else begin // STAGING_READY is 1
                    for (i = 0; i < OBJ_REGION_SZ; i = i + 1) begin
                        active_obj_ram[i] <= stage_obj_ram[i];
                    end
                    control_reg[1] <= 1'b0;
                    user_interrupt <= 1'b0;
                end
            end
        end
    end
    
    // --- Sub-module Instantiations ---
    video_controller u_video_controller(
        .clk      (clk    ),
        .reset    (~rst_n ),
        .polarity (1'b1   ),
        .hsync    (hsync  ),
        .vsync    (vsync  ),
        .visible  (visible),
        .pix_x    (pix_x  ),
        .pix_y    (pix_y  )
    );

    wire start = control_reg[3];

    wire [1:0] bg_R, bg_G, bg_B;
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
        .start(start)
    );

    // --- Rendering Logic ---
    wire [7:0] logic_x = pix_x[9:2];
    wire [7:0] logic_y = pix_y[9:2];

    always @(*) begin
            pix_hit <= 1'b0; // Default value for the cycle
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
                            pix_hit <= 1'b1;
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

    assign uo_out = {1'b0, 1'b0, vsync, hsync, B, G, R};
    
endmodule

