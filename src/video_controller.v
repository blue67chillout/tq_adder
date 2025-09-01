module video_controller (
    clk,
    reset,
    hsync,
    vsync,
    visible,
    pix_x,
    pix_y,
    polarity
);

  input clk;
  input reset;
  input polarity; 
  output reg hsync, vsync;
  output visible;
  output reg [9:0] pix_x;
  output reg [9:0] pix_y;

  // declarations for TV-simulator sync parameters
  // horizontal constants
  parameter H_DISPLAY = 1024;  // horizontal display width
  parameter H_BACK = 160;  // horizontal left border (back porch)
  parameter H_FRONT = 24;  // horizontal right border (front porch)
  parameter H_SYNC = 136;  // horizontal sync width
  // vertical constants
  parameter V_DISPLAY = 768;  // vertical display height
  parameter V_TOP = 29;  // vertical top border
  parameter V_BOTTOM = 6;  // vertical bottom border
  parameter V_SYNC = 6;  // vertical sync # lines
  // derived constants
  parameter H_SYNC_START = H_DISPLAY + H_FRONT;
  parameter H_SYNC_END = H_DISPLAY + H_FRONT + H_SYNC - 1;
  parameter H_MAX = H_DISPLAY + H_BACK + H_FRONT + H_SYNC - 1;
  parameter V_SYNC_START = V_DISPLAY + V_BOTTOM;
  parameter V_SYNC_END = V_DISPLAY + V_BOTTOM + V_SYNC - 1;
  parameter V_MAX = V_DISPLAY + V_TOP + V_BOTTOM + V_SYNC - 1;

  wire hmaxxed = ({pix_x, 1'b0} == H_MAX) || reset;  // set when pix_x is maximum
  wire vmaxxed = ({pix_y, 1'b0} == V_MAX) || reset;  // set when pix_y is maximum

  // horizontal position counter
  always @(posedge clk) begin
  if (polarity)  begin
    hsync <= (({pix_x, 1'b0} >= H_SYNC_START) && ({pix_x, 1'b0} <= H_SYNC_END));
  end
  else begin
    hsync <= ~(({pix_x, 1'b0} >= H_SYNC_START) && ({pix_x, 1'b0} <= H_SYNC_END));
  end
    
  if (hmaxxed) pix_x <= 0;
  else pix_x <= pix_x + 1;
  end

  // vertical position counter
  always @(posedge clk) begin
  if (polarity) begin
    vsync <= (({pix_y, 1'b0} >= V_SYNC_START) && ({pix_y, 1'b0} <= V_SYNC_END));
  end
  else begin
    vsync <= (({pix_y, 1'b0} >= V_SYNC_START) && ({pix_y, 1'b0} <= V_SYNC_END));
  end
  if (hmaxxed)
    if (vmaxxed) pix_y <= 0;
    else pix_y <= pix_y + 1;
  end

  // visible is set when beam is in "safe" visible frame
  assign visible = (({pix_x, 1'b0} < H_DISPLAY) && ({pix_y, 1'b0} < V_DISPLAY));

 

endmodule
