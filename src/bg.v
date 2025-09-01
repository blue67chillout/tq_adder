module bg (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       video_active,  // 1 = pixel is valid
    input  wire [9:0] pix_x,
    input  wire [9:0] pix_y,
    input  wire       vsync,
    output wire [1:0] R,
    output wire [1:0] G,
    output wire [1:0] B,
    input  wire       start
);
    localparam H_RES = 1024;
    localparam V_RES = 768;

    localparam GROUND_Y = V_RES-140,
               MOUND_X0 = 306,
               MOUND_W = 64,
               HALF_MOUND_W = 32;

    //----------------------- Scrolling Counter -------------------------
    reg [9:0] scroll_counter;
    always @(posedge vsync or negedge rst_n)
        if (!rst_n) scroll_counter <= 0;
        else        scroll_counter <= scroll_counter + 1;

    //----------------------- Mound LUT by Function ---------------------
    function [2:0] mound_lut_val;
        input [4:0] idx;
        begin
            case(idx)
                5'd0, 5'd1, 5'd2, 5'd3, 5'd4, 5'd5: mound_lut_val = 3'd0;
                5'd6, 5'd7, 5'd8: mound_lut_val = 3'd1;
                5'd9, 5'd10, 5'd11, 5'd12: mound_lut_val = 3'd2;
                5'd13, 5'd14, 5'd15: mound_lut_val = 3'd3;
                5'd16, 5'd17, 5'd18: mound_lut_val = 3'd4;
                5'd19, 5'd20, 5'd21: mound_lut_val = 3'd5;
                default: mound_lut_val = 3'd6;
            endcase
        end
    endfunction

    wire [10:0] temp_x = pix_x + scroll_counter - MOUND_X0;
    wire [9:0] mound_x = (temp_x >= H_RES) ? (temp_x - H_RES) : temp_x;
    wire in_mound_region = (mound_x < MOUND_W);
    
    wire [9:0] inverted_mound_x = MOUND_W-1 - mound_x;
    wire [5:0] mound_index = (mound_x < HALF_MOUND_W) ? mound_x[5:0] : inverted_mound_x[5:0];
    
    wire [2:0] mound_val = mound_lut_val(mound_index[4:0]);
    wire [9:0] ground_y_for_x = in_mound_region 
                                  ? (GROUND_Y - {7'b0, mound_val}) 
                                  : GROUND_Y;
    wire is_ground_line = (pix_y == ground_y_for_x[9:0]);

    //------------------- Ground Dots, Scrolling ------------------------
    wire [10:0] scroll_x = pix_x + scroll_counter;
    
    // --- FIX START: Create intermediate wires for all mod calculations ---
    wire [10:0] mod8_sub16  = scroll_x - 16;
    wire [10:0] mod8_sub8   = scroll_x - 8;
    wire [10:0] mod11_sub22 = scroll_x - 22;
    wire [10:0] mod11_sub11 = scroll_x - 11;
    wire [10:0] mod17_sub34 = scroll_x - 34;
    wire [10:0] mod17_sub17 = scroll_x - 17;
    
    wire [3:0] mod8  = (scroll_x>=16) ? mod8_sub16[3:0]  : (scroll_x>=8) ? mod8_sub8[3:0]   : scroll_x[3:0];
    wire [3:0] mod11 = (scroll_x>=22) ? mod11_sub22[3:0] : (scroll_x>=11)? mod11_sub11[3:0] : scroll_x[3:0];
    wire [4:0] mod17 = (scroll_x>=34) ? mod17_sub34[4:0] : (scroll_x>=17)? mod17_sub17[4:0] : scroll_x[4:0];
    // --- FIX END ---
    
    wire is_ground_dot =
         (pix_y > ground_y_for_x) && (pix_y <= ground_y_for_x + 8) &&
         ((mod8  == 2 && pix_y == ground_y_for_x+3)  ||
          (mod11 == 4 && pix_y == ground_y_for_x+5)  ||
          (mod17 == 9 && pix_y == ground_y_for_x+7));

    //----------------------------- Clouds ------------------------------
    localparam CLOUD_W = 20, CLOUD_H = 8, CLOUD_SCALE = 2;

    function [CLOUD_W-1:0] get_cloud_sprite_line;
        input [2:0] y;
        begin
            case (y)
                3'd0: get_cloud_sprite_line = 20'b00000001111000000000;
                3'd1: get_cloud_sprite_line = 20'b00000111111100000000;
                3'd2: get_cloud_sprite_line = 20'b00011111111110000000;
                3'd3: get_cloud_sprite_line = 20'b00111111111111000000;
                3'd4: get_cloud_sprite_line = 20'b01111111111111100000;
                3'd5: get_cloud_sprite_line = 20'b00111111111111000000;
                3'd6: get_cloud_sprite_line = 20'b00011111111110000000;
                3'd7: get_cloud_sprite_line = 20'b00000111111100000000;
                default: get_cloud_sprite_line = 20'b0;
            endcase
        end
    endfunction

    // --- FIX START: Create intermediate wires for cloud coordinate calculations ---
    wire [10:0] temp_c1_x = 140 + H_RES - (scroll_counter >> 1);
    wire [10:0] c1_sub_hres = temp_c1_x - H_RES;
    wire [9:0]  c1_x = (temp_c1_x >= H_RES) ? c1_sub_hres[9:0] : temp_c1_x[9:0];
    
    wire [10:0] temp_c2_x = 340 + H_RES - (scroll_counter >> 1);
    wire [10:0] c2_sub_hres = temp_c2_x - H_RES;
    wire [9:0]  c2_x = (temp_c2_x >= H_RES) ? c2_sub_hres[9:0] : temp_c2_x[9:0];
    // --- FIX END ---

    localparam C1_Y = GROUND_Y - 156;
    localparam C2_Y = GROUND_Y - 136;

    wire in_cloud1_box = (pix_x >= c1_x) && (pix_x < c1_x + CLOUD_W*CLOUD_SCALE) &&
                         (pix_y >= C1_Y) && (pix_y < C1_Y + CLOUD_H*CLOUD_SCALE);
    wire in_cloud2_box = (pix_x >= c2_x) && (pix_x < c2_x + CLOUD_W*CLOUD_SCALE) &&
                         (pix_y >= C2_Y) && (pix_y < C2_Y + CLOUD_H*CLOUD_SCALE);

    wire [9:0] c1_local_x = pix_x - c1_x;
    wire [9:0] c1_local_y = pix_y - C1_Y;
    wire [9:0] c2_local_x = pix_x - c2_x;
    wire [9:0] c2_local_y = pix_y - C2_Y;

    // --- FIX START: Create intermediate wires for cloud sprite indexing ---
    wire [9:0] c1_local_x_shifted = c1_local_x >> 1;
    wire [9:0] c1_local_y_shifted = c1_local_y >> 1;
    wire [9:0] c2_local_x_shifted = c2_local_x >> 1;
    wire [9:0] c2_local_y_shifted = c2_local_y >> 1;

    wire [4:0] c1_sprite_x = c1_local_x_shifted[4:0]; // CLOUD_SCALE = 2
    wire [2:0] c1_sprite_y = c1_local_y_shifted[2:0];
    wire [4:0] c2_sprite_x = c2_local_x_shifted[4:0];
    wire [2:0] c2_sprite_y = c2_local_y_shifted[2:0];
    // --- FIX END ---

    wire [CLOUD_W-1:0] cloud_sprite_line = get_cloud_sprite_line(c1_sprite_y);
    wire [CLOUD_W-1:0] cloud_sprite_line2 = get_cloud_sprite_line(c2_sprite_y);

    wire is_cloud1 = in_cloud1_box && cloud_sprite_line[CLOUD_W-1-c1_sprite_x];
    wire is_cloud2 = in_cloud2_box && cloud_sprite_line2[CLOUD_W-1-c2_sprite_x];
    wire is_cloud  = is_cloud1 || is_cloud2;

    //------------------------ Stars: Plus and Cross Twinkle -----------
    localparam STAR_SIZE = 2;

    wire is_star_plus =
        (((pix_x ==  47) && (pix_y >= (GROUND_Y-180-STAR_SIZE) && pix_y <= (GROUND_Y-180+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-180)) && (pix_x >= (47-STAR_SIZE) && pix_x <= (47+STAR_SIZE)))) ||
        (((pix_x == 110) && (pix_y >= (GROUND_Y-170-STAR_SIZE) && pix_y <= (GROUND_Y-170+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-170)) && (pix_x >= (110-STAR_SIZE) && pix_x <= (110+STAR_SIZE)))) ||
        (((pix_x == 154) && (pix_y >= (GROUND_Y-155-STAR_SIZE) && pix_y <= (GROUND_Y-155+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-155)) && (pix_x >= (154-STAR_SIZE) && pix_x <= (154+STAR_SIZE)))) ||
        (((pix_x == 205) && (pix_y >= (GROUND_Y-160-STAR_SIZE) && pix_y <= (GROUND_Y-160+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-160)) && (pix_x >= (205-STAR_SIZE) && pix_x <= (205+STAR_SIZE)))) ||
        (((pix_x == 290) && (pix_y >= (GROUND_Y-145-STAR_SIZE) && pix_y <= (GROUND_Y-145+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-145)) && (pix_x >= (290-STAR_SIZE) && pix_x <= (290+STAR_SIZE)))) ||
        (((pix_x == 382) && (pix_y >= (GROUND_Y-168-STAR_SIZE) && pix_y <= (GROUND_Y-168+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-168)) && (pix_x >= (382-STAR_SIZE) && pix_x <= (382+STAR_SIZE)))) ||
        (((pix_x == 440) && (pix_y >= (GROUND_Y-150-STAR_SIZE) && pix_y <= (GROUND_Y-150+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-150)) && (pix_x >= (440-STAR_SIZE) && pix_x <= (440+STAR_SIZE)))) ||
        (((pix_x == 496) && (pix_y >= (GROUND_Y-165-STAR_SIZE) && pix_y <= (GROUND_Y-165+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-165)) && (pix_x >= (496-STAR_SIZE) && pix_x <= (496+STAR_SIZE)))) ||
        (((pix_x ==  60) && (pix_y >= (GROUND_Y-140-STAR_SIZE) && pix_y <= (GROUND_Y-140+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-140)) && (pix_x >= (60-STAR_SIZE) && pix_x <= (60+STAR_SIZE)))) ||
        (((pix_x == 130) && (pix_y >= (GROUND_Y-135-STAR_SIZE) && pix_y <= (GROUND_Y-135+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-135)) && (pix_x >= (130-STAR_SIZE) && pix_x <= (130+STAR_SIZE)))) ||
        (((pix_x == 210) && (pix_y >= (GROUND_Y-178-STAR_SIZE) && pix_y <= (GROUND_Y-178+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-178)) && (pix_x >= (210-STAR_SIZE) && pix_x <= (210+STAR_SIZE)))) ||
        (((pix_x == 330) && (pix_y >= (GROUND_Y-120-STAR_SIZE) && pix_y <= (GROUND_Y-120+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-120)) && (pix_x >= (330-STAR_SIZE) && pix_x <= (330+STAR_SIZE)))) ||
        (((pix_x == 390) && (pix_y >= (GROUND_Y-148-STAR_SIZE) && pix_y <= (GROUND_Y-148+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-148)) && (pix_x >= (390-STAR_SIZE) && pix_x <= (390+STAR_SIZE)))) ||
        (((pix_x == 480) && (pix_y >= (GROUND_Y-182-STAR_SIZE) && pix_y <= (GROUND_Y-182+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-182)) && (pix_x >= (480-STAR_SIZE) && pix_x <= (480+STAR_SIZE)))) ||
        (((pix_x == 530) && (pix_y >= (GROUND_Y-125-STAR_SIZE) && pix_y <= (GROUND_Y-125+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-125)) && (pix_x >= (530-STAR_SIZE) && pix_x <= (530+STAR_SIZE)))) ||
        (((pix_x == 605) && (pix_y >= (GROUND_Y-110-STAR_SIZE) && pix_y <= (GROUND_Y-110+STAR_SIZE))) ||
        ((pix_y == (GROUND_Y-110)) && (pix_x >= (605-STAR_SIZE) && pix_x <= (605+STAR_SIZE))));

    wire is_star_cross =
        (((pix_x-47)  == (pix_y - (GROUND_Y-180))) && (pix_x >= 47-STAR_SIZE  && pix_x <= 47+STAR_SIZE)  && (pix_y >= (GROUND_Y-180)-STAR_SIZE  && pix_y <= (GROUND_Y-180)+STAR_SIZE)) ||
        (((pix_x-47)  ==-(pix_y - (GROUND_Y-180))) && (pix_x >= 47-STAR_SIZE  && pix_x <= 47+STAR_SIZE)  && (pix_y >= (GROUND_Y-180)-STAR_SIZE  && pix_y <= (GROUND_Y-180)+STAR_SIZE)) ||
        (((pix_x-110) == (pix_y - (GROUND_Y-170))) && (pix_x >= 110-STAR_SIZE && pix_x <= 110+STAR_SIZE) && (pix_y >= (GROUND_Y-170)-STAR_SIZE && pix_y <= (GROUND_Y-170)+STAR_SIZE)) ||
        (((pix_x-110) ==-(pix_y - (GROUND_Y-170))) && (pix_x >= 110-STAR_SIZE && pix_x <= 110+STAR_SIZE) && (pix_y >= (GROUND_Y-170)-STAR_SIZE && pix_y <= (GROUND_Y-170)+STAR_SIZE)) ||
        (((pix_x-154) == (pix_y - (GROUND_Y-155))) && (pix_x >= 154-STAR_SIZE && pix_x <= 154+STAR_SIZE) && (pix_y >= (GROUND_Y-155)-STAR_SIZE && pix_y <= (GROUND_Y-155)+STAR_SIZE)) ||
        (((pix_x-154) ==-(pix_y - (GROUND_Y-155))) && (pix_x >= 154-STAR_SIZE && pix_x <= 154+STAR_SIZE) && (pix_y >= (GROUND_Y-155)-STAR_SIZE && pix_y <= (GROUND_Y-155)+STAR_SIZE)) ||
        (((pix_x-205) == (pix_y - (GROUND_Y-160))) && (pix_x >= 205-STAR_SIZE && pix_x <= 205+STAR_SIZE) && (pix_y >= (GROUND_Y-160)-STAR_SIZE && pix_y <= (GROUND_Y-160)+STAR_SIZE)) ||
        (((pix_x-205) ==-(pix_y - (GROUND_Y-160))) && (pix_x >= 205-STAR_SIZE && pix_x <= 205+STAR_SIZE) && (pix_y >= (GROUND_Y-160)-STAR_SIZE && pix_y <= (GROUND_Y-160)+STAR_SIZE)) ||
        (((pix_x-290) == (pix_y - (GROUND_Y-145))) && (pix_x >= 290-STAR_SIZE && pix_x <= 290+STAR_SIZE) && (pix_y >= (GROUND_Y-145)-STAR_SIZE && pix_y <= (GROUND_Y-145)+STAR_SIZE)) ||
        (((pix_x-290) ==-(pix_y - (GROUND_Y-145))) && (pix_x >= 290-STAR_SIZE && pix_x <= 290+STAR_SIZE) && (pix_y >= (GROUND_Y-145)-STAR_SIZE && pix_y <= (GROUND_Y-145)+STAR_SIZE)) ||
        (((pix_x-382) == (pix_y - (GROUND_Y-168))) && (pix_x >= 382-STAR_SIZE && pix_x <= 382+STAR_SIZE) && (pix_y >= (GROUND_Y-168)-STAR_SIZE && pix_y <= (GROUND_Y-168)+STAR_SIZE)) ||
        (((pix_x-382) ==-(pix_y - (GROUND_Y-168))) && (pix_x >= 382-STAR_SIZE && pix_x <= 382+STAR_SIZE) && (pix_y >= (GROUND_Y-168)-STAR_SIZE && pix_y <= (GROUND_Y-168)+STAR_SIZE)) ||
        (((pix_x-440) == (pix_y - (GROUND_Y-150))) && (pix_x >= 440-STAR_SIZE && pix_x <= 440+STAR_SIZE) && (pix_y >= (GROUND_Y-150)-STAR_SIZE && pix_y <= (GROUND_Y-150)+STAR_SIZE)) ||
        (((pix_x-440) ==-(pix_y - (GROUND_Y-150))) && (pix_x >= 440-STAR_SIZE && pix_x <= 440+STAR_SIZE) && (pix_y >= (GROUND_Y-150)-STAR_SIZE && pix_y <= (GROUND_Y-150)+STAR_SIZE)) ||
        (((pix_x-496) == (pix_y - (GROUND_Y-165))) && (pix_x >= 496-STAR_SIZE && pix_x <= 496+STAR_SIZE) && (pix_y >= (GROUND_Y-165)-STAR_SIZE && pix_y <= (GROUND_Y-165)+STAR_SIZE)) ||
        (((pix_x-496) ==-(pix_y - (GROUND_Y-165))) && (pix_x >= 496-STAR_SIZE && pix_x <= 496+STAR_SIZE) && (pix_y >= (GROUND_Y-165)-STAR_SIZE && pix_y <= (GROUND_Y-165)+STAR_SIZE)) ||
        (((pix_x-60)  == (pix_y - (GROUND_Y-140))) && (pix_x >= 60-STAR_SIZE  && pix_x <= 60+STAR_SIZE)  && (pix_y >= (GROUND_Y-140)-STAR_SIZE  && pix_y <= (GROUND_Y-140)+STAR_SIZE)) ||
        (((pix_x-60)  ==-(pix_y - (GROUND_Y-140))) && (pix_x >= 60-STAR_SIZE  && pix_x <= 60+STAR_SIZE)  && (pix_y >= (GROUND_Y-140)-STAR_SIZE  && pix_y <= (GROUND_Y-140)+STAR_SIZE)) ||
        (((pix_x-130) == (pix_y - (GROUND_Y-135))) && (pix_x >= 130-STAR_SIZE && pix_x <= 130+STAR_SIZE) && (pix_y >= (GROUND_Y-135)-STAR_SIZE && pix_y <= (GROUND_Y-135)+STAR_SIZE)) ||
        (((pix_x-130) ==-(pix_y - (GROUND_Y-135))) && (pix_x >= 130-STAR_SIZE && pix_x <= 130+STAR_SIZE) && (pix_y >= (GROUND_Y-135)-STAR_SIZE && pix_y <= (GROUND_Y-135)+STAR_SIZE)) ||
        (((pix_x-210) == (pix_y - (GROUND_Y-178))) && (pix_x >= 210-STAR_SIZE && pix_x <= 210+STAR_SIZE) && (pix_y >= (GROUND_Y-178)-STAR_SIZE && pix_y <= (GROUND_Y-178)+STAR_SIZE)) ||
        (((pix_x-210) ==-(pix_y - (GROUND_Y-178))) && (pix_x >= 210-STAR_SIZE && pix_x <= 210+STAR_SIZE) && (pix_y >= (GROUND_Y-178)-STAR_SIZE && pix_y <= (GROUND_Y-178)+STAR_SIZE)) ||
        (((pix_x-330) == (pix_y - (GROUND_Y-120))) && (pix_x >= 330-STAR_SIZE && pix_x <= 330+STAR_SIZE) && (pix_y >= (GROUND_Y-120)-STAR_SIZE && pix_y <= (GROUND_Y-120)+STAR_SIZE)) ||
        (((pix_x-330) ==-(pix_y - (GROUND_Y-120))) && (pix_x >= 330-STAR_SIZE && pix_x <= 330+STAR_SIZE) && (pix_y >= (GROUND_Y-120)-STAR_SIZE && pix_y <= (GROUND_Y-120)+STAR_SIZE)) ||
        (((pix_x-390) == (pix_y - (GROUND_Y-148))) && (pix_x >= 390-STAR_SIZE && pix_x <= 390+STAR_SIZE) && (pix_y >= (GROUND_Y-148)-STAR_SIZE && pix_y <= (GROUND_Y-148)+STAR_SIZE)) ||
        (((pix_x-390) ==-(pix_y - (GROUND_Y-148))) && (pix_x >= 390-STAR_SIZE && pix_x <= 390+STAR_SIZE) && (pix_y >= (GROUND_Y-148)-STAR_SIZE && pix_y <= (GROUND_Y-148)+STAR_SIZE)) ||
        (((pix_x-480) == (pix_y - (GROUND_Y-182))) && (pix_x >= 480-STAR_SIZE && pix_x <= 480+STAR_SIZE) && (pix_y >= (GROUND_Y-182)-STAR_SIZE && pix_y <= (GROUND_Y-182)+STAR_SIZE)) ||
        (((pix_x-480) ==-(pix_y - (GROUND_Y-182))) && (pix_x >= 480-STAR_SIZE && pix_x <= 480+STAR_SIZE) && (pix_y >= (GROUND_Y-182)-STAR_SIZE && pix_y <= (GROUND_Y-182)+STAR_SIZE)) ||
        (((pix_x-530) == (pix_y - (GROUND_Y-125))) && (pix_x >= 530-STAR_SIZE && pix_x <= 530+STAR_SIZE) && (pix_y >= (GROUND_Y-125)-STAR_SIZE && pix_y <= (GROUND_Y-125)+STAR_SIZE)) ||
        (((pix_x-530) ==-(pix_y - (GROUND_Y-125))) && (pix_x >= 530-STAR_SIZE && pix_x <= 530+STAR_SIZE) && (pix_y >= (GROUND_Y-125)-STAR_SIZE && pix_y <= (GROUND_Y-125)+STAR_SIZE)) ||
        (((pix_x-605) == (pix_y - (GROUND_Y-110))) && (pix_x >= 605-STAR_SIZE && pix_x <= 605+STAR_SIZE) && (pix_y >= (GROUND_Y-110)-STAR_SIZE && pix_y <= (GROUND_Y-110)+STAR_SIZE)) ||
        (((pix_x-605) ==-(pix_y - (GROUND_Y-110))) && (pix_x >= 605-STAR_SIZE && pix_x <= 605+STAR_SIZE) && (pix_y >= (GROUND_Y-110)-STAR_SIZE && pix_y <= (GROUND_Y-110)+STAR_SIZE));

    reg star_toggle;
    always @(posedge vsync or negedge rst_n)
        if (!rst_n) star_toggle <= 0;
        else        star_toggle <= ~star_toggle;

    wire is_star = star_toggle ? is_star_plus : is_star_cross;

    //---------------------- Final Output ------------------------------
    assign R = (!video_active)       ? 2'b00 :
               (is_ground_line && start)     ? 2'b11 :
               (is_ground_dot && start)      ? 2'b11 :
               (is_cloud && start)           ? 2'b11 :
               (is_star && start)            ? 2'b11 :
               2'b00;
    assign G = R;
    assign B = R;

endmodule
