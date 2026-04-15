`timescale 1ns/1ps
module zedpp_pe_ultra_optimized #(
    parameter N         = 8,
    parameter BLOCK     = 4,
    parameter PARALLEL  = 4  // 1, 2, or 4
)(
    input  clk,
    input  rst,
    input  start,

    input              blk_valid,
    input  [1:0]       blk_idx,
    input  [BLOCK-1:0] blk_bitmap,
    input  [7:0]       blk_values [0:BLOCK-1],
    input              blk_last,

    input  [7:0] B_values [0:N-1],
    input  [N-1:0] B_bitmap,

    output reg [31:0] C,
    output reg done,

    output reg [31:0] cycle_cnt,
    output reg [31:0] mac_cnt,
    output reg [31:0] bit_scan_cnt,
    output reg [31:0] block_skip_cnt,
    output reg [31:0] bit_skip_cnt
);

    localparam IDLE   = 0,
               ACTIVE = 1;

    reg state;
    reg started;
    reg block_captured;  // NEW: Prevent re-capturing same block
    
    /* Working registers */
    reg [BLOCK-1:0]  work_bitmap;
    reg [7:0]        work_values [0:BLOCK-1];
    reg [1:0]        work_blk_idx;
    reg              work_last;
    
    /* Scan position */
    reg [$clog2(BLOCK)-1:0] bit_idx;
    
    integer i;

    /* Combinational logic */
    wire [$clog2(N)-1:0] abs_idx;
    assign abs_idx = work_blk_idx * BLOCK + bit_idx;

    wire [$clog2(N)-1:0] abs_idx_p1;
    assign abs_idx_p1 = work_blk_idx * BLOCK + bit_idx + 1;

    wire block_empty;
    assign block_empty = (work_bitmap == {BLOCK{1'b0}});

    wire [BLOCK-1:0] B_block_bitmap;
    assign B_block_bitmap = B_bitmap[work_blk_idx*BLOCK +: BLOCK];
    
    wire [BLOCK-1:0] intersection_bitmap;
    assign intersection_bitmap = work_bitmap & B_block_bitmap;

    wire intersection_empty;
    assign intersection_empty = (intersection_bitmap == {BLOCK{1'b0}});
    
    wire should_skip_block;
    assign should_skip_block = block_empty || intersection_empty;

    /* Parallel computation wires */
    wire [31:0] mac_result_p4;
    wire [3:0]  mac_count_p4;
    
    wire [31:0] mac_result_p2;
    wire [2:0]  mac_count_p2;
    wire [1:0]  scan_count_p2;
    
    // Parallel computation for PARALLEL=4
    wire [31:0] mac0 = intersection_bitmap[0] ? (work_values[0] * B_values[work_blk_idx*BLOCK + 0]) : 32'd0;
    wire [31:0] mac1 = intersection_bitmap[1] ? (work_values[1] * B_values[work_blk_idx*BLOCK + 1]) : 32'd0;
    wire [31:0] mac2 = intersection_bitmap[2] ? (work_values[2] * B_values[work_blk_idx*BLOCK + 2]) : 32'd0;
    wire [31:0] mac3 = intersection_bitmap[3] ? (work_values[3] * B_values[work_blk_idx*BLOCK + 3]) : 32'd0;
    
    assign mac_result_p4 = mac0 + mac1 + mac2 + mac3;
    assign mac_count_p4 = {3'b0, intersection_bitmap[0]} + 
                         {3'b0, intersection_bitmap[1]} + 
                         {3'b0, intersection_bitmap[2]} + 
                         {3'b0, intersection_bitmap[3]};
    
    // Parallel computation for PARALLEL=2
    wire [31:0] mac_bit0 = (bit_idx < BLOCK && intersection_bitmap[bit_idx]) ? 
                           (work_values[bit_idx] * B_values[abs_idx]) : 32'd0;
    wire [31:0] mac_bit1 = (bit_idx + 1 < BLOCK && intersection_bitmap[bit_idx+1]) ? 
                           (work_values[bit_idx+1] * B_values[abs_idx_p1]) : 32'd0;
    
    wire hit0 = (bit_idx < BLOCK && intersection_bitmap[bit_idx]);
    wire hit1 = (bit_idx + 1 < BLOCK && intersection_bitmap[bit_idx+1]);
    
    assign mac_result_p2 = mac_bit0 + mac_bit1;
    assign mac_count_p2 = {2'b0, hit0} + {2'b0, hit1};
    assign scan_count_p2 = (bit_idx < BLOCK ? 2'd1 : 2'd0) + 
                           (bit_idx + 1 < BLOCK ? 2'd1 : 2'd0);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            C <= 0;
            done <= 0;
            cycle_cnt <= 0;
            mac_cnt <= 0;
            bit_scan_cnt <= 0;
            block_skip_cnt <= 0;
            bit_skip_cnt <= 0;
            bit_idx <= 0;
            started <= 0;
            block_captured <= 0;
            
        end else begin
        
            case (state)
            
            IDLE: begin
                if (start) begin
                    C <= 0;
                    cycle_cnt <= 0;
                    mac_cnt <= 0;
                    bit_scan_cnt <= 0;
                    block_skip_cnt <= 0;
                    bit_skip_cnt <= 0;
                    done <= 0;
                    started <= 1;
                    block_captured <= 0;  // Reset on start
                end
                
                // Capture next block when available (but only once per block!)
                if (blk_valid && started && !block_captured) begin
                    work_bitmap <= blk_bitmap;
                    work_blk_idx <= blk_idx;
                    work_last <= blk_last;
                    for (i = 0; i < BLOCK; i = i + 1)
                        work_values[i] <= blk_values[i];
                    
                    bit_idx <= 0;
                    block_captured <= 1;  // Mark as captured
                    state <= ACTIVE;
                end
                
                // Reset captured flag when blk_valid goes low
                if (!blk_valid) begin
                    block_captured <= 0;
                end
            end
            
            ACTIVE: begin
                cycle_cnt <= cycle_cnt + 1;
                
                if (should_skip_block) begin
                    // Skip entire block
                    block_skip_cnt <= block_skip_cnt + 1;
                    
                    if (work_last) begin
                        done <= 1;
                        started <= 0;
                        state <= IDLE;
                    end else begin
                        state <= IDLE;
                    end
                    
                end else begin
                    // Process block based on parallelism
                    if (PARALLEL == 4) begin
                        // Process all 4 bits in 1 cycle
                        C <= C + mac_result_p4;
                        mac_cnt <= mac_cnt + mac_count_p4;
                        bit_scan_cnt <= bit_scan_cnt + mac_count_p4;
                        
                        if (work_last) begin
                            done <= 1;
                            started <= 0;
                            state <= IDLE;
                        end else begin
                            state <= IDLE;
                        end
                        
                    end else if (PARALLEL == 2) begin
                        // Process 2 bits per cycle
                        C <= C + mac_result_p2;
                        mac_cnt <= mac_cnt + mac_count_p2;
                        bit_scan_cnt <= bit_scan_cnt + scan_count_p2;
                        
                        // Check if we've processed all bits
                        if (bit_idx >= BLOCK - 2) begin
                            if (work_last) begin
                                done <= 1;
                                started <= 0;
                                state <= IDLE;
                            end else begin
                                state <= IDLE;
                            end
                        end else begin
                            bit_idx <= bit_idx + 2;
                            // Stay in ACTIVE state
                        end
                        
                    end else begin
                        // PARALLEL = 1: Process 1 bit per cycle
                        bit_scan_cnt <= bit_scan_cnt + 1;
                        
                        if (intersection_bitmap[bit_idx]) begin
                            C <= C + work_values[bit_idx] * B_values[abs_idx];
                            mac_cnt <= mac_cnt + 1;
                        end else begin
                            bit_skip_cnt <= bit_skip_cnt + 1;
                        end
                        
                        // Check if we've processed all bits
                        if (bit_idx == BLOCK - 1) begin
                            if (work_last) begin
                                done <= 1;
                                started <= 0;
                                state <= IDLE;
                            end else begin
                                state <= IDLE;
                            end
                        end else begin
                            bit_idx <= bit_idx + 1;
                            // Stay in ACTIVE state
                        end
                    end
                end
            end
            
            endcase
        end
    end
endmodule