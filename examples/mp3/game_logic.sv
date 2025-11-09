// sets initial gameboard using start_board from memory module
// controller gives pixel to tell game_logic what pixel to calculate
// always (posedge clk) calculating next_board[pixel] using current_board[pixel] and n and m, calcuated from pixel
// always (posedge clk) read_data = current_board[pixel] (choose some 8-bit representation)
// always (posedge update_board) current_board = next_board (pixel by pixel?)


module game_logic #(
    parameter INIT_FILE = ""
)(
    input logic clk, 
    input logic [5:0] pixel,
    input logic update_board,
    input logic transmit_pixel,
    output logic [7:0] pixel_data // shift register takes in pixel data as 8-bit
);

    logic current_board [63:0];// 64 1-bit values. unpacked array
    logic next_board [63:0];

    // memory
    initial if (INIT_FILE) begin
        $readmemh(INIT_FILE, current_board);
    end



    localparam [7:0] alive = 8'b01111111; // 127/254
    localparam [7:0] dead = 8'b00000000;

    // pixel row and column
    logic [2:0] col;
    logic [2:0] row;

    int alive_neighbours; // alive_neighbours needs to hold 8 values max
    localparam int matrix_len = 8; // 8 pixel-long LED matrix


// WORKS IN SIM
    always_ff @(posedge update_board) begin
        for (int i=0; i<64; i++) begin
            current_board[i] <= next_board[i];
        end
    end
        
    always_ff @(posedge clk) begin

        row = pixel[5:3]; // row is MSB half
        col = pixel[2:0]; // col is LSB half

        //row = 6'b111000 & pixel; // row is MSB half
        //col = 6'b000111 & pixel; // col is LSB half

        alive_neighbours = (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0);
        //alive_neighbours = (current_board[(row-8)& 6'b111000 + (col-1)& 6'b000111] > 0) + (current_board[(row-8)& 6'b111000 + col& 6'b000111] > 0) + (current_board[(row-8)& 6'b111000 + (col+1)& 6'b000111] > 0) + (current_board[row& 6'b111000 + (col-1)& 6'b000111] > 0) + (current_board[row& 6'b111000 + (col+1)& 6'b000111] > 0) + (current_board[(row+8)& 6'b111000 + (col-1)& 6'b000111] > 0) + (current_board[(row+8)& 6'b111000 + col& 6'b000111] > 0) + (current_board[(row+8)& 6'b111000 + (col+1)& 6'b000111] > 0);

        // deciding whether next state should be dead or alive
        if (current_board[pixel]) begin // if current_board[pixel] is alive
            next_board[pixel] = (alive_neighbours == 2 || alive_neighbours == 3) ? 1 : 0; // alive if 2 or 3 alive neighbours. else dead.
        end else begin // if current_board[pixel] is dead
            next_board[pixel] = (alive_neighbours == 3) ? 1 : 0; // dead cell with 3 living neighbours becomes alive
        end
    end
    

// update all at update_board, instead of incrementally with clk. no counting neighbours! get stuck at: Current simulation time is 985336 ticks.
/*
    always_ff @(posedge update_board) begin
        for (logic [5:0] i=6'b000000; i<=6'b111111; i++) begin

            row = i[5:3]; // row is MSB half
            col = i[2:0]; // col is LSB half
            alive_neighbours = (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0);
                

            // deciding whether next state should be dead or alive
            if (current_board[i]) begin // if current_board[pixel] is alive
                next_board[i] = (alive_neighbours == 2 || alive_neighbours == 3) ? alive : dead; // alive if 2 or 3 alive neighbours. else dead.
            end else begin
                next_board[i] = (alive_neighbours == 3) ? alive : dead; // dead cell with 3 living neighbours becomes alive
            end
        end

        for (int j=0; j<64; j++) begin
            current_board[j] = next_board[j];
        end
    end
    */


    // assign next board to current board when it's time to update
    /*
    genvar i;
    generate
        for (i=0; i<64; i++) begin
            always_ff @(posedge update_board) begin
                current_board[i] <= next_board[i];
                //next_board[i] <= 8'bxxxxxxxx; // removed this so only one always_ff assigns next_board
            end
            
            //current_board <= next_board;
            //next_board <= 8'bxxxxxxxx; // reset board to unspecified, so we know it's working correctly
        end
    endgenerate
    */
    
/*
    always_ff @(posedge clk) begin
        pixel_data = current_board[pixel];
    end
    */
    assign pixel_data = (current_board[pixel]) ? alive : dead;
    


endmodule