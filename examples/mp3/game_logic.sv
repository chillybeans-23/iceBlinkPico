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

    int alive_neighbours;
    localparam logic [3:0] matrix_len = 4'b1000; // 8 pixel-long LED matrix

    logic [2:0] top_row;
    logic [2:0] bot_row;
    logic [2:0] left_col;
    logic [2:0] right_col;

    always_ff @(posedge clk) begin
        row <= pixel[5:3]; // row is MSB half
        col <= pixel[2:0]; // col is LSB half

        top_row <= (row == 0) ? 7 : row-1; // if top row, row-1 = 7 (bottom row). else, row-1
        bot_row <= (row == 7) ? 0 : row+1; // if bottom row, row+1 = 0 (top row). else, row+1
        left_col <= (col == 0) ? 7 : col-1; // if left col, col-1 = 7 (right col). else, col-1
        right_col <= (col == 7) ? 0 : col+1; // if right col, col+1 = 0 (left col). else, col+1

        alive_neighbours <= 
        (current_board[{top_row, left_col}]) 
        + (current_board[{top_row, col}])
        + (current_board[{top_row, right_col}])
        + (current_board[{row, left_col}])
        + (current_board[{row, right_col}])
        + (current_board[{bot_row, left_col}])
        + (current_board[{bot_row, col}])
        + (current_board[{bot_row, right_col}]);
        

        if (update_board) begin
            for (int i=0; i<64; i++) begin
                current_board[i] <= next_board[i];
            end
        end

        // deciding whether next state should be dead or alive
        if (current_board[pixel]) begin // if current_board[pixel] is alive
            next_board[pixel] <= (alive_neighbours == 2 || alive_neighbours == 3) ? 1 : 0; // alive if 2 or 3 alive neighbours. else dead.
        end else begin // if current_board[pixel] is dead
            next_board[pixel] <= (alive_neighbours == 3) ? 1 : 0; // dead cell with 3 living neighbours becomes alive
        end
    end
    

    always_ff @(negedge transmit_pixel) begin
        pixel_data <= (current_board[pixel]) ? alive : dead;
    end
    


endmodule