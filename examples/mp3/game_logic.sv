// sets initial gameboard using start_board from memory module
// controller gives pixel to tell game_logic what pixel to calculate
// always (posedge clk) calculating next_board[pixel] using current_board[pixel] and n and m, calcuated from pixel
// always (posedge clk) read_data = current_board[pixel] (choose some 8-bit representation)
// always (posedge update_board) current_board = next_board (pixel by pixel?)

// try alive_neighbour as binary not int

module game_logic #(
    parameter INIT_FILE = ""
)(
    input logic clk, 
    input logic [5:0] pixel,
    input logic update_board,
    input logic transmit_pixel,
    output logic [7:0] pixel_data // shift register takes in pixel data as 8-bit
);

    // memory
    logic [7:0] current_board [0:63]; // 64 8-bit values
    logic [7:0] next_board [0:63];

    initial if (INIT_FILE) begin
        $readmemh(INIT_FILE, current_board);
    end



    localparam [7:0] alive = 8'b01111111; // 127/254
    localparam [7:0] dead = 8'b00000000;

    // pixel row and column
    int col;
    int row;

    int alive_neighbours; // alive_neighbours needs to hold 8 values max
    localparam int matrix_len = 8; // 8 pixel-long LED matrix

/*
    // calculate pixel for the next board based on its neighbours for the current board
    always_ff @(negedge transmit_pixel) begin

        alive_neighbours = 0;

        //find row and column for this pixel
        col = int'(pixel[5:3]); // col is second half
        row = int'(pixel[2:0]); // row is first half


        // go through all neighbours and count number of alive neighbours
        if (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len]) begin // top left
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + col]) begin // top middle
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+1)%matrix_len]) begin // top right
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*row + (col-1+matrix_len)%matrix_len]) begin // middle left
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*row + (col+1)%matrix_len]) begin // middle right
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*((row+1)%matrix_len) + (col-1+matrix_len)%matrix_len]) begin // bottom left
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*((row+1)%matrix_len) + col]) begin // bottom middle
            alive_neighbours = alive_neighbours + 1;
        end
        if (current_board[matrix_len*((row+1)%matrix_len) + (col+1)%matrix_len]) begin // bottom right
            alive_neighbours = alive_neighbours + 1;
        end
        

        // deciding whether next state should be dead or alive
        if (current_board[pixel]) begin // if current_board[pixel] is alive
            if (alive_neighbours < 2) begin // living cell with fewer than 2 living neighbours dies
                next_board[pixel] <= dead;
            end
            if (alive_neighbours >= 2 && alive_neighbours <= 3) begin // living cell with 2-3 living neighbours lives
                next_board[pixel] <= alive;
            end
            if (alive_neighbours > 3) begin // living cell with more than 3 living neighbours dies
                next_board[pixel] <= dead;
            end
        end

        if (!current_board[pixel]) begin // if current_board[pixel] is dead
            if (alive_neighbours == 3) begin // dead cell with 3 living neighbours becomes alive
                next_board[pixel] <= alive;
            end else begin
                next_board[pixel] <= dead;
            end
        end

    end

*/

// attempt #2 counts neighbours correctly
    always_ff @(negedge transmit_pixel) begin
        alive_neighbours <= 0; // reset number of pixels everytime pixel changes
        //find row and column for this pixel
        col <= int'(pixel[5:3]); // col is second half
        row <= int'(pixel[2:0]); // row is first half
    end


/*
    genvar icol;
    genvar irow;
    generate
        for (irow=-1; irow<2; irow++) begin
            for (icol=-1; icol<2; icol++) begin
                always_ff @(posedge transmit_pixel) begin // checking neighbours
                    if (current_board[matrix_len*((row+irow+matrix_len)%matrix_len) + (col+icol+matrix_len)%matrix_len]) begin
                        if (!(irow==0 && icol==0)) begin // don't count itself as a neighbour
                            alive_neighbours++;
                        end
                    end
                end
            end
        end
    endgenerate
    */

    always_comb begin
        alive_neighbours = (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row-1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col-1+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+matrix_len)%matrix_len] > 0) + (current_board[matrix_len*((row+1+matrix_len)%matrix_len) + (col+1+matrix_len)%matrix_len] > 0);

        // deciding whether next state should be dead or alive
        if (current_board[pixel]) begin // if current_board[pixel] is alive
            if (alive_neighbours < 2) begin // living cell with fewer than 2 living neighbours dies
                next_board[pixel] = dead;
            end
            if (alive_neighbours >= 2 && alive_neighbours <= 3) begin // living cell with 2-3 living neighbours lives
                next_board[pixel] = alive;
            end
            if (alive_neighbours > 3) begin // living cell with more than 3 living neighbours dies
                next_board[pixel] = dead;
            end
        end

        if (!current_board[pixel]) begin // if current_board[pixel] is dead
            if (alive_neighbours == 3) begin // dead cell with 3 living neighbours becomes alive
                next_board[pixel] = alive;
            end else begin
                next_board[pixel] = dead;
            end
        end
    end


    // assign next board to current board when it's time to update
    /*
    genvar i;
    generate
        for (i=0; i<64; i++) begin
            always_ff @(posedge update_board) begin
                current_board[i] <= next_board[i];
                next_board[i] <= 8'bxxxxxxxx;
            end
            
            //current_board <= next_board;
            //next_board <= 8'bxxxxxxxx; // reset board to unspecified, so we know it's working correctly
        end
    endgenerate
    */



    always_ff @(posedge clk) begin
        pixel_data = current_board[pixel];
    end


endmodule