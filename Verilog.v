module CPU(
    input clk,
    input rst,
    input [18:0] data_in,       
    output reg [18:0] address,  
    output reg [18:0] data_out,  
    output reg mem_read,        
    output reg mem_write        
);

    // Register file
    reg [18:0] registers [0:7];  
    reg [18:0] PC;                
    reg [18:0] SP;                
    reg [18:0] ALU_result;        

    ///////// Instruction encoding //////////////////
    reg [18:0] instruction;       

    ////////// ALU operation codes /////////////
    localparam ALU_ADD = 3'b000,
               ALU_SUB = 3'b001,
               ALU_MUL = 3'b010,
               ALU_DIV = 3'b011,
               ALU_AND = 3'b100,
               ALU_OR  = 3'b101,
               ALU_XOR = 3'b110,
               ALU_NOT = 3'b111;

    ///////// Main CPU operation ////////////
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC <= 0;
            SP <= 19'h1FFFF; // Example stack pointer initialization
            mem_read <= 0;
            mem_write <= 0;
            // Reset registers
            integer i;
            for (i = 0; i < 8; i = i + 1) begin
                registers[i] <= 0;
            end
        end else begin
            //////////// Fetch instruction ////////////
            address <= PC;
            mem_read <= 1;
            instruction <= data_in;

            //////// Decode and execute instruction ////////////////
            case (instruction[18:15]) // Assume 4-bit opcode
                4'b0000: begin // ADD
                    registers[instruction[14:12]] <= registers[instruction[11:9]] + registers[instruction[8:6]];
                end
                4'b0001: begin // SUB
                    registers[instruction[14:12]] <= registers[instruction[11:9]] - registers[instruction[8:6]];
                end
                4'b0010: begin // MUL
                    registers[instruction[14:12]] <= registers[instruction[11:9]] * registers[instruction[8:6]];
                end
                4'b0011: begin // DIV
                    registers[instruction[14:12]] <= registers[instruction[11:9]] / registers[instruction[8:6]];
                end
                4'b0100: begin // INC
                    registers[instruction[14:12]] <= registers[instruction[14:12]] + 1;
                end
                4'b0101: begin // DEC
                    registers[instruction[14:12]] <= registers[instruction[14:12]] - 1;
                end
                4'b0110: begin // AND
                    registers[instruction[14:12]] <= registers[instruction[11:9]] & registers[instruction[8:6]];
                end
                4'b0111: begin // OR
                    registers[instruction[14:12]] <= registers[instruction[11:9]] | registers[instruction[8:6]];
                end
                4'b1000: begin // XOR
                    registers[instruction[14:12]] <= registers[instruction[11:9]] ^ registers[instruction[8:6]];
                end
                4'b1001: begin // NOT
                    registers[instruction[14:12]] <= ~registers[instruction[11:9]];
                end
                4'b1010: begin // JMP
                    PC <= instruction[14:0]; // 15-bit address
                end
                4'b1011: begin // BEQ
                    if (registers[instruction[14:12]] == registers[instruction[11:9]])
                        PC <= instruction[8:0]; // 9-bit address
                end
                4'b1100: begin // BNE
                    if (registers[instruction[14:12]] != registers[instruction[11:9]])
                        PC <= instruction[8:0]; // 9-bit address
                end
                4'b1101: begin // CALL
                    registers[7] <= PC; 
                    SP <= SP - 1;
                    PC <= instruction[14:0]; 
                end
                4'b1110: begin 
                    PC <= registers[SP]; 
                    SP <= SP + 1;
                end
                4'b1111: begin 
                    if (instruction[14:12] == 3'b000) begin 
                        registers[instruction[11:9]] <= data_in; 
                    end else if (instruction[14:12] == 3'b001) begin 
                        data_out <= registers[instruction[11:9]]; 
                        address <= instruction[8:0]; 
                        mem_write <= 1; 
                    end
                end
                default: begin
                    ////////////////// Invalid instruction handling ////////////////////
                end
            endcase

            ////////// Update PC for next instruction  //////////////////////
            PC <= PC + 1;
            mem_read <= 0;
            mem_write <= 0;     /////// Reset memory control signals
        end
    end

endmodule
