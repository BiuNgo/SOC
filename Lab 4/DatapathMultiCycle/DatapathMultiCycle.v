/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31

// RV opcodes are 7 bits
`define OPCODE_SIZE 6

`ifndef DIVIDER_STAGES
  `define DIVIDER_STAGES 10
`endif

// Don't forget your CLA and Divider
`include "cla.v"
`include "DividerUnsignedPipelined.v"

module RegFile (
  input      [        4:0] rd,
  input      [`REG_SIZE:0] rd_data,
  input      [        4:0] rs1,
  output reg [`REG_SIZE:0] rs1_data,
  input      [        4:0] rs2,
  output reg [`REG_SIZE:0] rs2_data,
  input                    clk,
  input                    we,
  input                    rst
);

  // TODO: copy your homework #3 code here
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs[0:NumRegs-1];

  integer i;
  initial begin
    for (i = 0; i < NumRegs; i = i + 1) begin
      regs[i] = 32'd0;
    end
  end

  always @(*) begin
    rs1_data = (rs1 == 5'd0) ? 32'd0 : regs[rs1];
    rs2_data = (rs2 == 5'd0) ? 32'd0 : regs[rs2];
  end

  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) begin
        regs[i] <= 32'd0;
      end
    end else if (we && rd != 5'd0) begin
      regs[rd] <= rd_data;
    end
  end

endmodule

module DatapathMultiCycle (
    input                    clk,
    input                    rst,
    output reg               halt,
    output     [`REG_SIZE:0] pc_to_imem,
    input      [`REG_SIZE:0] inst_from_imem,
    // addr_to_dmem is a read-write port
    output reg [`REG_SIZE:0] addr_to_dmem,
    input      [`REG_SIZE:0] load_data_from_dmem,
    output reg [`REG_SIZE:0] store_data_to_dmem,
    output reg [        3:0] store_we_to_dmem
);

  // TODO: your code here (largely based on homework #3)
  // components of the instruction
  wire [           6:0] inst_funct7;
  wire [           4:0] inst_rs2;
  wire [           4:0] inst_rs1;
  wire [           2:0] inst_funct3;
  wire [           4:0] inst_rd;
  wire [`OPCODE_SIZE:0] inst_opcode;

  // split R-type instruction
  assign {inst_funct7, inst_rs2, inst_rs1, inst_funct3, inst_rd, inst_opcode} = inst_from_imem;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = inst_from_imem[31:20];
  wire [ 4:0] imm_shamt = inst_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s = {inst_funct7, inst_rd};
  
  // B - conditionals
  wire [12:0] imm_b;
  assign imm_b = {inst_from_imem[31], inst_from_imem[7], inst_from_imem[30:25], inst_from_imem[11:8], 1'b0};

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {inst_from_imem[31:12], 1'b0};

  wire [`REG_SIZE:0] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE:0] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE:0] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE:0] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};
  
  // U-Type
  wire [`REG_SIZE:0] imm_u_val  = {inst_from_imem[31:12], 12'b0};
  
  // opcodes
  localparam [`OPCODE_SIZE:0] OpLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpJalr    = 7'b11_001_11;
  localparam [`OPCODE_SIZE:0] OpMiscMem = 7'b00_011_11;
  localparam [`OPCODE_SIZE:0] OpJal     = 7'b11_011_11;

  localparam [`OPCODE_SIZE:0] OpRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpEnviron = 7'b11_100_11;

  localparam [`OPCODE_SIZE:0] OpAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpLui     = 7'b01_101_11;

  wire inst_lui    = (inst_opcode == OpLui    );
  wire inst_auipc  = (inst_opcode == OpAuipc  );
  wire inst_jal    = (inst_opcode == OpJal    );
  wire inst_jalr   = (inst_opcode == OpJalr   );
  //
  wire inst_branch = (inst_opcode == OpBranch );

  wire inst_beq    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_bne    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_blt    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_bge    = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b101);
  wire inst_bltu   = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b110);
  wire inst_bgeu   = (inst_opcode == OpBranch ) & (inst_from_imem[14:12] == 3'b111);

  wire inst_lb     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_lh     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_lw     = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b010);
  wire inst_lbu    = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_lhu    = (inst_opcode == OpLoad   ) & (inst_from_imem[14:12] == 3'b101);

  wire inst_sb     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_sh     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b001);
  wire inst_sw     = (inst_opcode == OpStore  ) & (inst_from_imem[14:12] == 3'b010);

  wire inst_addi   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b000);
  wire inst_slti   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b010);
  wire inst_sltiu  = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b011);
  wire inst_xori   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b100);
  wire inst_ori    = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b110);
  wire inst_andi   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b111);

  wire inst_slli   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b001) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srli   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srai   = (inst_opcode == OpRegImm ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'b0100000);

  wire inst_add    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b000) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sub    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b000) & (inst_from_imem[31:25] == 7'b0100000);
  wire inst_sll    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b001) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_slt    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b010) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sltu   = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b011) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_xor    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b100) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_srl    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_sra    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b101) & (inst_from_imem[31:25] == 7'b0100000);
  wire inst_or     = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b110) & (inst_from_imem[31:25] == 7'd0      );
  wire inst_and    = (inst_opcode == OpRegReg ) & (inst_from_imem[14:12] == 3'b111) & (inst_from_imem[31:25] == 7'd0      );

  wire inst_mul    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b000    );
  wire inst_mulh   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b001    );
  wire inst_mulhsu = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b010    );
  wire inst_mulhu  = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b011    );
  wire inst_div    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b100    );
  wire inst_divu   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b101    );
  wire inst_rem    = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b110    );
  wire inst_remu   = (inst_opcode == OpRegReg ) & (inst_from_imem[31:25] == 7'd1  ) & (inst_from_imem[14:12] == 3'b111    );

  wire inst_ecall  = (inst_opcode == OpEnviron) & (inst_from_imem[31:7] == 25'd0  );
  wire inst_fence  = (inst_opcode == OpMiscMem);
  
  wire is_div_op = inst_div || inst_divu || inst_rem || inst_remu;
  
  reg [$clog2(`DIVIDER_STAGES):0] div_cycle_counter;
  
  always @(posedge clk) begin
  if (rst) begin
      div_cycle_counter <= 0;
  end else begin
      if (is_div_op) begin
          if (div_cycle_counter < `DIVIDER_STAGES) begin
              div_cycle_counter <= div_cycle_counter + 1'b1;
          end else begin
              div_cycle_counter <= 0;
          end
      end else begin
          div_cycle_counter <= 0;
      end
  end
end
  
  wire div_stall = is_div_op && (div_cycle_counter < `DIVIDER_STAGES);
  
  //regfile
  wire [`REG_SIZE:0] rs1_data;
  wire [`REG_SIZE:0] rs2_data;
  reg [`REG_SIZE:0] rd_data;
  reg rf_we;
  
  RegFile rf (
    .clk      (clk),
    .rst      (rst),
    .we       (rf_we),
    .rd       (inst_rd),
    .rd_data  (rd_data),
    .rs1      (inst_rs1),
    .rs2      (inst_rs2),
    .rs1_data (rs1_data),
    .rs2_data (rs2_data)
  );
  
  //pc
  reg [`REG_SIZE:0] pcNext, pcCurrent;
  reg [63:0] cycles_current;
  
  wire [`REG_SIZE:0] pc_plus_4 = pcCurrent + 32'd4;
  wire [`REG_SIZE:0] pc_branch_target = pcCurrent + imm_b_sext;
  wire [`REG_SIZE:0] pc_jump_jal      = pcCurrent + imm_j_sext;
  wire [`REG_SIZE:0] pc_jump_jalr     = (rs1_data + imm_i_sext) & ~32'd1;
  
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;
  
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 64'd0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end
  
  //alu
  wire [`REG_SIZE:0] alu_op1 = rs1_data;
  reg [`REG_SIZE:0] alu_op2;
  
  always @(*) begin
    if (inst_opcode ==  OpRegImm) alu_op2 = imm_i_sext;
    else alu_op2 = rs2_data;
  end
  
  //add, sub
  wire cla_sub_mode = inst_sub;
  wire [31:0] cla_op2 = cla_sub_mode ? ~alu_op2 : alu_op2;
  wire cla_cin = cla_sub_mode ? 1'b1 : 1'b0;
  wire [31:0] cla_result;
  
  cla u_cla (
    .a(alu_op1),
    .b(cla_op2),
    .cin(cla_cin),
    .sum(cla_result)
  );
  
  //mul
  wire [63:0] mul_full_result = {{32{alu_op1[31]}}, alu_op1} * {{32{alu_op2[31]}}, alu_op2};
  wire [63:0] mul_full_result_u = {32'b0, alu_op1} * {32'b0, alu_op2};
  wire [63:0] mul_full_result_su = {{32{alu_op1[31]}}, alu_op1} * {32'b0, alu_op2};
  
  //pipelined div
  wire [31:0] neg_alu_op1 = ~alu_op1 + 1'b1;
  wire [31:0] neg_alu_op2 = ~alu_op2 + 1'b1;
  
  wire is_signed_div_rem = inst_div || inst_rem;
  
  wire [31:0] div_dividend = is_signed_div_rem && alu_op1[31] ? neg_alu_op1 : alu_op1;
  wire [31:0] div_divisor  = is_signed_div_rem && alu_op2[31] ? neg_alu_op2 : alu_op2;
  
  wire [31:0] div_quotient_u, div_remainder_u;
  
  DividerUnsignedPipelined u_div_pipe (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .i_dividend(div_dividend),
    .i_divisor(div_divisor),
    .o_quotient(div_quotient_u),
    .o_remainder(div_remainder_u)
  );
  
  reg [31:0] div_result;
  reg [31:0] rem_result;
  
  wire [31:0] neg_quotient = ~div_quotient_u + 1'b1;
  wire [31:0] neg_remainder = ~div_remainder_u + 1'b1;
  
  wire is_div_by_zero = (alu_op2 == 32'd0);
  wire is_overflow    = (inst_div || inst_rem) && (alu_op1 == 32'h80000000) && (alu_op2 == 32'hFFFFFFFF);
  
  always @(*) begin
    div_result = div_quotient_u;
    rem_result = div_remainder_u;
    
    if (is_div_by_zero) begin
      div_result = 32'hFFFFFFFF;
      rem_result = alu_op1;
    end else if (is_overflow) begin
      div_result = 32'h80000000;
      rem_result = 32'd0;
    end else begin
      if (inst_div) begin
        if (alu_op1[31] != alu_op2[31]) begin
          div_result = neg_quotient;
        end
      end
    
      if (inst_rem) begin
        if (alu_op1[31]) begin
          rem_result = neg_remainder;
        end
      end
    end
  end
  
  //branch
  reg branch_taken;
  always @(*) begin
    case (1'b1)
        inst_beq : branch_taken = (rs1_data == rs2_data);
        inst_bne : branch_taken = (rs1_data != rs2_data);
        inst_blt : branch_taken = ($signed(rs1_data) < $signed(rs2_data));
        inst_bge : branch_taken = ($signed(rs1_data) >= $signed(rs2_data));
        inst_bltu: branch_taken = (rs1_data < rs2_data);
        inst_bgeu: branch_taken = (rs1_data >= rs2_data);
        default  : branch_taken = 1'b0;
    endcase
  end
  
  always @(*) begin
    addr_to_dmem = rs1_data + imm_s_sext;
    if (inst_opcode == OpLoad) addr_to_dmem = rs1_data + imm_i_sext;
  end
  
  //store
  always @(*) begin
    store_we_to_dmem = 4'b0000;
    store_data_to_dmem = 32'd0;
    
    if (inst_sb) begin
        case (addr_to_dmem[1:0])
            2'b00: begin store_we_to_dmem = 4'b0001; store_data_to_dmem[7:0]   = rs2_data[7:0]; end
            2'b01: begin store_we_to_dmem = 4'b0010; store_data_to_dmem[15:8]  = rs2_data[7:0]; end
            2'b10: begin store_we_to_dmem = 4'b0100; store_data_to_dmem[23:16] = rs2_data[7:0]; end
            2'b11: begin store_we_to_dmem = 4'b1000; store_data_to_dmem[31:24] = rs2_data[7:0]; end
        endcase
    end else if (inst_sh) begin
        case (addr_to_dmem[1:0])
            2'b00: begin store_we_to_dmem = 4'b0011; store_data_to_dmem[15:0]  = rs2_data[15:0]; end
            2'b10: begin store_we_to_dmem = 4'b1100; store_data_to_dmem[31:16] = rs2_data[15:0]; end
            default: begin end 
        endcase
    end else if (inst_sw) begin
        store_we_to_dmem = 4'b1111;
        store_data_to_dmem = rs2_data;
    end
  end
  
  //load
  reg [`REG_SIZE:0] load_val;
  always @(*) begin
    load_val = 32'd0;
    case (inst_funct3)
        3'b000: begin
            case (addr_to_dmem[1:0])
                2'b00: load_val = {{24{load_data_from_dmem[7]}},   load_data_from_dmem[7:0]};
                2'b01: load_val = {{24{load_data_from_dmem[15]}},  load_data_from_dmem[15:8]};
                2'b10: load_val = {{24{load_data_from_dmem[23]}},  load_data_from_dmem[23:16]};
                2'b11: load_val = {{24{load_data_from_dmem[31]}},  load_data_from_dmem[31:24]};
            endcase
        end
        3'b001: begin
            case (addr_to_dmem[1:0])
                2'b00: load_val = {{16{load_data_from_dmem[15]}},  load_data_from_dmem[15:0]};
                2'b10: load_val = {{16{load_data_from_dmem[31]}},  load_data_from_dmem[31:16]};
                default: load_val = 32'd0; 
            endcase
        end
        3'b010: begin
            load_val = load_data_from_dmem;
        end
        3'b100: begin
             case (addr_to_dmem[1:0])
                2'b00: load_val = {24'd0, load_data_from_dmem[7:0]};
                2'b01: load_val = {24'd0, load_data_from_dmem[15:8]};
                2'b10: load_val = {24'd0, load_data_from_dmem[23:16]};
                2'b11: load_val = {24'd0, load_data_from_dmem[31:24]};
            endcase
        end
        3'b101: begin
             case (addr_to_dmem[1:0])
                2'b00: load_val = {16'd0, load_data_from_dmem[15:0]};
                2'b10: load_val = {16'd0, load_data_from_dmem[31:16]};
                default: load_val = 32'd0;
            endcase
        end
        default: load_val = 32'd0;
    endcase
  end
  
  //control logic
  reg illegal_inst;
  
  always @(*) begin
    illegal_inst = 1'b0;
    rf_we = 1'b0;
    rd_data = 32'd0;
    halt = 1'b0;
    
    pcNext = pc_plus_4;
    
    if (div_stall) begin
        pcNext = pcCurrent;
    end else begin
        if (inst_branch && branch_taken) pcNext = pc_branch_target;
        else if (inst_jal) pcNext = pc_jump_jal;
        else if (inst_jalr) pcNext = pc_jump_jalr;
    end

    if (inst_ecall) begin
        halt = 1'b1;
    end
    
    if (inst_opcode == OpStore && addr_to_dmem == 32'h400) begin
        halt = 1'b1;
    end
    
    case (inst_opcode)
      OpLui: begin
        rf_we = 1'b1;
        rd_data = imm_u_val;
      end
      
      OpAuipc: begin
        rf_we = 1'b1;
        rd_data = pcCurrent + imm_u_val;
      end
      
      OpJal: begin
        rf_we = 1'b1;
        rd_data = pc_plus_4;
      end
      
      OpJalr: begin
        rf_we = 1'b1;
        rd_data = pc_plus_4;
      end
      
      OpBranch: begin
        // No RF Write
      end
      
      OpLoad: begin
        rf_we = 1'b1;
        rd_data = load_val;
      end
      
      OpStore: begin
        // No RF Write
      end
      
      OpRegImm: begin
        rf_we = 1'b1;
        if (inst_addi)      rd_data = cla_result;
        else if (inst_slti) rd_data = ($signed(rs1_data) < $signed(imm_i_sext)) ? 32'd1 : 32'd0;
        else if (inst_sltiu)rd_data = (rs1_data < imm_i_sext) ? 32'd1 : 32'd0;
        else if (inst_xori) rd_data = rs1_data ^ imm_i_sext;
        else if (inst_ori)  rd_data = rs1_data | imm_i_sext;
        else if (inst_andi) rd_data = rs1_data & imm_i_sext;
        else if (inst_slli) rd_data = rs1_data << imm_shamt;
        else if (inst_srli) rd_data = rs1_data >> imm_shamt;
        else if (inst_srai) rd_data = $signed(rs1_data) >>> imm_shamt;
      end
      
      OpRegReg: begin
        if (is_div_op) begin
            rf_we = !div_stall;
        end else begin
            rf_we = 1'b1;
        end

        if (inst_add || inst_sub) rd_data = cla_result;
        else if (inst_sll)  rd_data = rs1_data << rs2_data[4:0];
        else if (inst_slt)  rd_data = ($signed(rs1_data) < $signed(rs2_data)) ? 32'd1 : 32'd0;
        else if (inst_sltu) rd_data = (rs1_data < rs2_data) ? 32'd1 : 32'd0;
        else if (inst_xor)  rd_data = rs1_data ^ rs2_data;
        else if (inst_srl)  rd_data = rs1_data >> rs2_data[4:0];
        else if (inst_sra)  rd_data = $signed(rs1_data) >>> rs2_data[4:0];
        else if (inst_or)   rd_data = rs1_data | rs2_data;
        else if (inst_and)  rd_data = rs1_data & rs2_data;
        else if (inst_mul)    rd_data = mul_full_result[31:0];
        else if (inst_mulh)   rd_data = mul_full_result[63:32];
        else if (inst_mulhsu) rd_data = mul_full_result_su[63:32];
        else if (inst_mulhu)  rd_data = mul_full_result_u[63:32];
        else if (inst_div || inst_divu) rd_data = div_result;
        else if (inst_rem || inst_remu) rd_data = rem_result;
      end
      
      OpEnviron: begin
        if (inst_ecall) halt = 1'b1;
      end
      
      OpMiscMem: begin
        // FENCE, etc.
      end
      
      default: begin
        illegal_inst = 1'b1;
      end
    endcase
  end

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
  input                    rst,                 // rst for both imem and dmem
  input                    clock_mem,           // clock for both imem and dmem
  input      [`REG_SIZE:0] pc_to_imem,          // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] inst_from_imem,      // the value at memory location pc_to_imem
  input      [`REG_SIZE:0] addr_to_dmem,        // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] load_data_from_dmem, // the value at memory location addr_to_dmem
  input      [`REG_SIZE:0] store_data_to_dmem,  // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
  // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
  // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
  input      [        3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  // preload instructions to mem_array
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(posedge clock_mem) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clock_mem) begin
    if (store_we_to_dmem[0]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
    end
    if (store_we_to_dmem[1]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
    end
    if (store_we_to_dmem[2]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
    end
    if (store_we_to_dmem[3]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
    end
    // dmem is "read-first": read returns value before the write
    load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  clock_proc,
    input  clock_mem,
    input  rst,
    output halt
);

  wire [`REG_SIZE:0] pc_to_imem, inst_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [        3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clock_mem           (clock_mem),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathMultiCycle datapath (
    .clk                 (clock_proc),
    .rst                 (rst),
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    .addr_to_dmem        (mem_data_addr),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we),
    .load_data_from_dmem (mem_data_loaded_value),
    .halt                (halt)
  );

endmodule
