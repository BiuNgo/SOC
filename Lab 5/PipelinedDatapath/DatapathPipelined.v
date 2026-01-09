`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31

// inst. are 32 bits in RV32IM
`define INST_SIZE 31

// RV opcodes are 7 bits
`define OPCODE_SIZE 6

`define DIVIDER_STAGES 10

// Don't forget your old codes
//`include "cla.v"
//`include "DividerUnsignedPipelined.v"

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
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs [0:NumRegs-1];

  integer i;
  
  always @(posedge clk) begin
    if (rst) begin
      for (i=0; i<NumRegs; i=i+1) begin
        regs[i]  <= 32'b0;
      end
    end else if ((we) && (rd != 5'b0)) begin
        regs[rd] <= rd_data;
    end
  end
  
  always @(*) begin
    rs1_data = (rs1 == 5'b0) ? 32'b0 : regs[rs1];
    rs2_data = (rs2 == 5'b0) ? 32'b0 : regs[rs2];
  end

endmodule

module DatapathPipelined (
  input                     clk,
  input                     rst,
  output     [ `REG_SIZE:0] pc_to_imem,
  input      [`INST_SIZE:0] inst_from_imem,
  // dmem is read/write
  output reg [ `REG_SIZE:0] addr_to_dmem,
  input      [ `REG_SIZE:0] load_data_from_dmem,
  output reg [ `REG_SIZE:0] store_data_to_dmem,
  output reg [         3:0] store_we_to_dmem,
  output reg                halt,
  // The PC of the inst currently in Writeback. 0 if not a valid inst.
  output reg [ `REG_SIZE:0] trace_writeback_pc,
  // The bits of the inst currently in Writeback. 0 if not a valid inst.
  output reg [`INST_SIZE:0] trace_writeback_inst
);

  // opcodes - see section 19 of RiscV spec
  localparam [`OPCODE_SIZE:0] OpcodeLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeJalr    = 7'b11_001_11;
  //localparam [`OPCODE_SIZE:0] OpcodeMiscMem = 7'b00_011_11;
  localparam [`OPCODE_SIZE:0] OpcodeJal     = 7'b11_011_11;

  localparam [`OPCODE_SIZE:0] OpcodeRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeEnviron = 7'b11_100_11;

  localparam [`OPCODE_SIZE:0] OpcodeAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpcodeLui     = 7'b01_101_11;

  // ALU Operations
  localparam ALU_ADD    = 0,
             ALU_SUB    = 1,
             ALU_AND    = 2,
             ALU_OR     = 3,
             ALU_XOR    = 4,
             ALU_SLL    = 5,
             ALU_SRL    = 6,
             ALU_SRA    = 7,
             ALU_SLT    = 8,
             ALU_SLTU   = 9,
             ALU_MUL    = 10,
             ALU_MULH   = 11,
             ALU_MULHSU = 12,
             ALU_MULHU  = 13;
  
  // Pipeline Stage Register
  // Fetch Register
  reg  [`REG_SIZE:0]  f_pc;
  wire [`REG_SIZE:0]  f_pc_next;
  wire [`INST_SIZE:0] f_inst;
  wire                stall_pipeline;
  
  // Decode Register
  reg  [`REG_SIZE:0]  d_pc;
  reg  [`INST_SIZE:0] d_inst;
  
  // Execute Register
  reg  [`REG_SIZE:0]  e_pc;
  reg  [`INST_SIZE:0] e_inst;
  reg  [`REG_SIZE:0]  e_rs1_data;
  reg  [`REG_SIZE:0]  e_rs2_data;
  reg  [`REG_SIZE:0]  e_imm;
  reg  [4:0]          e_rd;
  reg  [4:0]          e_rs1;
  reg  [4:0]          e_rs2;
  reg                 e_reg_write;
  reg                 e_mem_write;
  reg                 e_mem_read;
  reg                 e_branch;
  reg                 e_jal;
  reg                 e_jalr;
  reg                 e_halt;
  reg  [3:0]          e_alu_op;
  reg                 e_is_div;
  reg                 e_div_is_signed;
  reg                 e_div_is_rem;
  
  // Memory Register
  reg  [`REG_SIZE:0]  m_pc;
  reg  [`INST_SIZE:0] m_inst;
  reg  [`REG_SIZE:0]  m_alu_result;
  reg  [`REG_SIZE:0]  m_store_data_raw;
  reg  [4:0]          m_rd;
  reg                 m_reg_write;
  reg                 m_mem_write;
  reg                 m_mem_read;
  reg                 m_halt;
  
  // Writeback Register
  reg  [`REG_SIZE:0]  w_pc;
  reg  [`INST_SIZE:0] w_inst;
  reg  [`REG_SIZE:0]  w_alu_result;
  reg  [`REG_SIZE:0]  w_mem_data;
  reg  [4:0]          w_rd;
  reg                 w_reg_write;
  reg                 w_mem_read;
  reg                 w_halt;
  
  localparam PIPE_DEPTH = `DIVIDER_STAGES;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  reg [`REG_SIZE:0] cycles_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /***************/
  /* FETCH STAGE */
  /***************/
  wire [`REG_SIZE:0] branch_target;
  wire               branch_taken;
  
  assign f_pc_next  = branch_taken ? branch_target : (f_pc + 4);
  assign pc_to_imem = f_pc;
  assign f_inst     = inst_from_imem;

  always @(posedge clk) begin
    if (rst) begin
      f_pc <= 0;
    end else if ((!halt) && (!stall_pipeline)) begin
      f_pc <= f_pc_next;
    end
  end

  // F - D Pipeline Register
  always @(posedge clk) begin
    if (rst) begin
      d_pc   <= 0;
      d_inst <= 32'h00000013;
    end else if (branch_taken) begin
      d_pc   <= 0;
      d_inst <= 32'h00000013;
    end else if ((!halt) && (!stall_pipeline)) begin
      d_pc   <= f_pc;
      d_inst <= f_inst;
    //end else if (stall_pipeline) begin
    end
  end

  /****************/
  /* DECODE STAGE */
  /****************/
  wire [6:0] d_opcode  = d_inst[6:0];
  wire [4:0] d_rd      = d_inst[11:7];
  wire [2:0] d_funct3  = d_inst[14:12];
  wire [4:0] d_rs1_idx = d_inst[19:15];
  wire [4:0] d_rs2_idx = d_inst[24:20];
  wire [6:0] d_funct7  = d_inst[31:25];

  // Immediate Generation
  reg [`REG_SIZE:0] d_imm_gen;
  always @(*) begin
    case (d_opcode)
      OpcodeStore:  d_imm_gen = {{20{d_inst[31]}}, d_inst[31:25], d_inst[11:7]};
      OpcodeBranch: d_imm_gen = {{20{d_inst[31]}}, d_inst[7],     d_inst[30:25], d_inst[11:8],  1'b0};
      OpcodeJal:    d_imm_gen = {{12{d_inst[31]}}, d_inst[19:12], d_inst[20],    d_inst[30:21], 1'b0};
      OpcodeLui, OpcodeAuipc: d_imm_gen = {d_inst[31:12], 12'b0};
      default:      d_imm_gen = {{20{d_inst[31]}}, d_inst[31:20]};
    endcase
  end

  // Register File
  wire [`REG_SIZE:0] rf_rs1_data;
  wire [`REG_SIZE:0] rf_rs2_data;
  wire [`REG_SIZE:0] w_final_data;

  RegFile rf (
    .clk(clk),
    .rst(rst),
    .rd(w_rd),
    .rd_data(w_final_data),
    .rs1(d_rs1_idx),
    .rs1_data(rf_rs1_data),
    .rs2(d_rs2_idx),
    .rs2_data(rf_rs2_data),
    .we(w_reg_write)
  );

  // WD bypass
  wire [`REG_SIZE:0] d_rs1_val;
  wire [`REG_SIZE:0] d_rs2_val;

  assign d_rs1_val = ((w_reg_write) && (w_rd != 0) && (w_rd == d_rs1_idx)) ? w_final_data : rf_rs1_data;
  assign d_rs2_val = ((w_reg_write) && (w_rd != 0) && (w_rd == d_rs2_idx)) ? w_final_data : rf_rs2_data;

  // Control Logic
  reg ctrl_reg_write;
  reg ctrl_mem_write;
  reg ctrl_mem_read;
  reg ctrl_branch;
  reg ctrl_jal;
  reg ctrl_jalr;
  reg ctrl_halt;
  reg ctrl_div;
  reg [3:0] ctrl_alu_op;

  always @(*) begin
    ctrl_reg_write = 0;
    ctrl_mem_write = 0;
    ctrl_mem_read  = 0;
    ctrl_branch    = 0;
    ctrl_jal       = 0;
    ctrl_jalr      = 0;
    ctrl_halt      = 0;
    ctrl_div       = 0;
    ctrl_alu_op    = ALU_ADD;

    case (d_opcode)
      OpcodeRegReg: begin
        ctrl_reg_write = 1;
        if (d_funct7 == 7'b0000001) begin
          if (d_funct3[2]) begin
            ctrl_div = 1;
          end else begin
            case (d_funct3)
              3'b000: ctrl_alu_op = ALU_MUL;
              3'b001: ctrl_alu_op = ALU_MULH;
              3'b010: ctrl_alu_op = ALU_MULHSU;
              3'b011: ctrl_alu_op = ALU_MULHU;
              default: ctrl_alu_op = ALU_MUL;
            endcase
          end
        end else begin
          case (d_funct3)
            3'b000: ctrl_alu_op = d_funct7[5] ? ALU_SUB : ALU_ADD;
            3'b001: ctrl_alu_op = ALU_SLL;
            3'b010: ctrl_alu_op = ALU_SLT;
            3'b011: ctrl_alu_op = ALU_SLTU;
            3'b100: ctrl_alu_op = ALU_XOR;
            3'b101: ctrl_alu_op = d_funct7[5] ? ALU_SRA : ALU_SRL;
            3'b110: ctrl_alu_op = ALU_OR;
            3'b111: ctrl_alu_op = ALU_AND;
            default: ;
          endcase
        end
      end

      OpcodeRegImm: begin
        ctrl_reg_write = 1;
        case (d_funct3)
          3'b000: ctrl_alu_op = ALU_ADD;
          3'b001: ctrl_alu_op = ALU_SLL;
          3'b010: ctrl_alu_op = ALU_SLT;
          3'b011: ctrl_alu_op = ALU_SLTU;
          3'b100: ctrl_alu_op = ALU_XOR;
          3'b101: ctrl_alu_op = d_funct7[5] ? ALU_SRA : ALU_SRL;
          3'b110: ctrl_alu_op = ALU_OR;
          3'b111: ctrl_alu_op = ALU_AND;
          default: ;
        endcase
      end

      OpcodeLoad: begin
        ctrl_reg_write = 1;
        ctrl_mem_read  = 1;
      end

      OpcodeStore: begin
        ctrl_mem_write = 1;
      end

      OpcodeBranch: begin
        ctrl_branch = 1;
      end

      OpcodeJal: begin
        ctrl_reg_write = 1;
        ctrl_jal       = 1;
      end

      OpcodeJalr: begin
        ctrl_reg_write = 1;
        ctrl_jalr      = 1;
      end

      OpcodeLui: begin
        ctrl_reg_write = 1;
      end

      OpcodeAuipc: begin
        ctrl_reg_write = 1;
      end

      OpcodeEnviron: begin
        if (d_inst == 32'h00000073) begin
          ctrl_halt = 1;
        end
      end
      
      default: ;
    endcase
  end

  // Divider pipeline
  reg [4:0] div_pipe_rd    [0:PIPE_DEPTH-1];
  reg       div_pipe_valid [0:PIPE_DEPTH-1];
  wire      div_busy;
  reg       div_busy_reg;
  
  integer b;
  
  always @(*) begin
    div_busy_reg = 0;
    for (b=0; b<PIPE_DEPTH; b=b+1) begin
      div_busy_reg = div_busy_reg | div_pipe_valid[b];
    end
  end
  
  assign div_busy = div_busy_reg;

  function check_div_dep;
    input [4:0] reg_idx;

    integer k;

    begin
      check_div_dep = 1'b0;
      if (reg_idx != 0) begin
        for (k=0; k<PIPE_DEPTH; k=k+1) begin
          if ((div_pipe_valid[k]) && (div_pipe_rd[k] == reg_idx)) begin
            check_div_dep = 1'b1;
          end
        end
        
        if (e_is_div && (e_rd == reg_idx)) begin
          check_div_dep = 1'b1;
        end
      end
    end
  endfunction

  // Stall logic
  wire load_use_hazard;
  wire div_dep_hazard;
  wire div_structural_stall;
  //wire flush_decode;

  assign load_use_hazard      = (e_mem_read) && (e_rd != 0) && ((e_rd == d_rs1_idx) || ((e_rd == d_rs2_idx) && (d_opcode != OpcodeStore)));
  assign div_dep_hazard       = (check_div_dep(d_rs1_idx)) || (check_div_dep(d_rs2_idx));
  assign div_structural_stall = (div_busy) && (!ctrl_div);
  assign stall_pipeline       = (load_use_hazard) || (div_dep_hazard) || (div_structural_stall);
  //assign flush_decode         = (branch_taken); //|| (stall_pipeline);

  // D - E Pipeline Register
  always @(posedge clk) begin
    if ((rst) || (branch_taken) || (stall_pipeline)) begin
      e_pc        <= 0;
      e_inst      <= 32'h00000013;
      e_reg_write <= 0;
      e_mem_write <= 0;
      e_mem_read  <= 0;
      e_branch    <= 0;
      e_jal       <= 0;
      e_jalr      <= 0;
      e_halt      <= 0;
      e_is_div    <= 0;
      e_div_is_signed <= 0;
      e_div_is_rem <= 0;
    end else begin
      e_pc        <= d_pc;
      e_inst      <= d_inst;
      e_rs1_data  <= d_rs1_val;
      e_rs2_data  <= d_rs2_val;
      e_imm       <= d_imm_gen;
      e_rd        <= d_rd;
      e_rs1       <= d_rs1_idx;
      e_rs2       <= d_rs2_idx;
      e_reg_write <= ctrl_reg_write;
      e_mem_write <= ctrl_mem_write;
      e_mem_read  <= ctrl_mem_read;
      e_branch    <= ctrl_branch;
      e_jal       <= ctrl_jal;
      e_jalr      <= ctrl_jalr;
      e_halt      <= ctrl_halt;
      e_alu_op    <= ctrl_alu_op;
      e_is_div    <= ctrl_div;
      e_div_is_signed <= ~d_funct3[0];
      e_div_is_rem    <= d_funct3[1];
    end
  end

  /*******************/
  /* EXECUTION STAGE */
  /*******************/
  // MX, WX bypass
  reg [`REG_SIZE:0] alu_in_a;
  reg [`REG_SIZE:0] alu_in_b_raw;

  always @(*) begin
    if ((m_reg_write) && (m_rd != 0) && (m_rd == e_rs1)) begin
      alu_in_a = m_alu_result;
    end else if ((w_reg_write) && (w_rd != 0) && (w_rd == e_rs1)) begin
      alu_in_a = w_final_data;
    end else begin
      alu_in_a = e_rs1_data;
    end

    if ((m_reg_write) && (m_rd != 0) && (m_rd == e_rs2)) begin
      alu_in_b_raw = m_alu_result;
    end else if ((w_reg_write) && (w_rd != 0) && (w_rd == e_rs2)) begin
      alu_in_b_raw = w_final_data;
    end else begin
      alu_in_b_raw = e_rs2_data;
    end
  end

  wire [`REG_SIZE:0] alu_in_b;
  assign alu_in_b = ((e_inst[6:0] == OpcodeRegImm) ||
                    (e_inst[6:0] == OpcodeLoad)    ||
                    (e_inst[6:0] == OpcodeStore)   ||
                    (e_inst[6:0] == OpcodeJalr)    ||
                    (e_inst[6:0] == OpcodeLui)     ||
                    (e_inst[6:0] == OpcodeAuipc)) ? e_imm : alu_in_b_raw;

  wire [`REG_SIZE:0] cla_sum;
  wire               is_sub;

  assign is_sub = ((e_alu_op == ALU_SUB) || (e_alu_op == ALU_SLT) || (e_alu_op == ALU_SLTU) || (e_branch));

  cla adder(
    .a   (alu_in_a),
    .b   ((is_sub) ? ~alu_in_b : alu_in_b),
    .cin (is_sub),
    .sum (cla_sum)
  );
  
  reg [63:0] mul_op_a;
  reg [63:0] mul_op_b;
  wire [63:0] mul_result_wide;
  
  always @(*) begin
    case (e_alu_op)
      ALU_MULH: begin
        mul_op_a = {{32{alu_in_a[31]}}, alu_in_a};
        mul_op_b = {{32{alu_in_b[31]}}, alu_in_b};
      end
      
      ALU_MULHSU: begin
        mul_op_a = {{32{alu_in_a[31]}}, alu_in_a};
        mul_op_b = {32'b0, alu_in_b};
      end
      
      ALU_MULHU: begin
        mul_op_a = {32'b0, alu_in_a};
        mul_op_b = {32'b0, alu_in_b};
      end
      
      default: begin
        mul_op_a = {{32{alu_in_a[31]}}, alu_in_a};
        mul_op_b = {{32{alu_in_b[31]}}, alu_in_b};
      end
    endcase
  end
  
  assign mul_result_wide = mul_op_a * mul_op_b;

  reg [`REG_SIZE:0] e_alu_result;
  wire              eq;
  wire              lt_s;
  wire              lt_u;

  assign eq   = (cla_sum == 0);
  assign lt_s = (alu_in_a[31] != alu_in_b[31]) ? alu_in_a[31] : cla_sum[31];
  assign lt_u = (alu_in_a < alu_in_b);

  always @(*) begin
    case (e_alu_op)
      ALU_ADD, ALU_SUB: e_alu_result = cla_sum;
      ALU_AND:          e_alu_result = alu_in_a & alu_in_b;
      ALU_OR:           e_alu_result = alu_in_a | alu_in_b;
      ALU_XOR:          e_alu_result = alu_in_a ^ alu_in_b;
      ALU_SLL:          e_alu_result = alu_in_a << alu_in_b[4:0];
      ALU_SRL:          e_alu_result = alu_in_a >> alu_in_b[4:0];
      ALU_SRA:          e_alu_result = $signed(alu_in_a) >>> alu_in_b[4:0];
      ALU_SLT:          e_alu_result = {31'b0, lt_s};
      ALU_SLTU:         e_alu_result = {31'b0, lt_u};
      ALU_MUL:          e_alu_result = mul_result_wide[31:0];
      ALU_MULH,
      ALU_MULHSU,
      ALU_MULHU:        e_alu_result = mul_result_wide[63:32];
      default:          e_alu_result = 0;
    endcase

    if (e_inst[6:0] == OpcodeLui) begin
      e_alu_result = e_imm;
    end else if (e_inst[6:0] == OpcodeAuipc) begin
      e_alu_result = e_pc + e_imm;
    end else if (e_inst[6:0] == OpcodeJal || e_inst[6:0] == OpcodeJalr)  begin
      e_alu_result = e_pc + 4;
    end
  end

  // Branch Logic
  reg br_cond;

  always @(*) begin
    case (e_inst[14:12])
      3'b000: br_cond = eq;
      3'b001: br_cond = !eq;
      3'b100: br_cond = lt_s;
      3'b101: br_cond = !lt_s;
      3'b110: br_cond = lt_u;
      3'b111: br_cond = !lt_u;
      default: br_cond = 0;
    endcase
  end

  assign branch_taken  = (e_branch && br_cond) || (e_jal) || (e_jalr);
  assign branch_target = (e_jalr) ? ((alu_in_a + e_imm) & ~1) : (e_pc + e_imm);
  
  wire div_sign_a = e_div_is_signed & alu_in_a[31];
  wire div_sign_b = e_div_is_signed & alu_in_b_raw[31];
  wire [`REG_SIZE:0] div_abs_a = div_sign_a ? -alu_in_a : alu_in_a;
  wire [`REG_SIZE:0] div_abs_b = div_sign_b ? -alu_in_b_raw : alu_in_b_raw;
  wire div_by_zero = (alu_in_b_raw == 0);

  wire div_neg_q = (alu_in_b_raw != 0) && (div_sign_a ^ div_sign_b);
  wire div_neg_r = div_sign_a;

  // Divider pipeline
  wire [`REG_SIZE:0] div_rem_out;
  wire [`REG_SIZE:0] div_quo_out;

  DividerUnsignedPipelined divider (
    .clk         (clk),
    .rst         (rst),
    .stall       (1'b0),
    .i_dividend  (div_abs_a),
    .i_divisor   (div_abs_b),
    .o_remainder (div_rem_out),
    .o_quotient  (div_quo_out)
  );

  reg [`REG_SIZE:0]  div_pc_sr        [0:PIPE_DEPTH-1];
  reg [`INST_SIZE:0] div_inst_sr      [0:PIPE_DEPTH-1];
  reg [4:0]          div_rd_sr        [0:PIPE_DEPTH-1];
  reg                div_reg_write_sr [0:PIPE_DEPTH-1];
  reg                div_is_rem_sr    [0:PIPE_DEPTH-1];
  reg                div_neg_q_sr     [0:PIPE_DEPTH-1];
  reg                div_neg_r_sr     [0:PIPE_DEPTH-1];
  reg                div_by_zero_sr   [0:PIPE_DEPTH-1];
  reg [`REG_SIZE:0]  div_orig_div_sr  [0:PIPE_DEPTH-1];

  integer j;

  always @(posedge clk) begin
    if (rst) begin
      for (j=0; j<PIPE_DEPTH; j=j+1) begin
        div_pipe_valid[j]   <= 0;
        div_pipe_rd[j]      <= 0;
        div_reg_write_sr[j] <= 0;
        div_neg_q_sr[j]     <= 0;
        div_neg_r_sr[j]     <= 0;
        div_by_zero_sr[j]   <= 0;
        div_orig_div_sr[j]  <= 0;
      end
    end else begin
      for (j=PIPE_DEPTH-1; j>0; j=j-1) begin
        div_pipe_valid[j]   <= div_pipe_valid[j-1];
        div_pipe_rd[j]      <= div_pipe_rd[j-1];
        div_pc_sr[j]        <= div_pc_sr[j-1];
        div_inst_sr[j]      <= div_inst_sr[j-1];
        div_reg_write_sr[j] <= div_reg_write_sr[j-1];
        div_is_rem_sr[j]    <= div_is_rem_sr[j-1];
        div_neg_q_sr[j]     <= div_neg_q_sr[j-1];
        div_neg_r_sr[j]     <= div_neg_r_sr[j-1];
        div_by_zero_sr[j]   <= div_by_zero_sr[j-1];
        div_orig_div_sr[j]  <= div_orig_div_sr[j-1];
      end

      div_pipe_valid[0]   <= e_is_div;
      div_pipe_rd[0]      <= e_is_div ? e_rd : 5'b0;
      div_pc_sr[0]        <= e_pc;
      div_inst_sr[0]      <= e_inst;
      div_reg_write_sr[0] <= e_reg_write;
      div_is_rem_sr[0]    <= e_div_is_rem;
      div_neg_q_sr[0]     <= div_neg_q;
      div_neg_r_sr[0]     <= div_neg_r;
      div_by_zero_sr[0]   <= div_by_zero;
      div_orig_div_sr[0]  <= alu_in_a;
    end
  end

  // E - M pipeline
  wire div_done = div_pipe_valid[PIPE_DEPTH-1];

  always @(posedge clk) begin
    if (rst) begin
      m_pc        <= 0;
      m_inst      <= 32'h00000013;
      m_reg_write <= 0;
      m_mem_write <= 0;
      m_mem_read  <= 0;
      m_halt      <= 0;
      m_alu_result <= 0;
      m_rd <= 0;
    end else begin
      if (div_done) begin
        m_pc         <= div_pc_sr[PIPE_DEPTH-1];
        m_inst       <= div_inst_sr[PIPE_DEPTH-1];
        
        if (div_by_zero_sr[PIPE_DEPTH-1]) begin
          if (div_is_rem_sr[PIPE_DEPTH-1]) begin
            m_alu_result <= div_orig_div_sr[PIPE_DEPTH-1];
          end else begin
            m_alu_result <= 32'hFFFFFFFF;
          end
        end else begin
          m_alu_result <= div_is_rem_sr[PIPE_DEPTH-1] 
                          ? (div_neg_r_sr[PIPE_DEPTH-1] ? -div_rem_out : div_rem_out)
                          : (div_neg_q_sr[PIPE_DEPTH-1] ? -div_quo_out : div_quo_out);
        end
        
        m_rd         <= div_rd_sr[PIPE_DEPTH-1];
        m_reg_write  <= div_reg_write_sr[PIPE_DEPTH-1];
        m_mem_write  <= 0;
        m_mem_read   <= 0;
        m_halt       <= 0;
      end else if (e_is_div) begin
        m_pc         <= 0;
        m_inst       <= 32'h00000013;
        m_reg_write  <= 0;
        m_mem_write  <= 0;
        m_mem_read   <= 0;
        m_halt       <= 0;
        m_alu_result <= 0;
        m_rd         <= 0;
      end else begin
        m_pc             <= e_pc;
        m_inst           <= e_inst;
        m_alu_result     <= e_alu_result;
        m_store_data_raw <= alu_in_b_raw;
        m_rd             <= e_rd;
        m_reg_write      <= e_reg_write;
        m_mem_write      <= e_mem_write;
        m_mem_read       <= e_mem_read;
        m_halt           <= e_halt;
      end
    end
  end

  /****************/
  /* MEMORY STAGE */
  /****************/
  reg [`REG_SIZE:0] m_final_store_data;

  always @(*) begin
    if ((w_reg_write) && (w_rd != 0) && (w_rd == m_inst[24:20])) begin
      m_final_store_data = w_final_data;
    end else begin
      m_final_store_data = m_store_data_raw;
    end
  end

  always @(*) begin
    addr_to_dmem       = m_alu_result;
    store_data_to_dmem = m_final_store_data;
    store_we_to_dmem   = 4'b1111;

    case (m_inst[14:12])
      3'b000: begin
        store_data_to_dmem = {4{m_final_store_data[7:0]}};
        case (m_alu_result[1:0])
          2'b00: store_we_to_dmem = 4'b0001;
          2'b01: store_we_to_dmem = 4'b0010;
          2'b10: store_we_to_dmem = 4'b0100;
          2'b11: store_we_to_dmem = 4'b1000;
        endcase
      end

      3'b001: begin
        store_data_to_dmem = {2{m_final_store_data[15:0]}};
        case (m_alu_result[1])
          1'b0: store_we_to_dmem = 4'b0011;
          1'b1: store_we_to_dmem = 4'b1100;
        endcase
      end

      default: ;
    endcase

    if (!m_mem_write) begin
      store_we_to_dmem = 4'b0000;
    end
  end

  // M - W pipeline
  always @(posedge clk) begin
    if (rst) begin
      w_pc        <= 0;
      w_inst      <= 32'h00000013;
      w_reg_write <= 0;
      w_mem_read  <= 0;
      w_halt      <= 0;
    end else begin
      w_pc         <= m_pc;
      w_inst       <= m_inst;
      w_alu_result <= m_alu_result;
      w_mem_read   <= m_mem_read;
      w_mem_data   <= load_data_from_dmem;
      w_rd         <= m_rd;
      w_reg_write  <= m_reg_write;
      w_halt       <= m_halt;
    end
  end

  /*******************/
  /* WRITEBACK STAGE */
  /*******************/
  reg [`REG_SIZE:0] w_load_val;

  always @(*) begin
    case (w_inst[14:12])
      3'b000: begin
        case (w_alu_result[1:0])
          2'b00: w_load_val = {{24{w_mem_data[7]}}, w_mem_data[7:0]};
          2'b01: w_load_val = {{24{w_mem_data[15]}}, w_mem_data[15:8]};
          2'b10: w_load_val = {{24{w_mem_data[23]}}, w_mem_data[23:16]};
          2'b11: w_load_val = {{24{w_mem_data[31]}}, w_mem_data[31:24]};
        endcase
      end

      3'b001: begin
        case (w_alu_result[1])
          1'b0: w_load_val = {{16{w_mem_data[15]}}, w_mem_data[15:0]};
          1'b1: w_load_val = {{16{w_mem_data[31]}}, w_mem_data[31:16]};
        endcase
      end

      3'b010: w_load_val = w_mem_data;
      
      3'b100: begin
        case (w_alu_result[1:0])
          2'b00: w_load_val = {24'b0, w_mem_data[7:0]};
          2'b01: w_load_val = {24'b0, w_mem_data[15:8]};
          2'b10: w_load_val = {24'b0, w_mem_data[23:16]};
          2'b11: w_load_val = {24'b0, w_mem_data[31:24]};
        endcase
      end

      3'b101: begin
        case (w_alu_result[1])
          1'b0: w_load_val = {16'b0, w_mem_data[15:0]};
          1'b1: w_load_val = {16'b0, w_mem_data[31:16]};
        endcase
      end

      default: w_load_val = w_mem_data;
    endcase
  end

  assign w_final_data = (w_mem_read) ? w_load_val : w_alu_result;

  always @(*) begin
    halt = w_halt;
    trace_writeback_pc = w_pc;
    trace_writeback_inst = w_inst;
  end

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
    input                    rst,                 // rst for both imem and dmem
    input                    clk,                 // clock for both imem and dmem
	                                              // The memory reads/writes on @(negedge clk)
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
//  initial begin
//    $readmemh("mem_initial_contents.hex", mem_array);
//  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(negedge clk) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clk) begin
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

/* This design has just one clock for both processor and memory. */
module Processor (
    input                 clk,
    input                 rst,
    output                halt,
    output [ `REG_SIZE:0] trace_writeback_pc,
    output [`INST_SIZE:0] trace_writeback_inst
);

  wire [`INST_SIZE:0] inst_from_imem;
  wire [ `REG_SIZE:0] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [         3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clk                 (clk),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathPipelined datapath (
    .clk                  (clk),
    .rst                  (rst),
    .pc_to_imem           (pc_to_imem),
    .inst_from_imem       (inst_from_imem),
    .addr_to_dmem         (mem_data_addr),
    .store_data_to_dmem   (mem_data_to_write),
    .store_we_to_dmem     (mem_data_we),
    .load_data_from_dmem  (mem_data_loaded_value),
    .halt                 (halt),
    .trace_writeback_pc   (trace_writeback_pc),
    .trace_writeback_inst (trace_writeback_inst)
  );

endmodule