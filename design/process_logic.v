module process_logic #(
    parameter   BL  =   3'b011, // Burst Length = 8
                BT  =   1'b0,   // Burst Type = Sequential
                CL  =   3'b100, // CAS Latency (CL) = 4
                AL  =   3'b100  // Posted CAS# Additive Latency (AL) = 4
)(
    // Generic inputs
    input           ready_init_i,
    input           clk,
    input           reset,
    input           CK,
    // FIFO related inputs
    input   [24:0]  addr_cmdFIFO_i,
    input   [2:0]   cmd_cmdFIFO_i,
    input   [1:0]   sz_cmdFIFO_i, 
    input   [2:0]   op_cmdFIFO_i, 
    input   [15:0]  din_dataFIFO_i,
    input           emptyBar_cmdFIFO_i,
    input           fullBar_returnFIFO_i,
    input   [6:0]   fillcount_returnFIFO_i,
    // Address outputs
    output  [24:0]  addr_returnFIFO_o, 
    output          get_cmdFIFO_o, 
    output          get_dataFIFO_o, 
    output          put_returnFIFO_o,
    output          csbar_o, 
    output          rasbar_o, 
    output          casbar_o, 
    output          webar_o,
    output  [2:0]   readPtr_ringBuff_o, 
    output          listen_ringBuff_o,
    output  [15:0]  dq_SSTL_o,
    output  [1:0]   dm_SSTL_o,
    output  [1:0]   dqs_i_SSTL_o,
    output  [1:0]   ba,
    output  [12:0]  a,
    output          ts_i_o,
    output          ri_i_o
);

    // Output registers
    reg             get_cmdFIFO_o;
    reg             put_returnFIFO_o;
    reg     [2:0]   readPtr_ringBuff_o; 
    reg             listen_ringBuff_o;
    reg     [15:0]  dq_SSTL_o;
    reg     [1:0]   dm_SSTL_o;
    reg     [1:0]   dqs_i_SSTL_o;
    reg             ts_i_o; 
    reg             ri_i_o;
    reg     [24:0]  addr_returnFIFO_o;
    reg     [12:0]  a;

    // Internal registers
    reg             csbar, rasbar, casbar, webar;
    reg     [24:0]  addr_cmdFIFO_reg;
    reg     [5:0]   cnt_reg;            // Generic state machine counter.
    reg     [11:0]  refCnt_reg;
    reg     [3:0]   blkCnt_reg;
    reg     [3:0]   state;              // Active state.
    reg     [1:0]   block_state;
    reg     [12:0]  row_address;        // Current row address.
    reg     [1:0]   bank_address;       // Current bank address.
    reg     [9:0]   column_address;     // Current column address.
    reg     [12:0]  rowb_address;       // Current row address (banked).
    reg     [1:0]   bankb_address;      // Current bank address (banked).
    reg     [9:0]   columnb_address;    // Current column address (banked).
    reg     [2:0]   cmd_reg;
    reg     [1:0]   sz_reg;
    reg     [1:0]   szBlk_reg;
    reg     [1:0]   szRd_reg;
    reg             flag;
    reg             get_dataFIFO_p;
    reg             get_dataFIFO_n;

    // Internal wires
    wire    [12:0]  row_address_wire;        // Current row address.
    wire    [1:0]   bank_address_wire;       // Current bank address.
    wire    [9:0]   column_address_wire;     // Current column address.
    wire            cmdready;

    // Internal parameters
    localparam  [2:0]   NOP         =   3'b000, // Current command (from FIFO).
                        SCR         =   3'b001,
                        SCW         =   3'b010,
                        BLR         =   3'b011,
                        BLW         =   3'b100;

    localparam  [1:0]   B_IDLE      =   2'b00,
                        B_ACT       =   2'b01,
                        B_WRT       =   2'b10,
                        B_RD        =   2'b11;

    localparam  [3:0]   IDLE        =   4'b0000, // States of the state machine.
                        GETCMD      =   4'b0001,
                        ACTIVATE    =   4'b0010,
                        SCREAD      =   4'b0011,
                        SCRLSN      =   4'b0100,
                        SCRNLSN     =   4'b0101,
                        SCRDNTH     =   4'b0110,
                        SCREND      =   4'b0111,
                        SCWRITE     =   4'b1000,
                        SCWRDATA    =   4'b1001,
                        SCWREND     =   4'b1010,
                        RPRE        =   4'b1011,
                        RNOP        =   4'b1100,
                        RREF        =   4'b1101,
                        RIDL        =   4'b1110;

    // Command for the SSTL (Uses csbar, casbar, webar and rasbar).
    localparam  [3:0]   CNOP        =   4'b0111,    // NOP.
                        CLM         =   4'b0000,    // MR and EMRs.
                        CREFRESH    =   4'b0001,    // Refresh and Auto Refresh.
                        CPRECHARGE  =   4'b0010,    // Precharge.
                        CACTIVATE   =   4'b0011,    // Activate.
                        CREAD       =   4'b0101,    // Read.
                        CWRITE      =   4'b0100;    // Write.

    assign row_address_wire     = (flag) ? rowb_address     : row_address;
    assign column_address_wire  = (flag) ? columnb_address  : column_address;
    assign bank_address_wire    = (flag) ? bankb_address    : bank_address;
    assign get_dataFIFO_o = get_dataFIFO_p | get_dataFIFO_n;
    assign cmdready = (emptyBar_cmdFIFO_i & ready_init_i) ? 1'b1 : 1'b0;   // Initialization is complete and input FIFOs are not empty.
    assign ba = bank_address_wire; // The current bank address.
    assign casbar_o = (flag) ? casbarb  : casbar;
    assign csbar_o  = (flag) ? csbarb   : csbar;
    assign rasbar_o = (flag) ? rasbarb  : rasbar;
    assign webar_o  = (flag) ? webarb   : webar;

    // Address signal generation
    reg flag_a;
    always @ (posedge clk) begin
        if (reset) begin
            a   <=  13'b0;
            flag_a  <=  1'b0;
        end else begin
            a   <=   row_address_wire;
            if (flag_a) begin
                flag_a  <=  1'b0;
                a       <=  a;
            end else if (block_state == B_ACT && !blkCnt_reg) begin
                a   <=  {3'b001, column_address_wire};
                flag_a  <=  1'b1; 
            end else if (!cnt_reg) begin
                if (state == ACTIVATE) begin 
                    a   <=  {3'b001, column_address_wire};
                    flag_a  <=  1'b1;
                end 
                if (state == RPRE) begin 
                    a   <=  {3'b001, 10'b0};
                    flag_a  <=  1'b1;
                end
            end
        end
    end

    // Refresh counter and state management
    always @ (posedge clk) begin
        if (reset || !ready_init_i)
            refCnt_reg  <=  12'b1111_0011_1100;
        else
            refCnt_reg  <=  refCnt_reg - 1'b1;

        if(flag && block_state == B_IDLE && !blkCnt_reg && !szBlk_reg) begin
            flag    <=  1'b0;
        end
        if (flag && (block_state == B_RD || block_state == B_WRT) && !blkCnt_reg) begin
            addr_cmdFIFO_reg    <=  addr_cmdFIFO_reg + 4'b1000;
        end

        if(reset) begin
            state               <=  IDLE;
            cnt_reg             <=  6'b0;
            put_returnFIFO_o    <=  1'b0;
            get_cmdFIFO_o       <=  1'b0;
            get_dataFIFO_p      <=  1'b0;
            listen_ringBuff_o   <=  1'b0;
            readPtr_ringBuff_o  <=  3'b0;
            addr_returnFIFO_o   <=  25'b0;
            addr_cmdFIFO_reg    <=  25'b0;
            row_address         <=  13'b0;
            bank_address        <=  2'b0;
            column_address      <=  10'b0;
            flag                <=  1'b0;
            cmd_reg             <=  3'b0;
            sz_reg              <=  2'b0;
            szRd_reg            <=  2'b0;
            {csbar, rasbar, casbar, webar} <= CNOP;
        end else begin
            case(state)
                IDLE: begin
                    if (ready_init_i && refCnt_reg < 12'b0000_0110_0100) begin
                        state               <=  RPRE;
                        if (CK) begin
                            {csbar, rasbar, casbar, webar}  <=  CPRECHARGE;
                            cnt_reg         <=  6'b00_0001;
                        end else begin
                            {csbar, rasbar, casbar, webar}  <=  CNOP;
                            cnt_reg         <=  6'b00_0010;
                        end
                    end else if(!cnt_reg) begin
                        if(cmdready) begin
                            state               <=  GETCMD;
                            cnt_reg             <=  6'b00_0010;
                            get_cmdFIFO_o       <=  1'b1;
                            readPtr_ringBuff_o  <=  3'b111;
                            flag                <=  1'b0;
                        end else begin
                            state               <=  IDLE;
                            cnt_reg             <=  6'b00;
                            get_cmdFIFO_o       <=  1'b0;
                        end
                    end else begin
                        state       <=  IDLE;
                        cnt_reg     <=  cnt_reg - 1'b1;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end
                end
                // Precharge all banks (for refresh).
                RPRE: begin
                    if (!cnt_reg) begin
                        state       <=  RNOP;
                        cnt_reg     <=  6'b00_0111;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end else begin
                        state       <=  RPRE;
                        cnt_reg     <=  cnt_reg - 1'b1;
                        {csbar, rasbar, casbar, webar}  <=  CPRECHARGE;
                    end
                end
                // Wait after precharge (for refresh).
                RNOP: begin
                    if (!cnt_reg) begin
                        state       <=  RREF;
                        cnt_reg     <=  6'b00_0001;
                        {csbar, rasbar, casbar, webar}  <=  CREFRESH;
                    end else begin
                        state       <=  RNOP;
                        cnt_reg     <=  cnt_reg - 1'b1;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end
                end
                // Give the refresh command.
                RREF: begin
                    if (!cnt_reg) begin
                        state       <=  RIDL;
                        cnt_reg     <=  6'b11_0111;
                        refCnt_reg  <=  12'b1111_0011_1100;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end else begin
                        state       <=  RREF;
                        cnt_reg     <=  cnt_reg - 1'b1;
                        {csbar, rasbar, casbar, webar}  <=  CREFRESH;
                    end
                end
                // Wait before next activate can be given.
                RIDL: begin
                    if (!cnt_reg) begin
                        state       <=  IDLE;
                        cnt_reg     <=  6'b00_0001;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end else begin
                        state       <=  RIDL;
                        cnt_reg     <=  cnt_reg - 1'b1;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end
                end
                // If ready is active and command fifo has commands, get the command.
                GETCMD: begin
                    if(!cnt_reg) begin
                        state               <=  ACTIVATE;   // For scalar read or write command, go to activate.
                        addr_cmdFIFO_reg    <=  addr_cmdFIFO_i;
                        addr_returnFIFO_o   <=  addr_cmdFIFO_i;
                        cmd_reg             <=  cmd_cmdFIFO_i;
                        sz_reg              <=  sz_cmdFIFO_i;
                        szRd_reg            <=  sz_cmdFIFO_i;

                        if (CK) begin
                            {csbar, rasbar, casbar, webar}  <=  CACTIVATE;
                            cnt_reg         <=  6'b00_0001;
                        end else begin
                            {csbar, rasbar, casbar, webar}  <=  CNOP;
                            cnt_reg         <=  6'b00_0010;
                        end
                    end else begin
                        state           <= GETCMD;
                        cnt_reg         <=  cnt_reg - 1'b1;
                        get_cmdFIFO_o   <=  1'b0;
                        {csbar, rasbar, casbar, webar}  <=  CNOP;
                    end
                    if (cnt_reg == 6'b00_0001) begin
                        row_address         <=  addr_cmdFIFO_i[24:12];
                        column_address      <=  {addr_cmdFIFO_i[11: 5], addr_cmdFIFO_i[2:0]};
                        bank_address        <=  addr_cmdFIFO_i[4:3];
                    end
                end
                // If the command is valid, then give the activate command.
                ACTIVATE: begin
                    if(!cnt_reg) begin
                        case(cmd_reg)
                            SCR: begin
                                state           <=  SCREAD;
                                cnt_reg         <=  6'b00_0001;
                                {csbar, rasbar, casbar, webar}  <=  CREAD;
                            end
                            SCW: begin
                                state           <=  SCWRITE;
                                cnt_reg         <=  6'b00_0001;
                                get_dataFIFO_p  <=  1'b1;
                                {csbar, rasbar, casbar, webar}  <=  CWRITE;
                            end
                            BLR: begin
                                state   <=  ACTIVATE;
                                cnt_reg <=  6'b00_0000;
                                {csbar, rasbar, casbar, webar}  <=  CNOP;
                                case (sz_reg)
                                    2'b00: if (fillcount_returnFIFO_i < 6'b11_1000) begin
                                        state   <=  SCREAD;
                                        cnt_reg <=  6'b00_0001;
                                        {csbar, rasbar, casbar, webar}  <=  CREAD;
                                    end
                                    2'b01: if (fillcount_returnFIFO_i < 6'b11_0000) begin
                                        state   <=  SCREAD;
                                        cnt_reg <=  6'b00_0001;
                                        {csbar, rasbar, casbar, webar}  <=  CREAD;
                                    end
                                    2'b10: if (fillcount_returnFIFO_i < 6'b10_1000) begin
                                        state   <=  SCREAD;
                                        cnt_reg <=  6'b00_0001;
                                        {csbar, rasbar, casbar, webar}  <=  CREAD;
                                    end
                                    2'b11: if (fillcount_returnFIFO_i < 6'b10_0000) begin
                                        state   <=  SCREAD;
                                        cnt_reg <=  6'b00_0001;
                                        {csbar, rasbar, casbar, webar}  <=  CREAD;
                                    end
                                    default: begin
                                        state   <=  ACTIVATE;
                                        cnt_reg <=  6'b00_0001;
                                        {csbar, rasbar, casbar, webar}  <=  CACTIVATE;
                                    end
                                endcase
                            end
                            BLW: begin
                                state           <=  SCWRITE;
                                cnt_reg         <=  6'b00_0001;
                                get_dataFIFO_p  <=  1'b1;
                                {csbar, rasbar, casbar, webar}  <=  CWRITE;
                            end
                            default: begin
                                state   <=  IDLE;
                                cnt_reg <=  6'b00_0000;
                                {csbar, rasbar, casbar, webar}  <=  CNOP;
                            end
                        endcase
                    end else begin
                        state   <=  ACTIVATE;
                        cnt_reg <=  cnt_reg - 1'b1;
                       