module mt47h32m16_37e(
    ck,
    ckbar,
    cke,
    csbar,
    rasbar,
    casbar,
    webar,
    ba,
    a,
    dq,
    dqs,
    dqsbar,
    dm,
    odt
);
    parameter memory_spec = "/home/scf-06/ee577/ee577b/ddr2/mt47h32m16_37e.soma";
    parameter init_file   = "";
    parameter sim_control = "";
    input ck;
    input ckbar;
    input cke;
    input csbar;
    input rasbar;
    input casbar;
    input webar;
    input [1:0] ba;
    input [12:0] a;
    inout [15:0] dq;
      reg [15:0] den_dq;
      assign dq = den_dq;
    inout [1:0] dqs;
      reg [1:0] den_dqs;
      assign dqs = den_dqs;
    inout [1:0] dqsbar;
      reg [1:0] den_dqsbar;
      assign dqsbar = den_dqsbar;
    input [1:0] dm;
    input odt;
initial
    $ddr_II_access(ck,ckbar,cke,csbar,rasbar,casbar,webar,ba,a,dq,den_dq,dqs,den_dqs,dqsbar,den_dqsbar,dm,odt);
endmodule

