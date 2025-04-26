module risc_processor_testbench;

reg clk;

initial begin
    $dumpfile("risc_processor.vcd"); // VCD output file
    $dumpvars(0, risc_processor_testbench);
end

initial clk = 0;
always #5 clk = ~clk;

top_level_processor proc(
    .clk(clk)
);

initial begin
    repeat (40) @(posedge clk);
    $finish;
end

endmodule