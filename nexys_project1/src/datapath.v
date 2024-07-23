//=============================================================================
//               ------->  Revision History  <------
//=============================================================================
//
//   Date     Who   Ver  Changes
//=============================================================================
// 17-Jul-24  SM     1  Initial creation
//=============================================================================

/*

    This module ....

*/

module datapath #(parameter DW=512)
(
    input              clk, resetn,
    input[31:0]        FRAME_SIZE,
    // The input frame data stream
    input[DW-1:0]      axis_df_tdata,
    input              axis_df_tvalid,
    output             axis_df_tready,

    // The input meta data stream
    input[DW-1:0]      axis_md_tdata,
    input              axis_md_tvalid,
    output             axis_md_tready,

    // #1 output stream
    output[DW-1:0]           axis_out0_tdata,
    output[(DW/8)-1:0]       axis_out0_tkeep,
    output                   axis_out0_tlast,
    output                   axis_out0_tvalid,
    input                    axis_out0_tready,

    // #2 output stream
    output[DW-1:0]           axis_out1_tdata,
    output[(DW/8)-1:0]       axis_out1_tkeep,
    output                   axis_out1_tlast,
    output                   axis_out1_tvalid,
    input                    axis_out1_tready,

     //================== This is an AXI4-Lite slave interface ==================
        
    // "Specify write address"              -- Master --    -- Slave --
    input[31:0]                             DP_CONFIG_AWADDR,   
    input                                   DP_CONFIG_AWVALID,  
    output                                                  DP_CONFIG_AWREADY,
    input[2:0]                              DP_CONFIG_AWPROT,

    // "Write Data"                         -- Master --    -- Slave --
    input[31:0]                             DP_CONFIG_WDATA,      
    input                                   DP_CONFIG_WVALID,
    input[3:0]                              DP_CONFIG_WSTRB,
    output                                                  DP_CONFIG_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    output[1:0]                                             DP_CONFIG_BRESP,
    output                                                  DP_CONFIG_BVALID,
    input                                   DP_CONFIG_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    input[31:0]                             DP_CONFIG_ARADDR,     
    input                                   DP_CONFIG_ARVALID,
    input[2:0]                              DP_CONFIG_ARPROT,     
    output                                                  DP_CONFIG_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    output[31:0]                                            DP_CONFIG_RDATA,
    output                                                  DP_CONFIG_RVALID,
    output[1:0]                                             DP_CONFIG_RRESP,
    input                                   DP_CONFIG_RREADY
    //==========================================================================
);


//=========================  AXI Register Map  =============================
localparam REG_START      = 0;
localparam REG_FRAME_SIZE = 1;

localparam DEFAULT_FRAME_SIZE = 4096;
//==========================================================================

//==========================================================================
// We'll communicate with the AXI4-Lite Slave core with these signals.
//==========================================================================
// AXI Slave Handler Interface for write requests
wire[31:0]  ashi_windx;     // Input   Write register-index
wire[31:0]  ashi_waddr;     // Input:  Write-address
wire[31:0]  ashi_wdata;     // Input:  Write-data
wire        ashi_write;     // Input:  1 = Handle a write request
reg[1:0]    ashi_wresp;     // Output: Write-response (OKAY, DECERR, SLVERR)
wire        ashi_widle;     // Output: 1 = Write state machine is idle

// AXI Slave Handler Interface for read requests
wire[31:0]  ashi_rindx;     // Input   Read register-index
wire[31:0]  ashi_raddr;     // Input:  Read-address
wire        ashi_read;      // Input:  1 = Handle a read request
reg[31:0]   ashi_rdata;     // Output: Read data
reg[1:0]    ashi_rresp;     // Output: Read-response (OKAY, DECERR, SLVERR);
wire        ashi_ridle;     // Output: 1 = Read state machine is idle
//==========================================================================

// The state of the state-machines that handle AXI4-Lite read and AXI4-Lite write
reg ashi_write_state, ashi_read_state;

// The AXI4 slave state machines are idle when in state 0 and their "start" signals are low
assign ashi_widle = (ashi_write == 0) && (ashi_write_state == 0);
assign ashi_ridle = (ashi_read  == 0) && (ashi_read_state  == 0);
   
// These are the valid values for ashi_rresp and ashi_wresp
localparam OKAY   = 0;
localparam SLVERR = 2;
localparam DECERR = 3;

// An AXI slave is gauranteed a minimum of 128 bytes of address space
// (128 bytes is 32 32-bit registers)
localparam ADDR_MASK = 7'h7F;

// This will strobe high for one cycle when it's time to generate data
reg start;

//==========================================================================
// This state machine handles AXI4-Lite write requests
//
// Drives:
//==========================================================================
always @(posedge clk) begin

    // This will strobe high for only 1 clock-cycle at a time
    start <= 0;

    // If we're in reset, initialize important registers
    if (resetn == 0) begin
        ashi_write_state  <= 0;
        FRAME_SIZE        <= DEFAULT_FRAME_SIZE;

    // If we're not in reset, and a write-request has occured...        
    end else case (ashi_write_state)
        
        0:  if (ashi_write) begin
       
                // Assume for the moment that the result will be OKAY
                ashi_wresp <= OKAY;              
            
                // Convert the byte address into a register index
                case (ashi_windx)
               
                    REG_FRAME_SIZE:
                        FRAME_SIZE <= ashi_wdata;

                    // Does the user want us to generate some frames?
                    REG_START:
                        if (ashi_wdata) begin
                            frames_to_generate <= ashi_wdata;
                            start              <= 1;
                        end

                    // Writes to any other register are a decode-error
                    default: ashi_wresp <= DECERR;
                endcase
            end

        // Dummy state, doesn't do anything
        1: ashi_write_state <= 0;

    endcase
end
//==========================================================================





//==========================================================================
// World's simplest state machine for handling AXI4-Lite read requests
//==========================================================================
always @(posedge clk) begin
    // If we're in reset, initialize important registers
    if (resetn == 0) begin
        ashi_read_state <= 0;
    
    // If we're not in reset, and a read-request has occured...        
    end else if (ashi_read) begin
   
        // Assume for the moment that the result will be OKAY
        ashi_rresp <= OKAY;              
        
        // Convert the byte address into a register index
        case (ashi_rindx)
            
            // Allow a read from any valid register                
            REG_START:      ashi_rdata <= (fdsm_idle == 0);
            REG_FRAME_SIZE: ashi_rdata <= FRAME_SIZE;
            
            // Reads of any other register are a decode-error
            default: ashi_rresp <= DECERR;
        endcase
    end
end
//==========================================================================



//==========================================================================
// This connects us to an AXI4-Lite slave core
//==========================================================================
axi4_lite_slave#(ADDR_MASK) axil_slave
(
    .clk            (clk),
    .resetn         (resetn),
    
    // AXI AW channel
    .AXI_AWADDR     (S_AXI_AWADDR),
    .AXI_AWVALID    (S_AXI_AWVALID),   
    .AXI_AWREADY    (S_AXI_AWREADY),
    
    // AXI W channel
    .AXI_WDATA      (S_AXI_WDATA),
    .AXI_WVALID     (S_AXI_WVALID),
    .AXI_WSTRB      (S_AXI_WSTRB),
    .AXI_WREADY     (S_AXI_WREADY),

    // AXI B channel
    .AXI_BRESP      (S_AXI_BRESP),
    .AXI_BVALID     (S_AXI_BVALID),
    .AXI_BREADY     (S_AXI_BREADY),

    // AXI AR channel
    .AXI_ARADDR     (S_AXI_ARADDR), 
    .AXI_ARVALID    (S_AXI_ARVALID),
    .AXI_ARREADY    (S_AXI_ARREADY),

    // AXI R channel
    .AXI_RDATA      (S_AXI_RDATA),
    .AXI_RVALID     (S_AXI_RVALID),
    .AXI_RRESP      (S_AXI_RRESP),
    .AXI_RREADY     (S_AXI_RREADY),

    // ASHI write-request registers
    .ASHI_WADDR     (ashi_waddr),
    .ASHI_WINDX     (ashi_windx),
    .ASHI_WDATA     (ashi_wdata),
    .ASHI_WRITE     (ashi_write),
    .ASHI_WRESP     (ashi_wresp),
    .ASHI_WIDLE     (ashi_widle),

    // ASHI read registers
    .ASHI_RADDR     (ashi_raddr),
    .ASHI_RINDX     (ashi_rindx),
    .ASHI_RDATA     (ashi_rdata),
    .ASHI_READ      (ashi_read ),
    .ASHI_RRESP     (ashi_rresp),
    .ASHI_RIDLE     (ashi_ridle)
);
//==========================================================================

endmodule