// MIT License

// Copyright (c) 2022 Adam Gastineau

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

// A very simple audio i2s bridge to APF, based on their example code
module sound_i2s #(
    parameter CHANNEL_WIDTH = 15,
    parameter SIGNED_INPUT  = 0
) (
    input wire clk_74a,

    // Left and right audio channels. Can be in an arbitrary clock domain
    input wire [CHANNEL_WIDTH - 1:0] audio_l,
    input wire [CHANNEL_WIDTH - 1:0] audio_r,

    output reg audgen_mclk,
    output reg audgen_lrck,
    output reg audgen_dac
);
  //
  // audio i2s generator
  //

  reg audgen_nextsamp;

  // generate MCLK = 12.288mhz with fractional accumulator
  reg [21:0] audgen_accum;
  parameter [20:0] CYCLE_48KHZ = 21'd122880 * 2;
  always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if (audgen_accum >= 21'd742500) begin
      audgen_mclk  <= ~audgen_mclk;
      audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
  end

  // generate SCLK = 3.072mhz by dividing MCLK by 4
  reg [1:0] aud_mclk_divider;
  wire audgen_sclk = aud_mclk_divider[1]  /* synthesis keep*/;
  always @(posedge audgen_mclk) begin
    aud_mclk_divider <= aud_mclk_divider + 1'b1;
  end

  // shift out audio data as I2S
  // 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
  //
  // synchronize audio samples coming from the core

  localparam CHANNEL_RIGHT_HIGH = SIGNED_INPUT ? 16 : 15;
  localparam CHANNEL_LEFT_HIGH = 16 + CHANNEL_RIGHT_HIGH;

  wire [31:0] audgen_sampdata;

  assign audgen_sampdata[CHANNEL_LEFT_HIGH-1:CHANNEL_LEFT_HIGH-CHANNEL_WIDTH] = audio_l;
  assign audgen_sampdata[31-CHANNEL_WIDTH:16] = 0;
  assign audgen_sampdata[CHANNEL_RIGHT_HIGH-1:CHANNEL_RIGHT_HIGH-CHANNEL_WIDTH] = audio_r;
  assign audgen_sampdata[15-CHANNEL_WIDTH:0] = 0;

  generate
    if (~SIGNED_INPUT) begin
      // If not signed, make sure high bit is 0
      assign audgen_sampdata[31] = 0;
      assign audgen_sampdata[15] = 0;
    end
  endgenerate

  wire [31:0] audgen_sampdata_s;
  synch_3 #(
      .WIDTH(32)
  ) s5 (
      audgen_sampdata,
      audgen_sampdata_s,
      audgen_sclk
  );
  reg [31:0] audgen_sampshift;
  reg [ 4:0] audgen_lrck_cnt;
  always @(negedge audgen_sclk) begin
    // output the next bit
    audgen_dac <= audgen_sampshift[31];

    // 48khz * 64
    audgen_lrck_cnt <= audgen_lrck_cnt + 1'b1;
    if (audgen_lrck_cnt == 31) begin
      // switch channels
      audgen_lrck <= ~audgen_lrck;

      // Reload sample shifter
      if (~audgen_lrck) begin
        audgen_sampshift <= audgen_sampdata_s;
      end
    end else if (audgen_lrck_cnt < 16) begin
      // only shift for 16 clocks per channel
      audgen_sampshift <= {audgen_sampshift[30:0], 1'b0};
    end
  end

  initial begin
    // Verify parameters
    if (CHANNEL_WIDTH > 16) begin
      $error("CHANNEL_WIDTH must be <= 16. Received %d", CHANNEL_WIDTH);
    end

    if (SIGNED_INPUT != 0 && SIGNED_INPUT != 1) begin
      $error("SIGNED_INPUT must be 0 or 1. Received %d", SIGNED_INPUT);
    end

    if (CHANNEL_WIDTH == 16 && SIGNED_INPUT == 0) begin
      $error("Cannot have CHANNEL_WIDTH of 16 and an unsigned input");
    end
  end
endmodule
