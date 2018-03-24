/*============================================================================================
*File name: pwm_hardware.v : This file conatains the basic logic for HArdware PWM detction.

* Author: Tejas Chavan (techavan@pdx.edu)
* First Release: 22nd Jan 2018
* Final release: 26th jan 2018

Description:  
   - pwm_hwd module provides the basic PWM detection functionality for a specific color.
   - It detectes the rising and falling edge by iimplementing the edge detection algorithm.
   - At every rising edge the count value is assigned to low_time and at every falling edge the 
     count value is assigneed to the high_time.
     Else the count keeps on incrementing
   - the pwm_hwd is instantiated thrice, ones each for red, green and blue.
================================================================================================*/
/*=================================================================================================
Module Declaration and port instantiation for the hardware PWM detection module
This module instantiates the basic functionality midule for all three Red, Green and Blue colors.
The outputs are connected to GPIO inputs to send the count value to the software for detetction
==================================================================================================*/
module pwm_hardware#(parameter DW = 32)(
    input clk,
    input red,
    input blue,
    input green,
    output  [DW-1:0] red_high_count,            // Final red high count
    output  [DW-1:0] red_low_count,             // Final red low count
    output  [DW-1:0] green_high_count,          // Final green high count
    output  [DW-1:0] green_low_count,           // Final red low count
    output  [DW-1:0] blue_high_count,           // Final blue high count
    output  [DW-1:0] blue_low_count             // Final red low count

    );
    

pwm_hwd DUTR(.clk(clk),                         // pwm_hwd instantiation for RED
            .color(red),
            .high_time(red_high_count),
            .low_time(red_low_count)
            );
   
pwm_hwd DUTG(.clk(clk),                         // pwm_hwd instantiation for GREEN
             .color(green),
             .high_time(green_high_count),
             .low_time(green_low_count)
            ); 
            
pwm_hwd DUTB(.clk(clk),                         // / pwm_hwd instantiation for BLUE
             .color(blue),
             .high_time(blue_high_count),
             .low_time(blue_low_count)
             );        
      
endmodule

/*=====================================================================================================
This module provides the basic functionality to caclulte the high and low time for a color
This is done by applying edge detection algorithm.
OOn Every rising edge the previous count is assigned to low_count and similar operation for high_count as well
=======================================================================================================*/
module pwm_hwd(output reg [31:0] high_time,             // High time for the signal
               output reg [31:0] low_time,               // Low time for the signal
               input color,                             // Input pwm signal, RED GREEN OR BLUE
               input clk);                              // 5MHz input clock

// Temporary variables for storing the count and previous pwm state                
reg [31:0] count = 0;
reg old_pwm = 0;    
// 
always @(posedge clk or posedge color or negedge color) // Initiated at clock change or change of pwm state
    begin
        if (color==0 && old_pwm==1)                 // When pwm change from high to low
            high_time<=count;                        // Pass the high count value
        else if (color==1 && old_pwm==0)                // When pwm changes from low to high
            low_time<=count;                         // Pass the low count value

        if (old_pwm!=color)
            count<=15'd0;                            // When pwm changes , reset the count to 0
            
         count<=count+1;                              // Increment the count at every positive edge of the clock
        
        old_pwm<=color;                              // Update the pwm state

    end
endmodule