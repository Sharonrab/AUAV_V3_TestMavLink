# 
AUAV_V3_TestMavLink is a Mavlink simulink model for AUAV3 pilot (Developed by Avi Manor)
SLUGS is an AUtoPilot simulink model (Developed by Mariano Lizarraga)
We are using both models as a baseline software for full integration of SLUGS solution on new target (AUAV3).

5.17.16

1 - Commented out a number of GoTo blocks in the model to remove the warnings generated, replaced the commented blocks with sinks. 
    These need to be sorted in the future.

2 - A demux was modified from 2 to 3 outputs for the height PID controller, the controller is actually a PI controller but the function
    call used to retrieve the gains for the controller is deisgned to return 3 variables. I've added a sink for the third (D-gain) variable.

3 - Created a config.xml file to include all the necessary directories from the XC16 compiler, mavlink, and clib.

4 - Currently incurring a build error due to the model's "_data.o" 