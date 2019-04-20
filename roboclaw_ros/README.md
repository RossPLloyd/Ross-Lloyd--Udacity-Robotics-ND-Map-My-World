# Forked on 21/10/2016
Fork of https://github.com/sonyccd/roboclaw_ros  
Original driver was buggy.  
Added thread to manage both odom and command in the same time.   
WIP, please use at your own risk  

# Changes
26/10/2016  
Added acceleration limit  

25/10/2016  
Split max speed in max linear and angular  
Added custom msg. Needed to modify name of script  
Publish commanded wheels speeds and motors currents readings  

21/10/2016  
Added thread usage for simultaneous odom and command  
Watchdog hack using SpeedDistanceM1M2 function   
Inverted l/r motor command  
Added Currents feedback on rosdebug  

