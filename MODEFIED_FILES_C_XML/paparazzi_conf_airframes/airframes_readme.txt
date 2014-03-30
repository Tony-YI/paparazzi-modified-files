Changed:
2013_7_13----------1.using KILL_SWITCH to arm the motors instead of zero THROTTLE and full YAW.
                   2.remove the <configure name="AHRS_PROPAGATE_FREQUENCY" value="500"/> line since the default value is 500 already.

2013_7_27----------1.change the PID values of horizontal_loop and vertical_loop
                   2.change the maximum angle of horizontal_loop and attitude_loop
