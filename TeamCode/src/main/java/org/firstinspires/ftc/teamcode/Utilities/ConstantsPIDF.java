package org.firstinspires.ftc.teamcode.Utilities;

public class ConstantsPIDF {
        // Turning PID constants
        //3520 ticks/s is the Max velocity on a full battery
        private static final double MAX_VELOCITY = 3520;
        public static final double TURN_P = TURN_F*.1;
        public static final double TURN_I = TURN_P*.1;
        public static final double TURN_D = 0;
        public static final double TURN_F = 32767/MAX_VELOCITY;

        // Driving PID constants
        public static final double DRIVE_P = 0.4;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0.01;

    }
