package org.firstinspires.ftc.teamcode.Swerve;


/**
 * Class used to calculate the wheel angles and speeds
 *
 * Math Derived from this CD post: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
 *
 * @author Will Richards
 */
class SwerveMath {

    // Constants regarding the robots physical setup
    private static double WHEEL_BASE = 0;
    private static double TRACK_WIDTH = 0;

    // Calculated value from the track width and wheel base
    private static double R = 0;
    private static boolean fieldCentric = false;

    /**
     * Calculates the intermediate variables to simplify math
     * @return array containing the 4 variables
     */
    private static double[] calculateIntermediaries(double FWD, double STR, double RCW){
        double[] intermidates = new double[4];

        // A
        intermidates[0] = STR - RCW * (WHEEL_BASE/R);

        // B
        intermidates[1] = STR + RCW * (WHEEL_BASE/R);

        // C
        intermidates[2] = FWD - RCW * (TRACK_WIDTH/R);

        // D
        intermidates[3] = FWD + RCW * (TRACK_WIDTH/R);

        return intermidates;
    }

    /**
     * Takes in the FWD and STR from the input and then calculates field centric values
     * @param FWD the forward vectors
     * @param STR the strafe vector
     * @param gyroAngle 360 degree angle from the gyro that is zeroed at straight down the field
     * @return array containing new values in the same order they were passes
     */
    public static double[] calculateControlVectors(double FWD, double STR, double gyroAngle){
        double[] vectors = new double[2];

        // FWD
        vectors[0] = FWD*Math.cos(gyroAngle) + STR * Math.sin(gyroAngle);

        // STR
        vectors[1] = -FWD*Math.sin(gyroAngle) + STR*Math.cos(gyroAngle);

        return vectors;
    }

    /**
     * Calculate the speeds for each of the swerve module wheels, and then normalize and return the results
     * @param FWD the forward power vector
     * @param STR the strafe power vector
     * @param RCW the rotation vector
     * @return the speeds for each wheel to be commanded to run at
     */
    public static double[] calculateWheelSpeeds(double FWD, double STR, double RCW){
        double[] wheelSpeeds = new double[4];
        double[] intermediates = calculateIntermediaries(FWD, STR, RCW);
        double max = 0;

        //Front Right
        wheelSpeeds[0] = Math.sqrt(((Math.pow(intermediates[1],2))+(Math.pow(intermediates[2],2))));

        //Front Left
        wheelSpeeds[1] = Math.sqrt(((Math.pow(intermediates[1],2))+(Math.pow(intermediates[3],2))));

        //Back Left
        wheelSpeeds[2] = Math.sqrt(((Math.pow(intermediates[0],2))+(Math.pow(intermediates[3],2))));

        //Back Right
        wheelSpeeds[3] = Math.sqrt(((Math.pow(intermediates[0],2))+(Math.pow(intermediates[2],2))));

        // Normalize results, effectively clamp the values between 0 and 1 to between 0 and +1
        max = wheelSpeeds[0];
        if(wheelSpeeds[1]>max)
            max=wheelSpeeds[1];
        if(wheelSpeeds[2]>max)
            max=wheelSpeeds[2];
        if(wheelSpeeds[3]>max)
            max=wheelSpeeds[3];
        if(max >1) {
            wheelSpeeds[0] /= max;
            wheelSpeeds[1] /= max;
            wheelSpeeds[2] /= max;
            wheelSpeeds[3] /= max;
        }

        return wheelSpeeds;
    }

    /**
     * Calculate the angles the wheels should be facing to drive the correct direction
     * @param FWD the forward power vector
     * @param STR the strafe power vector
     * @param RCW the rotation power vector
     * @return the angles the swerve modules should be facing
     */
    public static double[] calculateWheelAngles(double FWD, double STR, double RCW){
        double[] wheelAngles = new double[4];
        double[] intermediates = calculateIntermediaries(FWD, STR, RCW);

        //Front Right
        wheelAngles[0] = Math.toDegrees(Math.atan2(intermediates[1], intermediates[2]));

        //Front Left
        wheelAngles[1] = Math.toDegrees(Math.atan2(intermediates[1], intermediates[3]));

        //Back Left
        wheelAngles[2] = Math.toDegrees(Math.atan2(intermediates[0], intermediates[3]));

        //Back Right
        wheelAngles[3] = Math.toDegrees(Math.atan2(intermediates[0], intermediates[2]));

        return wheelAngles;
    }

    /**
     * Convert the angle for a modules from the true angle to motor ticks
     * @return the altered angle
     */
    public static double convertAnglesToTicks(double angle, double gearRatio){
        angle *= (2240/360.0) * gearRatio;

        return angle;
    }

    /**
     * Sets the physical robot wheel base
     * @param wheelBase measurement of robot wheelbase
     */
    public static void setWheelBase(double wheelBase){
        WHEEL_BASE = wheelBase;
        if(TRACK_WIDTH > 0 && WHEEL_BASE > 0){
            // Calculate R which is just sqrt(wheelbase^2 + trackwidth^2)
            R = Math.sqrt(((Math.pow(WHEEL_BASE, 2))+(Math.pow(TRACK_WIDTH, 2))));
        }
    }

    /**
     * Sets the physical robot track width
     * @param trackWidth robot track width
     */
    public static void setTrackWidth(double trackWidth){
        TRACK_WIDTH = trackWidth;
        if(TRACK_WIDTH > 0 && WHEEL_BASE > 0){

            // Calculate R which is just sqrt(wheelbase^2 + trackwidth^2)
            R = Math.sqrt(((Math.pow(WHEEL_BASE, 2))+(Math.pow(TRACK_WIDTH, 2))));
        }
    }

    /**
     * Sets the control style of the swerve
     * @param isFieldCentric boolean representing the control style
     */
    public static void setFieldCentric(boolean isFieldCentric){
        fieldCentric = isFieldCentric;
    }

}
