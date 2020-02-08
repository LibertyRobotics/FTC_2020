package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Swerve.Enums.WheelDirection;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.IMU;

/**
 * Created to house any math involved in controlling the diff. swerve
 * @author Will Richards
 */
public class SwerveMath {

    public static final double WHEEL_DIAMETER = 2.5; //inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;


    /**
     * TODO: Convert angles to field centric variants
     * Static method used to normalize the joystick
     * Y is inverted because for some strange reason someone decided that up on the joystick should be negative
     * @param gamepad1 the primary driving controller
     * @return the angle in terms of module rotations
     */
    public static double normalizeJoystickAngle(Gamepad gamepad1){

        double alteredAngle = 0;
        double trueAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y*-1, gamepad1.left_stick_x));

        //Centered
        if(gamepad1.left_stick_x==0 && gamepad1.left_stick_y*-1==0)
            alteredAngle = 0;

        //Anywhere in the top hemisphere (including 90 on the right)
        else if(trueAngle>=0 && trueAngle<=180)
            alteredAngle = 90-trueAngle;

        //90 on the left
        else if(gamepad1.left_stick_x<=0&&gamepad1.left_stick_y*-1==0)
            alteredAngle = -90;

        //Bottom Left Quad.
        else if(gamepad1.left_stick_y*-1<=0&&gamepad1.left_stick_x<=0)
            alteredAngle = 90-(trueAngle+180);

        //Bottom Right Quad.
        else if(gamepad1.left_stick_y*-1<=0&&gamepad1.left_stick_x>=0)
            alteredAngle =  (-90)-trueAngle;

        //If none of those applied its okay to return the current angle
        else {
            alteredAngle = trueAngle;
        }

        //Convert to rotations
        return alteredAngle/360;
    }

    /**
     * Static overload used to angle the module based on realtionship to the field
     * Y is inverted because for some strange reason someone decided that up on the joystick should be negative
     * @param gamepad1 the primary driving controller
     * @return the angle in terms of module rotations
     */
    public static double normalizeJoystickAngle(Gamepad gamepad1, IMU imu){

        double alteredAngle = 0;
        double trueAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y*-1, gamepad1.left_stick_x));

        trueAngle = trueAngle + imu.getHeading();

        //Centered
        if(gamepad1.left_stick_x==0 && gamepad1.left_stick_y*-1==0)
            alteredAngle = 0;

            //Anywhere in the top hemisphere (including 90 on the right)
        else if(trueAngle>=0 && trueAngle<=180)
            alteredAngle = 90-trueAngle;

            //90 on the left
        else if(gamepad1.left_stick_x<=0&&gamepad1.left_stick_y*-1==0)
            alteredAngle = -90;

            //Bottom Left Quad.
        else if(gamepad1.left_stick_y*-1<=0&&gamepad1.left_stick_x<=0)
            alteredAngle = 90-(trueAngle+180);

            //Bottom Right Quad.
        else if(gamepad1.left_stick_y*-1<=0&&gamepad1.left_stick_x>=0)
            alteredAngle =  (-90)-trueAngle;

            //If none of those applied its okay to return the current angle
        else {
            alteredAngle = trueAngle;
        }

        //Convert to rotations
        return alteredAngle/360;
    }

    /**
     * Static method used to normalize a given angle
     * @return the angle in terms of module rotations
     */
    /*
    public static double normalizeAngle(double angle){

        double alteredAngle = 0;
        // FIXME: Isn't this trueAngle variable redundant? We could just rename the parameter to achieve the same effect
        double trueAngle = angle;
        if(angle==0)
            alteredAngle = 0;
        else if(trueAngle>=0 && trueAngle<=180)
            alteredAngle = 90-trueAngle;
        else {
            alteredAngle = trueAngle;
        }

        return alteredAngle/360;
    }
    */
    public static double normalizeAngle(double angle) {
        double alteredAngle = smallestPeriodicAngle(angle);

        return alteredAngle/360;
    }

    /**
     * A method for taking an angle greater than 360 degrees and turning it into a similar angle
     * that is actually within the unit circle.
     */
    private static double smallestPeriodicAngle(double angle) {
        if (angle >= 360) {
            return smallestPeriodicAngle(angle - 360);
        } else if (angle <= -360) {
            return smallestPeriodicAngle(angle + 360);
        } else {
            return angle;
        }
    }


    /**
     * Returns the direction the wheel should be running based on the current Y axis, not really math but whatever
     * @param gamepad1 ref. to the primary gamepad
     * @return the direction the wheel should run
     */
    public static WheelDirection getWheelDirection(Gamepad gamepad1){
        if(gamepad1.left_stick_y*-1 == 0){
            return WheelDirection.STATIC;
        }
        else if (gamepad1.left_stick_y*-1 < 0){
            return WheelDirection.BACKWARD;
        }
        else{
            return WheelDirection.FORWARD;
        }
    }

    /**
     * Returns the direction the wheel should be running based on the current angle
     * @return the direction the wheel should run
     */
    public static WheelDirection getWheelDirection(double angle){
        if (angle < 0){
            return WheelDirection.BACKWARD;
        }
        else {
            return WheelDirection.FORWARD;
        }
    }

    /**
     * Calculates the current module rotation from the current tick count
     * @param topMotorTicks number of total ticks on top
     * @param bottomMotorTicks number of total ticks on bottom
     * @return the position in % of a rotation
     */
    public static double getModulePosition(int topMotorTicks, int bottomMotorTicks){
        return (((double)topMotorTicks+(double)bottomMotorTicks)/2250);
    }

    /**
     * Calculates the current rotation count of the wheel
     * @param topMotorTicks the number of encoder ticks on the top motors
     * @param bottomMotorTicks the number of encoder ticks on the bottom motor
     * @return the current positional value of the wheel
     */
    public static double getWheelPosition(int topMotorTicks, int bottomMotorTicks){
        return ((-topMotorTicks+bottomMotorTicks)/548.75);
    }

    /**
     * Calculate the required wheel rotations, assuming the motor encoders have already been reset.
     * @param distance the distance needed to move in inches
     * @return the distance in terms of rotations
     */
    public static double calculateWheelPosition(double distance){
       return (distance / WHEEL_CIRCUMFERENCE);
    }

}
