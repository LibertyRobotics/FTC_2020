package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.Enums.ModulePosition;
import org.firstinspires.ftc.teamcode.Swerve.Enums.WheelDirection;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.IMU;

/**
 * Class used to control an individual swerve module
 * @author Will Richards, Zane Othman-Gomez, Avi Lance
 */
public class SwerveModule {

    //Holds the assigned module position be it left or right
    private ModulePosition modPos;

    //Gets A reference to the hardwareMap so motors can be mapped, aswell as a reference to the gamepad
    private HardwareMap hardwareMap;
    private Gamepad gamepad1;

    private Telemetry telemetry;

    //Gets references to the top and bottom motors
    private DcMotorEx TopSwerveMotor;
    private DcMotorEx BottomSwerveMotor;

    //Magnetic limit switch that is used to zero modules
    private DigitalChannel MagSwitch;

    private double motorSpeedOffset = 0.1;

    //Determines the offset to apply to the tickCounts
    private int topMotorTickOffset = 0;
    private int bottomMotorTickOffset = 0;

    public boolean hasResetEncoders = false;

    //Module control variables are created up here to not destroy the garbage collector
    private double currentRotation = 0;
    private double wantedRotation = 0;
    private WheelDirection wheelDirection = WheelDirection.STATIC;

    //Represents the distances in wheel rotations
    private double currentDistanceRot = 0;
    private double wantedDistanceRot = 0;

    //P = 3 or 5, or 8
    private final double P = Constants.TURN_P, I = Constants.TURN_I, D = Constants.TURN_D;

    //Reference to the PID class that allows us to use PID
    private PIDFCoefficients TeleOpPID;

    //Both deal with fine control of the swerve modules in autonomous
    private PIDFCoefficients turnPID;
    private PIDFCoefficients drivePID;

    //Power given to motors, in TeleOP
    private double power;

    //Power that will be set from the turnPID and drivePID commands respectively
    private double turnPower;
    private double drivePower;

    private List<DcmotorEx> motorList = new ArrayList<>();
    //private double motorSpeed = 0;

    /**
     * Constructs each variable as well as determining the module side so it can properly assign motors
     * @param modPos which side the module is located on
     * @param hardwareMap the mapping of names to ports
     * @param gamepad1 the controller input
     */
    public SwerveModule(ModulePosition modPos, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){

        this.telemetry = telemetry;

        /*
         * Assign parameters to class variables
         */
        this.modPos = modPos;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        /*
         * This block of code will determine which side the module is on based on the enum that was set in the SwerveController class
         */

        switch(modPos) {
            case RIGHT:
                TopSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "RightTopSwerveMotor");
                BottomSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "RightBottomSwerveMotor");
                MagSwitch = hardwareMap.get(DigitalChannel.class, "RightMagSwitch");
                break;
            case LEFT:
                TopSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "LeftTopSwerveMotor");
                BottomSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "LeftBottomSwerveMotor");
                MagSwitch = hardwareMap.get(DigitalChannel.class, "LeftMagSwitch");
                break;
            case CENTER:
                try {
                    TopSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "CenterTopSwerveMotor");
                    BottomSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "CenterBottomSwerveMotor");
                }
                catch (Exception e){
                    throw new RuntimeException("3rd Swerve Module Not Setup Yet");
                }
                break;
        }

        
        motorList.add(TopSwerveMotor);
        motorList.add(BottomSwerveMotor);

        TeleOpPID = new PIDFCoefficients(P,I,D,F);
        /*
         * Creates a new TeleOpPID and passes P, I and D into it.
         * After that it then sets the acceptable range which is the range +/- that it qualifies as at the right spot
         * And finally it sets the point the module is trying to reach, at this point we just set it to 0
         *
         * There is 2250 ticks in one rotatio

         **/
        for (DcMotorEx motor: motorList) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(TeleOpPID);
            motor.setTargetPositionTolerance(0.02*2250);
            motor.setVelocity(0);

        }
        //Tells the motors to run at a constant velocity, not just based on the motor values

        /*
         * Creates a new TeleOpPID and passes P, I and D into it.
         * After that it then sets the acceptable range which is the range +/- that it qualifies as at the right spot
         * And finally it sets the point the module is trying to reach, at this point we just set it to 0
         */

        TeleOpPID.setAcceptableRange(0.02);
        TeleOpPID.setSetpoint(0);
        TeleOpPID.setMaxOutput(0.75);


        /*
         * Create a new turnPID to specifically be used for autonomous turning of the robot
         */
        turnPID = new PID(P,I,D);
        turnPID.setAcceptableRange(0.02);
        turnPID.setSetpoint(0);

        /*
         * Create a new drivePID to be used for autonomous driving of the modules
         * TODO: Tune values
         */
        drivePID = new PID(Constants.DRIVE_P,Constants.DRIVE_I,Constants.DRIVE_D);
        drivePID.setAcceptableRange(0.1);
        drivePID.setSetpoint(0);

        resetBottomEncoder();
        resetTopEncoder();
    }

    //region Teleop Module Control
    /**
     * Method Uses The PID Controller / PID class to move the module to the right position and then once there will allow it to spin, robot centric
     */
    public void PIDControl() {

        double motorSpeed = 0.6;

        /*
         * Collect information to be used in module control / PID
         * currentRotation - The number of rotations he module has currently completed
         * wantedRotation - The normalized top hemisphere value of the joystick converted into rotations
         * wheelDirection - The direction the wheel should spin based off the Y axis of the joystick
         */
        currentRotation = SwerveMath.getModulePosition(getTopMotorTicks(),getBottomMotorTicks());
        wantedRotation = SwerveMath.normalizeJoystickAngle(gamepad1);


        wheelDirection = SwerveMath.getWheelDirection(gamepad1);

        telemetry.addData(modPos + " Current Rotation: ", currentRotation);


        /*
         * This small section simply updates the point that it wants to reach based off the new wantedRotation
         * And then the method calcOutput is called which you pass your current value into and it preforms the PID opperation and returns the output with your scalars
         */
        TeleOpPID.setSetpoint(wantedRotation);
        power = TeleOpPID.calcOutput(currentRotation);
        if(power<0)
            power -= 0.05;
        else
            power+=0.05;

        /*
         * This code is used to actively re-zero the modules while on the field
         * It takes the mag switches that are above the modules and uses those to re-zero the module when the start button is pressed
         * It then turns the modules at a very slow speed to realign with the magnetic limit switches
         * At which point it resets the encoders on the motors
         */
        if(gamepad1.start){
            if(MagSwitch.getState()){
                telemetry.addData("Mag Switch: ", MagSwitch.getState());
                if(modPos == ModulePosition.RIGHT) {
                    TopSwerveMotor.setPower(0.3);
                    BottomSwerveMotor.setPower(0.3);
                }
                else{
                    TopSwerveMotor.setPower(-0.3);
                    BottomSwerveMotor.setPower(-0.3);
                }
            }
            else{
                TopSwerveMotor.setPower(0);
                BottomSwerveMotor.setPower(0);
                resetBottomEncoder();
                resetTopEncoder();
            }
        }


        // If we weren't trying to zero the modules, drive them
        else {
            /*
             * This is where the bulk of the control code happens
             * It first checks to see if the control loop has reached its desired angle, if it has allow the module to drive forward, if it hasn't then keep trying
             * Next if it is "in range" and the driver is pressing the trigger, check which module it is and then depending on where the joystick is either run the wheels forward or backward
             */
            if(TeleOpPID.isInRange()){
                if(gamepad1.right_trigger > 0.1){
                    if(modPos == ModulePosition.RIGHT) {
                        if (wheelDirection == WheelDirection.FORWARD) {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        } else if (wheelDirection == WheelDirection.BACKWARD) {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        } else {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        }
                    }
                    else {
                        if (wheelDirection == WheelDirection.FORWARD) {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        } else if (wheelDirection == WheelDirection.BACKWARD) {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        } else {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        }
                    }

                }

                /*
                 * If it isn't trying to drive then check if it is trying to turn right
                 */
                else if(gamepad1.right_bumper) {
                    TopSwerveMotor.setPower(-motorSpeed);
                    BottomSwerveMotor.setPower(motorSpeed);
                }
                /*
                 * If not right then check left
                 */
                else if(gamepad1.left_bumper) {
                    TopSwerveMotor.setPower(motorSpeed);
                    BottomSwerveMotor.setPower(-motorSpeed);
                }

                /*
                 * Finally if it is aligned and we aren't trying to drive or turn stop the motors and don't do anything
                 */
                else {
                    TopSwerveMotor.setPower(0);
                    BottomSwerveMotor.setPower(0);
                }
            }

            /*
             * Now if the angle is outside the acceptable range feed the PID loop values into the motors to reach the range
             */
            else {
                TopSwerveMotor.setPower(power);
                BottomSwerveMotor.setPower(power);
            }
        }

    }

    /**
     * Method Uses The PID Controller / PID class to move the module to the right position and then once there will allow it to spin, field centric
     */
    public void PIDControl(IMU imu){

        double motorSpeed = 0.7;



        telemetry.addData(modPos + " Top Encoder: ", TopSwerveMotor.getCurrentPosition());
        telemetry.addData(modPos + " Bottom Encoder: ", BottomSwerveMotor.getCurrentPosition());

        /*
         * Collect information to be used in module control / PID
         * currentRotation - The number of rotations he module has currently completed
         * wantedRotation - The normalized top hemisphere value of the joystick converted into rotations
         * wheelDirection - The direction the wheel should spin based off the Y axis of the joystick
         */
        currentRotation = SwerveMath.getModulePosition(TopSwerveMotor.getCurrentPosition(), BottomSwerveMotor.getCurrentPosition());
        wantedRotation = SwerveMath.normalizeJoystickAngle(gamepad1, imu);
        wheelDirection = SwerveMath.getWheelDirection(gamepad1);

        telemetry.addData("Wanted Rotation (IMU): ", wantedRotation);

        /*
         * This small section simply updates the point that it wants to reach based off the new wantedRotation
         * And then the method calcOutput is called which you pass your current value into and it preforms the PID operation and returns the output with your scalars
         */
        TeleOpPID.setSetpoint(wantedRotation);
        power = TeleOpPID.calcOutput(currentRotation);

        /*
         * This code is used to actively re-zero the modules while on the field
         * It takes the mag switches that are above the modules and uses those to re-zero the module when the start button is pressed
         * It then turns the modules at a very slow speed to realign with the magnetic limit switches
         * At which point it resets the encoders on the motors
         */
        if(gamepad1.start){
            if(MagSwitch.getState()){
                if(modPos == ModulePosition.RIGHT) {
                    TopSwerveMotor.setPower(0.15);
                    BottomSwerveMotor.setPower(0.15);
                }
                else{
                    TopSwerveMotor.setPower(-0.15);
                    BottomSwerveMotor.setPower(-0.15);
                }
            }
            else{
                TopSwerveMotor.setPower(0);
                BottomSwerveMotor.setPower(0);
                resetBottomEncoder();
                resetTopEncoder();
            }
        }


        //If we weren't trying to zero the modules
        else{

            /*
             * This is where the bulk of the control code happens
             * It first checks to see if the control loop has reached its desired angle, if it has allow the module to drive forward, if it hasn't then keep trying
             * Next if it is "in range" and the driver is pressing the trigger, check which module it is and then depending on where the joystick is either run the wheels forward or backward
             */
            if(TeleOpPID.isInRange()){
                if(gamepad1.right_trigger > 0.1){
                    if(modPos == ModulePosition.RIGHT) {
                        if (wheelDirection == WheelDirection.FORWARD) {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        } else if (wheelDirection == WheelDirection.BACKWARD) {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        } else {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        }
                    }
                    else {
                        if (wheelDirection == WheelDirection.FORWARD) {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        } else if (wheelDirection == WheelDirection.BACKWARD) {
                            TopSwerveMotor.setPower(motorSpeed);
                            BottomSwerveMotor.setPower(-motorSpeed);
                        } else {
                            TopSwerveMotor.setPower(-motorSpeed);
                            BottomSwerveMotor.setPower(motorSpeed);
                        }
                    }

                }

                /*
                 * If it isn't trying to drive then check if it is trying to turn right
                 */
                else if(gamepad1.right_bumper) {
                    TopSwerveMotor.setPower(-motorSpeed);
                    BottomSwerveMotor.setPower(motorSpeed);
                }
                /*
                 * If not right then check left
                 */
                else if(gamepad1.left_bumper) {
                    TopSwerveMotor.setPower(motorSpeed);
                    BottomSwerveMotor.setPower(-motorSpeed);
                }

                /*
                 * Finally if it is aligned and we aren't trying to drive or turn stop the motors and don't do anything
                 */
                else {
                    TopSwerveMotor.setPower(0);
                    BottomSwerveMotor.setPower(0);
                }
            }

            /*
             * Now if the angle is outside the acceptable range feed the PID loop values into the motors to reach the range
             */
            else{

                TopSwerveMotor.setPower(power);
                BottomSwerveMotor.setPower(power);
            }
        }
    }

    //endregion

    //region Autonomous Module Control
    /**
     * Overloaded autoPIDControl used for autonomous control
     * This method works in such a way that since it may take multiple loops of the program to align each time through this method will return true or false
     * If the module is oriented right it will return true and vice versa, this means that you will have to check this every iteration through the program and ignore it if the action has completed
     * @return the status of the completeness of the control
     */
    public boolean autoPIDControl(double angle, double distance, double maxMotorSpeed){
//        if(!hasResetEncoders) {
//            resetBottomEncoder();
//            resetTopEncoder();
//            hasResetEncoders = true;
//        }

        /*
         * Collect information to be used in module control / PID
         * currentRotation - The number of rotations he module has currently completed
         * wantedRotation - The normalized values of the angle
         * wheelDirection - The direction the wheel should spin based off the angle passed as a parameter
         */
        currentRotation = SwerveMath.getModulePosition(getTopMotorTicks(), getBottomMotorTicks());
        wantedRotation = SwerveMath.normalizeAngle(angle);
        wheelDirection = SwerveMath.getAutoWheelDirection(angle);

        /*
         * Calculate the distances needed to rotate offset from the current
         * currentDistanceRot - The current # of rotations already completed
         * wantedDistanceRot - The new distance value we want to reach
         */
        currentDistanceRot = SwerveMath.getWheelPosition(getTopMotorTicks(), getBottomMotorTicks());
        wantedDistanceRot = SwerveMath.calculateWheelPosition(distance);

        if (modPos == ModulePosition.LEFT){
            wantedDistanceRot *= -1;
        }


        //wantedDistanceRot = SwerveMath.calculateWheelPosition(distance);

        if(modPos == ModulePosition.RIGHT) {
            telemetry.addData("Right Module Wanted Distance: ", wantedDistanceRot);
            telemetry.addData("Right Module Current Distance: ", currentDistanceRot);
        }
        else{
            telemetry.addData("Left Module Wanted Distance: ", wantedDistanceRot);
            telemetry.addData("Left Module Current Distance: ", currentDistanceRot);
        }

        /*
         * This small section simply updates the point that it  wants to reach based off the new wantedRotation
         * And then the method calcOutput is called which you pass your current value into and it preforms the PID opperation and returns the output with your scalars
         */
        turnPID.setSetpoint(wantedRotation);
        turnPower = turnPID.calcOutput(currentRotation);



        /*
         * This section sets the position we want to reach with the wheels and the maxMotorSpeed
         */
        drivePID.setSetpoint(wantedDistanceRot);
        drivePID.setMaxOutput(maxMotorSpeed);

        drivePower = drivePID.calcOutput(currentDistanceRot);

        if(modPos == ModulePosition.RIGHT)
            drivePower *= -1;

        /*
         * Runs the motors to a given position and then stops
         */
        if(turnPID.isInRange()){

            //Checks to see if the drive loop is not at the right spot and if not then keep trying if it is then return the command as true to stop this loop
            if(!drivePID.isInRange()) {
                if (modPos == ModulePosition.RIGHT) {
                    if (wheelDirection == WheelDirection.FORWARD) {
                        TopSwerveMotor.setPower(drivePower);
                        BottomSwerveMotor.setPower(-drivePower);
                    } else if (wheelDirection == WheelDirection.BACKWARD) {
                        TopSwerveMotor.setPower(-drivePower);
                        BottomSwerveMotor.setPower(drivePower);
                    } else {
                        TopSwerveMotor.setPower(drivePower);
                        BottomSwerveMotor.setPower(-drivePower);
                    }
                } else {
                    if (wheelDirection == WheelDirection.FORWARD) {
                        TopSwerveMotor.setPower(-drivePower);
                        BottomSwerveMotor.setPower(drivePower);
                    } else if (wheelDirection == WheelDirection.BACKWARD) {
                        TopSwerveMotor.setPower(drivePower);
                        BottomSwerveMotor.setPower(-drivePower);
                    } else {
                        TopSwerveMotor.setPower(-drivePower);
                        BottomSwerveMotor.setPower(drivePower);
                    }
                }
            }
            else {
                telemetry.addData(modPos == ModulePosition.RIGHT ? "Right Module Finished: " : "Left Module Finished: ", true);
                stopMotors();
                return true;
            }
        }

        //If it hasn't reached the turn set point yet keep turning
        else {
            TopSwerveMotor.setPower(turnPower);
            BottomSwerveMotor.setPower(turnPower);
        }

        //Hasn't reached the right point yet
        return false;
    }

    /**
     * Method for running module motors, albeit with scaled power.
     * This allows the power of the motors to be tuned according to external factors,
     * like other PID loops or sensor inputs.
     * @param angle The angle to run the module at.
     * @param scaleFactor How much to scale the power based on external factors.
     */
    public void PIDControlScaled(double angle, double scaleFactor){

        /*
         * Collect information to be used in module control / PID
         * currentRotation - The number of rotations he module has currently completed
         * wantedRotation - The normalized values of the angle
         * wheelDirection - The direction the wheel should spin based off the angle passed as a parameter
         */
        currentRotation = SwerveMath.getModulePosition(getTopMotorTicks(), getBottomMotorTicks());
        wantedRotation = SwerveMath.normalizeAngle(angle);
        wheelDirection = SwerveMath.getWheelDirection(angle);

        /*
         * This small section simply updates the point that it wants to reach based off the new wantedRotation
         * And then the method calcOutput is called which you pass your current value into and it preforms the PID operation and returns the output with your scalars
         */
        TeleOpPID.setSetpoint(wantedRotation);
        power = TeleOpPID.calcOutput(currentRotation);

        /*
         * This is what the module will do once it has reached the wanted position
         */

        if(TeleOpPID.isInRange()){
            if(modPos == ModulePosition.RIGHT) {
                if (wheelDirection == WheelDirection.FORWARD) {
                    TopSwerveMotor.setPower(1 * scaleFactor);
                    BottomSwerveMotor.setPower(-1 * scaleFactor);
                } else if (wheelDirection == WheelDirection.BACKWARD) {
                    TopSwerveMotor.setPower(-1 * scaleFactor);
                    BottomSwerveMotor.setPower(1 * scaleFactor);
                } else {
                    TopSwerveMotor.setPower(1 * scaleFactor);
                    BottomSwerveMotor.setPower(-1 * scaleFactor);
                }
            }
            else{
                if (wheelDirection == WheelDirection.FORWARD) {
                    TopSwerveMotor.setPower(-1 * scaleFactor);
                    BottomSwerveMotor.setPower(1 * scaleFactor);
                } else if (wheelDirection == WheelDirection.BACKWARD) {
                    TopSwerveMotor.setPower(1 * scaleFactor);
                    BottomSwerveMotor.setPower(-1 * scaleFactor);
                } else {
                    TopSwerveMotor.setPower(-1 * scaleFactor);
                    BottomSwerveMotor.setPower(1 * scaleFactor);
                }
            }
        }
        else{
            TopSwerveMotor.setPower(power);
            BottomSwerveMotor.setPower(power);
        }
    }

    /**
     * This method works similarly to the last except it will just run the motors in a direction that is wanted at a certain speed
     * @param wheelDirection the direction that the wheel is meant to move
     * @param motorSpeed the speed you want the motors to run, run directon is determined from that
     */
    public void runMotorsDumb(WheelDirection wheelDirection, double motorSpeed){
        if(modPos == ModulePosition.RIGHT) {
            if (wheelDirection == WheelDirection.FORWARD) {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            } else if (wheelDirection == WheelDirection.BACKWARD) {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            } else {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            }
        }
        else{
            if (wheelDirection == WheelDirection.FORWARD) {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            } else if (wheelDirection == WheelDirection.BACKWARD) {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            } else {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            }
        }
    }

    /**
     * This method works similarly to the last except it will just run the motors in a direction that is wanted at a certain speed
     * @param angle The angle at which you want the wheels to run
     * @param motorSpeed the speed you want the motors to run, run directon is determined from that
     */
    public void runMotorsDumbWithAngle(double angle, double motorSpeed){
        // Determine the wheel direction based on the angle parameter
        wheelDirection = SwerveMath.getWheelDirection(angle);

        if(modPos == ModulePosition.RIGHT) {
            if (wheelDirection == WheelDirection.FORWARD) {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            } else if (wheelDirection == WheelDirection.BACKWARD) {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            } else {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            }
        }
        else{
            if (wheelDirection == WheelDirection.FORWARD) {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            } else if (wheelDirection == WheelDirection.BACKWARD) {
                TopSwerveMotor.setPower(motorSpeed);
                BottomSwerveMotor.setPower(-motorSpeed);
            } else {
                TopSwerveMotor.setPower(-motorSpeed);
                BottomSwerveMotor.setPower(motorSpeed);
            }
        }
    }

    /**
     * Method used to feed changes in angle and power to the modules
     * @param angle the angle the modules need to be facing
     * @param motorSpeed the speed the motors are set to drive at
     */
    public void activeDrive(double angle, double motorSpeed){

        /**
         * Collect information to be used in module control / PID
         * currentRotation - The number of rotations he module has currently completed
         * wantedRotation - The normalized values of the angle
         * wheelDirection - The direction the wheel should spin based off the angle passed as a parameter
         */
        currentRotation = SwerveMath.getModulePosition(getTopMotorTicks(), getBottomMotorTicks());
        wantedRotation = SwerveMath.normalizeAngle(angle);
        wheelDirection = SwerveMath.getWheelDirection(angle);

        turnPID.setSetpoint(wantedRotation);
        turnPower = turnPID.calcOutput(currentRotation);

        /**
         * Runs the motors to a given position and then stops
         */
        if(turnPID.isInRange()) {
            //Checks to see if the drive loop is not at the right spot and if not then keep trying if it is then return the command as true to stop this loop
            if (modPos == ModulePosition.RIGHT) {
                if (wheelDirection == WheelDirection.FORWARD) {
                    TopSwerveMotor.setPower(motorSpeed);
                    BottomSwerveMotor.setPower(-motorSpeed);
                } else if (wheelDirection == WheelDirection.BACKWARD) {
                    TopSwerveMotor.setPower(-motorSpeed);
                    BottomSwerveMotor.setPower(motorSpeed);
                } else {
                    TopSwerveMotor.setPower(motorSpeed);
                    BottomSwerveMotor.setPower(-motorSpeed);
                }
            } else {
                if (wheelDirection == WheelDirection.FORWARD) {
                    TopSwerveMotor.setPower(-motorSpeed);
                    BottomSwerveMotor.setPower(motorSpeed);
                } else if (wheelDirection == WheelDirection.BACKWARD) {
                    TopSwerveMotor.setPower(motorSpeed);
                    BottomSwerveMotor.setPower(-motorSpeed);
                } else {
                    TopSwerveMotor.setPower(-motorSpeed);
                    BottomSwerveMotor.setPower(motorSpeed);
                }
            }
        }

        //If it hasn't reached the turn set point yet keep turning
        else {
            TopSwerveMotor.setPower(turnPower);
            BottomSwerveMotor.setPower(turnPower);
        }
    }

    /**
     * Stops both motors in a given swerve module.
     */
    public void stopMotors() {
        TopSwerveMotor.setPower(0);
        BottomSwerveMotor.setPower(0);
    }

    //endregion

    /**
     * Gets the number of ticks including the zeroed value for the top motor
     * @return the zeroed number of ticks
     */
    public int getTopMotorTicks(){
        return TopSwerveMotor.getCurrentPosition()-topMotorTickOffset;
    }

    /**
     * Gets the zeroed number of ticks for the bottom motor
     * @return the zeroed number of ticks
     */
    public int getBottomMotorTicks(){
        return BottomSwerveMotor.getCurrentPosition()-bottomMotorTickOffset;
    }

    /**
     * Pseudo reset the top encoder value
     */
    public void resetTopEncoder(){

        topMotorTickOffset = TopSwerveMotor.getCurrentPosition();
    }

    /**
     * Pseudo reset the bottom encoder value
     */
    public void resetBottomEncoder(){
        bottomMotorTickOffset = BottomSwerveMotor.getCurrentPosition();
    }

    /**
     * Used to ramp the motor speed to a setpoint
     * @param rampRate the rate at which we will increase to reach it
     * @param setPower the point we are trying to reach
     * @return the current speed for the motor
     */
    private double rampMotorSpeed(double setPower, double rampRate, double currentPower){
        if(setPower > currentPower){
            currentPower+=rampRate;
            return currentPower;
        }
        else if(setPower < currentPower){
            currentPower-=rampRate;
            return currentPower;
        }
        else {
            return setPower;
        }
    }

    /**
     * Reset both module encoders
     */
    public void resetEncoders(){

    }

}
