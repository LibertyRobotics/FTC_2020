package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSystemController;
import org.firstinspires.ftc.teamcode.Subsystems.StoneGripController;
import org.firstinspires.ftc.teamcode.Swerve.SwerveController;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;
/**
 * Main TeleOp Mode to be used during comp
 * @author Will Richards, Troy Lopez, Zane Othman-Gomez
 */

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {

    //Creates a new swerve controller
    private SwerveController swerveController;

    private DcMotor gripMotor;
    private DcMotor vertMotor;


    // This is for the stone grabbing arm with an elevator
    private StoneGripController stoneGripController;

    private Boolean armToggle = false;

    @Override
    public void init() {
        //Initialize the swerve controller
        swerveController = new SwerveController(gamepad1, hardwareMap, telemetry, IMUOrientation.VERTICAL, false);

        vertMotor = hardwareMap.get(DcMotor.class, "VerticalMotor");
        gripMotor = hardwareMap.get(DcMotor.class, "GripMotor");


        // Instantiate a controller for the stone-grabbing arms
        stoneGripController = new StoneGripController(hardwareMap, gamepad2, telemetry);

        //Zero the position of the modules at init
        swerveController.zeroModules();
    }

    @Override
    public void loop() {
        //Control the swerve modules
        swerveController.controlModules();

        // Control the two different stone-gripping arms with switching between them supported
        stoneGripController.controlArms();

        if (gamepad2.dpad_up){
            vertMotor.setPower(0.7);
        }
        else if (gamepad2.dpad_down){
            vertMotor.setPower(-0.7);
        }
        else{
            vertMotor.setPower(0);
        }

        if (gamepad2.dpad_left){
            gripMotor.setPower(0.6);
        }
        else if (gamepad2.dpad_right) {
            gripMotor.setPower(-0.6);
        }
        else{
            gripMotor.setPower(0);
        }

    }
}