package org.firstinspires.ftc.teamcode.Autonomous.OpModes.BlueAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.StoneGripController;
import org.firstinspires.ftc.teamcode.Subsystems.Utilities.GripArmPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Utilities.MoveArmDirection;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;


@Autonomous

public class BlueMiddleStone extends OpMode {
    private SwerveController swerve;
    private StoneGripController stoneArms;
    private Servo LeftBlockGrip;
    private Servo RightBlockGrip;

    // This list is used for progression in actions executed.
    //private boolean[] actionCompletions = {false, false, false, false, false, false, false};

    private boolean ran1 = false;
    private boolean ran2 = false;
    private boolean ran3 = false;
    private boolean ran4 = false;

    @Override
    public void init() {
        // Initialize the swerve module controller
        swerve = new SwerveController(null, hardwareMap, telemetry, IMUOrientation.VERTICAL, false);

        RightBlockGrip = hardwareMap.get(Servo.class, "svGrip");
        LeftBlockGrip = hardwareMap.get(Servo.class, "LeftBlockGrip");

        // Initialize the controller for the stone-grabbing arms
        stoneArms = new StoneGripController(hardwareMap, null, telemetry);
    }

    @Override
    public void loop() {
        // Drive up to the stone.
        while (ran1 == false) {
            stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.DOWN, 1000);
            ran1 = swerve.autoControlModules(0, 26, 0.25);

        }
        while (ran2 == false) {
            stoneArms.ungripArm(GripArmPosition.LEFT);
            stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.DOWN, 1000);
            stoneArms.gripArm(GripArmPosition.LEFT);
            stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.UP, 1000);
            // toFoundation
            ran2 = swerve.autoControlModules(270, 56, 0.5);
        }


        // Lower arms, release stone and grab foundation
        while (ran3 != true) {
            // Drop stone and left arm
            stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.DOWN, 1000);
            stoneArms.ungripArm(GripArmPosition.LEFT);

            // Drop left arm
            stoneArms.autoPivot(GripArmPosition.RIGHT, MoveArmDirection.DOWN, 1000);
            ran4 = swerve.autoControlModules(180, 26, .3);
        }


        // Raise arms so we don't take the foundation with us
        while (ran4 != true) {
            stoneArms.autoPivot(GripArmPosition.RIGHT, MoveArmDirection.UP, 1000);
            stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.UP, 1000);
            ran4 =  swerve.autoControlModules(90, 40, 0.5);
        }
        // Stop the op mode
        requestOpModeStop();


    }
}

