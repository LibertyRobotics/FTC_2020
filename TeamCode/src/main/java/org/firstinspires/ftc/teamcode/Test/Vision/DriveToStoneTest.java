package org.firstinspires.ftc.teamcode.Test.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Utilites.StoneApproach;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.VuforiaWrapper;

@Autonomous(name = "Drive to Skystone Test", group = "Test")
public class DriveToStoneTest extends OpMode {
    private VuforiaWrapper vuforia;
    private SwerveController swerveController;

    // Z Offset to be relative to the stone
    private double zOffset = 10;

    private double driveAngle;
    private boolean hasRun = false;

    // Code to approach stone
    private StoneApproach approach;

    @Override
    public void init() {
        // Initialize the controller for the swerve modules
        swerveController = new SwerveController(hardwareMap, telemetry);

        vuforia = new VuforiaWrapper(hardwareMap);

        // Initialize class facilitating the approach of a stone
        approach = new StoneApproach(vuforia, swerveController);
    }

    @Override
    public void init_loop() {
        vuforia.initVuforia();
    }

    @Override
    public void loop() {
        if(!hasRun) {
            //hasRun = approach.approachStone(zOffset, 20);
        }
    }
}