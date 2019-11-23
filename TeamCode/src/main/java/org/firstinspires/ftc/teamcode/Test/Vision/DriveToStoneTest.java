package org.firstinspires.ftc.teamcode.Test.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Utilites.StoneApproach;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.VuforiaWrapper;
import org.firstinspires.ftc.teamcode.Swerve.SwerveController;

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
        // Initialize the conroller for the swerve modules
        swerveController = new SwerveController(hardwareMap);

        // Initialize class facilitating the approach of a stone
        StoneApproach approach = new StoneApproach(vuforia, swerveController, hardwareMap);
    }

    @Override
    public void init_loop() {
        vuforia.initVuforia();
    }

    @Override
    public void loop() {
        if(!hasRun) {
            approach.approachStone(zOffset, 10);

            // Stop the op mode
            hasRun = true;
        }
    }
}
