package org.firstinspires.ftc.teamcode.Test.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;

@Autonomous(name="Distance Drive Test", group="Test")
public class DriveToDistanceTest extends OpMode {
    private SwerveController swerve;
    private boolean hasRun = false;

    @Override
    public void init() {
        // Initialize the swerve module controller
        swerve = new SwerveController(null, hardwareMap, telemetry, IMUOrientation.VERTICAL, false);
    }

    @Override
    public void loop() {
        if (!hasRun) {
            hasRun = swerve.autoControlModules(0, 12, 0.2);
        }
    }
}
