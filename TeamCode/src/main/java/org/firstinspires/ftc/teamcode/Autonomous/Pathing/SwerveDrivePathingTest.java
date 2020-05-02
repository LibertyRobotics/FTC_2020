package org.firstinspires.ftc.teamcode.Autonomous.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.DriveStyles.SwerveDriveFollow;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.Path;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.Utilities.DriveStyles;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.Utilities.Waypoint;

import java.util.List;

/**
 * Test class used to test swerve autonomous through path following
 * @author Will Richards
 */
@Autonomous(name = "Swerve Drive Test Path", group = "Test")
@Disabled
public class SwerveDrivePathingTest extends OpMode {

    SwerveController controller;

    Path path;

    List<Waypoint> waypointList;

    @Override
    public void init() {
        controller = new SwerveController(hardwareMap, telemetry);

        path = new Path(DriveStyles.SWERVE,controller,hardwareMap, IMUOrientation.HORIZONTAL);

        waypointList = path.load("SwerveTestPath");
    }

    @Override
    public void loop() {
        SwerveDriveFollow.follow(controller, waypointList);
    }
}
