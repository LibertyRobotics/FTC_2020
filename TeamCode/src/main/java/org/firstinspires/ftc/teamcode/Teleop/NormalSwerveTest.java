package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;

@TeleOp(name = "SwerveTeleopTest", group = "Test")
public class NormalSwerveTest extends OpMode {

    private SwerveModule module;

    @Override
    public void init() {
        module = new SwerveModule("TurnMotor", "DriveMotor", hardwareMap);
    }

    @Override
    public void loop() {
        module.control(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
