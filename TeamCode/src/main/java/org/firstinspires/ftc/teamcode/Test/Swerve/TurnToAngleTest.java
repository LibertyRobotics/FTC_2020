package org.firstinspires.ftc.teamcode.Test.Swerve;

import android.content.ContentUris;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.tfod.BorderedText;
import org.firstinspires.ftc.teamcode.Swerve.Enums.WheelDirection;
import org.firstinspires.ftc.teamcode.Swerve.SwerveController;
import org.firstinspires.ftc.teamcode.Swerve.SwerveMath;

/**
 * Class created to test driving the swerve module to a wanted angle
 * @author Will Richards
 */
@TeleOp(name = "Swerve Turn Test", group = "Test")
public class TurnToAngleTest extends OpMode {

    private SwerveController swerveController;

    @Override
    public void init() {

        swerveController = new SwerveController(gamepad1, hardwareMap);

    }

    @Override
    public void loop() {
        swerveController.controlModules();

    }
}
