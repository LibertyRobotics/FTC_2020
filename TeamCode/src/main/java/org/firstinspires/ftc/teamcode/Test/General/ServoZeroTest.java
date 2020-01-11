package org.firstinspires.ftc.teamcode.Test.General;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "svBottom Test", group = "Test")
public class ServoZeroTest extends OpMode {
    private Servo svBottom;
    private boolean hasRun = false;

    @Override
    public void init() {
        svBottom = hardwareMap.servo.get("svBottom");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            svBottom.setPosition(0.6);
        } else if (gamepad1.left_bumper) {
            svBottom.setPosition(0.3);
        } else if (gamepad1.back) {
            svBottom.setPosition(0.5);
        } else if (gamepad1.right_trigger > 0) {
            svBottom.setPosition(0.64);
        } else if (gamepad1.left_trigger > 0) {
            svBottom.setPosition(0.47);
        }

        /* Autonomous zeroing code
        if (!hasRun) {
            svBottom.setPosition(0.5);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            hasRun = true;
        } else {
            requestOpModeStop();
        }
         */
    }
}
