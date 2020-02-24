package org.firstinspires.ftc.teamcode.Test.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Quasi Static Voltage Test", group = "test")
public class QuasiStaticVoltageTest extends LinearOpMode {

    private DcMotorEx RightTopSwerveMotor, RightBottomSwerveMotor, LeftTopSwerveMotor, LeftBottomSwerveMotor;

    private double currentVelocityRT, currentVelocityRB, currentVelocityLT, currentVelocityLB;

    private double currentPower = 0.0;

    @Override

    public void runOpMode() {

        RightTopSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "RightTopSwerveMotor");
        RightBottomSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "RightBottomSwerveMotor");
        LeftTopSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "LeftTopSwerveMotor");
        LeftBottomSwerveMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "LeftBottomSwerveMotor");

        waitForStart();

        while (opModeIsActive()) {

            RightTopSwerveMotor.setPower(currentPower);
            RightBottomSwerveMotor.setPower(-currentPower);
            LeftTopSwerveMotor.setPower(-currentPower);
            LeftBottomSwerveMotor.setPower(currentPower);

            currentVelocityRT = RightTopSwerveMotor.getVelocity();
            currentVelocityRB = RightBottomSwerveMotor.getVelocity();
            currentVelocityLT = LeftTopSwerveMotor.getVelocity();
            currentVelocityLB = LeftBottomSwerveMotor.getVelocity();

            currentPower = currentPower + .0001;





            telemetry.addData("current velocityRT", currentVelocityRT);
            telemetry.addData("current velocityRB", currentVelocityRB);
            telemetry.addData("current velocityLT", currentVelocityLT);
            telemetry.addData("current velocityLB", currentVelocityLB);

            telemetry.addData("current power", currentPower);

            telemetry.update();

        }

    }


}
