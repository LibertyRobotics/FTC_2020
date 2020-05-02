
package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;

@Autonomous(name = "Line Drive Auto", group = "Autonomous")
public class LineDriveAuto extends OpMode {

    private boolean hasRun = false;
    private SwerveController swerveController;
    private DcMotorSimple elevator;

    @Override
    public void init() {
        swerveController = new SwerveController(gamepad1, hardwareMap, telemetry, IMUOrientation.HORIZONTAL, false);
        elevator = hardwareMap.get(DcMotorSimple.class, "ElevatorMotor");

    }

    @Override
    public void loop() {
        if(!hasRun){
            //Parks On Line
            hasRun = swerveController.autoControlModules(0, 26, 0.5);

        }
    }
}