package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import java.util.ArrayList;

/**
 * Individual Module Class
 */
public class SwerveModule {

    // Control values
    private double FWD = 0;
    private double STR = 0;
    private double RCW = 0;

    private int angle = 0;

    private DcMotor angleMotor;
    private DcMotor driveMotor;

    public SwerveModule(String angleMotorName, String driveMotorName, HardwareMap hardwareMap){
        angleMotor = hardwareMap.get(DcMotor.class, angleMotorName);
        driveMotor = hardwareMap.get(DcMotor.class, driveMotorName);

        angleMotor.setTargetPosition(0);

        // Set the angle motor to be controlled only by the ticks and then set the drive motor to run at a constant speed based off the encoders
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Overall Control method
     */
    public void control(double FWD, double STR, double RCW){
        double[] wheelSpeeds = SwerveMath.calculateWheelSpeeds(FWD, STR, RCW);
        double[] moduleAngle = SwerveMath.calculateWheelAngles(FWD, STR, RCW);

        if(moduleAngle[0] >= -90 && moduleAngle[0] <= 90)
            driveMotor.setPower(wheelSpeeds[0]);
        else
            driveMotor.setPower(-wheelSpeeds[0]);

        angle = (int)SwerveMath.convertAnglesToTicks(moduleAngle[0], 1);
        angleMotor.setTargetPosition(angle);


    }

}
