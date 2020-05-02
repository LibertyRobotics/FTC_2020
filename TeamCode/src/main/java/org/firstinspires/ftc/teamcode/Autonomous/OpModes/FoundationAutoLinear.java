package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.StoneGripController;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;

@Autonomous(name = "Linear Foundation Auto", group = "Autonomous")
public class FoundationAutoLinear extends LinearOpMode {
    private SwerveController swerve;
    private StoneGripController stoneArms;

    //Currently ignores side differences & start rules, array errors need to be patched first

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new SwerveController(null, hardwareMap, telemetry, IMUOrientation.VERTICAL, false);
        stoneArms = new StoneGripController(hardwareMap, null, telemetry);

        waitForStart();

        //Sliiide to the left
        boolean strafeToPosition = false;
        while (!strafeToPosition) {
            strafeToPosition = swerve.autoControlModules(75, 50, 0.4);
        }

        sleep(1000);
        /*
        // Drive to foundation
        boolean drivenToFoundation = false;
        while (!drivenToFoundation) {
            drivenToFoundation = swerve.autoControlModules(90, 6, 0.4);
        }


        sleep(1000);

        stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.DOWN, 1000);
        stoneArms.autoPivot(GripArmPosition.RIGHT, MoveArmDirection.DOWN, 1000);

        sleep(1000);

        boolean pulledFoundationBack = false;
        while (!pulledFoundationBack) {
            pulledFoundationBack = swerve.autoControlModules(180, 24, 0.4);
        }

        sleep(1000);

        stoneArms.autoPivot(GripArmPosition.LEFT, MoveArmDirection.UP, 1000);
        stoneArms.autoPivot(GripArmPosition.RIGHT, MoveArmDirection.UP, 1000);

        sleep(1000);

        //Sliiide to the left pt.2
        boolean strafeToPark = false;
        while (!strafeToPark) {
            strafeToPark = swerve.autoControlModules(270, 60, 0.6);
        }

         */

    }
}
