package org.firstinspires.ftc.teamcode.Autonomous.OpModes.Archive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.StoneGripController;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;

@Autonomous
@Disabled
public class BlueRightStone extends OpMode {
    private SwerveController swerve;
    private StoneGripController stoneArms;

    // This list is used for progression in actions executed.
    private boolean[] actionCompletions = {false, false, false, false, false, false, false};

    @Override
    public void init() {
        // Initialize the swerve module controller
        swerve = new SwerveController(null, hardwareMap, telemetry, IMUOrientation.VERTICAL, false);

        // Initialize the controller for the stone-grabbing arms
        stoneArms = new StoneGripController(hardwareMap, null, telemetry);
    }

    @Override
    public void loop() {
        // Drive up to the stone.
        if (!actionCompletions[0]) {
            actionCompletions[0] = swerve.autoControlModules(0, 32, 0.5);
        }

        // Pick up the stone
        else if (!actionCompletions[1]) {
            // TODO: Implement autonomous control method for moving arms up and down.

            actionCompletions[1] = true;
        }

        // Drive sideways to the foundation
        else if (!actionCompletions[2]) {
            actionCompletions[2] = swerve.autoControlModules(90, 69, 0.5);
        }

        // Lower arms, release stone and grab foundation
        else if (!actionCompletions[3]) {
            // TODO: Same as above: autonomous control for the grip arms.
        }

        // Back up and put foundation in building site
        else if (!actionCompletions[4]) {
            actionCompletions[4] = swerve.autoControlModules(180, 14, 0.5);
        }

        // Raise arms so we don't take the foundation with us
        else if (!actionCompletions[5]) {
            // TODO: Same as 2 above todos
        }

        // Drive under the bridge
        else if (!actionCompletions[6]) {

        }

        // Stop the op mode
        else {
            requestOpModeStop();
        }
    }
}
