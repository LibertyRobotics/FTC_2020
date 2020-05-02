package org.firstinspires.ftc.teamcode.Autonomous.OpModes.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Utilites.StoneApproach;
import org.firstinspires.ftc.teamcode.Subsystems.ServoArmController;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.SkystonePostion;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.VuforiaWrapper;

@Autonomous(name = "Skystone to Foundation BLUE", group = "Week 2")
public class StoneToFoundationBlue extends OpMode {
    //private boolean[] actionsComplete = {false, false, false, false};

    // Declare all the different subsystems
    private ServoArmController servos;
    private SwerveController swerveController;
    private VuforiaWrapper vuforia;
    private StoneApproach approach;
    private SkystonePostion stonePosition = SkystonePostion.NONE;

    /**
     * Initialization method for this OpMode where all the subsystems are initialized
     * (besides Vuforia).
     */

    @Override
    public void init() {
        // Initialize Vuforia
        vuforia = new VuforiaWrapper(hardwareMap);

        stonePosition = SkystonePostion.NONE;

        // Initialize swerve
        swerveController = new SwerveController(hardwareMap, telemetry);

        // Initialize the StoneApproach class
        approach = new StoneApproach(vuforia, swerveController);

        // Initialize the servo controller for the stone arms
        servos = new ServoArmController(hardwareMap, null);
    }

    /**
     * Run continuously when init button is pressed to initialize Vuforia.
     */
    public void init_loop() {
        vuforia.initVuforia();
    }

    /**
     * Normally, this method is run continuously (as it is a loop), but because of the variable
     * hasRun, the code within the if statement only runs once (or however many times it takes to run to completion).
     */
    @Override
    public void loop() {
        // Drive ~2 feet forward to be able to detect the Skystones
        swerveController.autoControlModules(0, 25, 1);

        switch(stonePosition) {
            default:
            case LEFT:
                swerveController.autoControlModules(360-33.7, 25.3, 0.8);
                swerveController.autoControlModules(270, 28, 0.8);
            case RIGHT:
                swerveController.autoControlModules(33.7, 25.3, 0.8);
                swerveController.autoControlModules(270, 44, 0.8);

            case CENTER:
                swerveController.autoControlModules(0, 24, 0.8);
                swerveController.autoControlModules(270, 36, 0.8);
                break;
            case NONE:
                break;
        }


        /*
        if(!actionsComplete[0]) {
            actionsComplete[0] = swerveController.autoControlModules(0, 25, 1);
        }

        // Drive up to stone using vuforia
        else if(!actionsComplete[1]) {
            actionsComplete[1] = approach.approachStone(10, 10);
        }

        // Pick up stone using right servo arm
        else if(!actionsComplete[2]) {
            // Lower arm
            servos.controlArmsAutonomous(MoveArmDirection.DOWN);

            // Grab stone
            servos.controlArmsAutonomous(MoveArmDirection.GRIP);

            // Raise arm
            servos.controlArmsAutonomous(MoveArmDirection.UP);

            // Get initial stone position for next action
            stonePosition = approach.getInitPosition();

            // Mark this action as complete
            actionsComplete[2] = true;
        }

        // Drive under skybridge (~4 feet away)
        else if(!actionsComplete[3]) {
            switch(stonePosition) {
                default:
                case LEFT:
                    swerveController.autoControlModules(180, 6, .8);
                    swerveController.autoControlModules(270, 40, .8);
                case RIGHT:
                    swerveController.autoControlModules(180, 6, .8);
                    swerveController.autoControlModules(270, 48, .8);
                case CENTER:
                    swerveController.autoControlModules(180, 6, .8);
                    swerveController.autoControlModules(270, 56, .8);
                    break;
                case NONE:
                    break;
            }
            actionsComplete[3] = swerveController.autoControlModules(-90, 48, 1);
        }

         */

    }
}