package org.firstinspires.ftc.teamcode.Autonomous.OpModes.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Utilites.StoneApproach;
import org.firstinspires.ftc.teamcode.Subsystems.ServoArmController;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.SkystonePostion;
import org.firstinspires.ftc.teamcode.Utilities.Vuforia.VuforiaWrapper;

@Disabled
@Autonomous(name = "Skystone to Foundation RED", group = "Week 2")
public class StoneToFoundationRed extends OpMode {
    // This array allows us to progress through different actions in our program
    // TODO: Write our own style of OpMode that does this automatically
    private boolean[] actionsComplete = {false, false, false, false};

    // Declare all of the robot subsystems
    private ServoArmController servos;
    private SwerveController swerveController;
    private VuforiaWrapper vuforia;
    private StoneApproach approach;
    private SkystonePostion stonePosition = SkystonePostion.NONE;


    /**
     * Initialization method for this OpMode.
     */
    @Override
    public void init() {
        // Initialize Vuforia
        vuforia = new VuforiaWrapper(hardwareMap);

        // Initialize swerve
        swerveController = new SwerveController(hardwareMap, telemetry);

        // Initialize the StoneApproach class for going to a skystone
        approach = new StoneApproach(vuforia, swerveController);

        // Initialize servo arm controller
        servos = new ServoArmController(hardwareMap, null);
    }

    /**
     * Run continuously when init button is pressed to initialize Vuforia.
     */
    @Override
    public void init_loop() {
        vuforia.initVuforia();
    }

    /**
     * Normally, this method is run continuously (as it is a loop), but because of the variable
     * hasRun, the code within the if statement only runs once.
     */
    @Override
    public void loop() {
        // Drive forward ~2 feet to be able to detect the Skystones
        if(!actionsComplete[0]) {
            actionsComplete[0] = swerveController.autoControlModules(0, 25, 1);
        }

        // Approach the stone using Vuforia via the StoneApproach class
        else if(!actionsComplete[1]) {
            //actionsComplete[1] = approach.approachStone(10, 10);
        }

        // Pick up the Skystone after approaching it
        else if(!actionsComplete[2]) {
            // Lower arm
            //servos.controlArmsAutonomous(MoveArmDirection.DOWN);

            // Grab the stone
            //servos.controlArmsAutonomous(MoveArmDirection.GRIP);

            // Raise the arm
            //servos.controlArmsAutonomous(MoveArmDirection.UP);

            // Mark this as done
            actionsComplete[2] = true;
        }
        else if(!actionsComplete[3]) {
            switch(stonePosition) {
                default:
                case LEFT:
                    swerveController.autoControlModules(270, 6, .8);
                    swerveController.autoControlModules(90, 40, .8);
                case RIGHT:
                    swerveController.autoControlModules(270, 6, .8);
                    swerveController.autoControlModules(90, 48, .8);
                case CENTER:
                    swerveController.autoControlModules(270, 6, .8);
                    swerveController.autoControlModules(90, 56, .8);
                    break;
                case NONE:
                    break;
            }
            actionsComplete[3] = swerveController.autoControlModules(-90, 48, 1);
        }
    }
}
