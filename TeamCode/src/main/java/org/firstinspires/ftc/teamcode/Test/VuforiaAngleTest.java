package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.Helper.Tensorflow.DetectedObject;
import org.firstinspires.ftc.teamcode.Helper.Tensorflow.TensorflowWrapper;

import java.util.List;

/**
 * Class used to test angle calculations from the robot to the skystones
 */
@TeleOp (name = "Tensorflow Angle Test", group = "Test")
public class VuforiaAngleTest extends OpMode {

    private TensorflowWrapper tf;

    @Override
    public void init() {

        //Initializes the Tensorflow wrapper class and gives it access to the hardware map
        tf = new TensorflowWrapper(hardwareMap);

        tf.initVuforia();

        //Temp. Just for inital test, it should be fine
        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            tf.initTensorflow();
        }
        else{
            telemetry.addData("Error:", "Device Does Not Support Tensorflow");
        }
    }

    @Override
    public void loop() {

        //Gets and prints out all the tensor flow objects
        getTensorflowObjects();

    }

    /**
     * Creates a list of DetectedObjects then iterates through the list and displays information about each visible object
     */
    private void getTensorflowObjects(){
        //Gets the list of detected objects
        List<DetectedObject> objects = tf.getDetectedObjects();

        if(objects != null){

            telemetry.addData("# Of Objects: ", objects.size());
            int i = 0;
            //For Every Object Output Some Information On it
            for(DetectedObject object : objects) {

                telemetry.addData(i + ": " + "Label: ", object.getLabel());
                telemetry.addData(i + ": " + "Top Left: ", object.getTopLeft());
                telemetry.addData(i + ": " + "Bottom Right: ", object.getBottomRight());
                telemetry.addData(i + ": " + "Center: ", object.getCenter());
                i++;
            }

        telemetry.update();
        }
    }
}
