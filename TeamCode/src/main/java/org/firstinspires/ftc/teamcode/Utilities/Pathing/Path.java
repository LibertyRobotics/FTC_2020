package org.firstinspires.ftc.teamcode.Utilities.Pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.Hardware.Enums.IMUOrientation;
import org.firstinspires.ftc.teamcode.Utilities.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.DriveStyles.SwerveDriveFollow;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.DriveStyles.TankDriveFollow;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.Utilities.DriveStyles;
import org.firstinspires.ftc.teamcode.Utilities.Pathing.Utilities.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Class that is used to do anything with the path
 * @author Will Richards
 */
public class Path {

    DriveStyles driveStyles;

    //Arrays that can hold a list of motors for each side
    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    double encoderResoulution;

    SwerveController controller;

    double wheelCircumfrance;

    IMU imu;

    /**
     * USE WITH SWERVE
     * Sets up the parameters used for the
     * @param driveStyles tells the pather what kind of drive it will be running
     */
    public Path(DriveStyles driveStyles, SwerveController controller, HardwareMap hardwareMap, IMUOrientation orientation){
        this.driveStyles = driveStyles;
        this.controller = controller;

        imu = new IMU(hardwareMap, orientation);
    }

    /**
     * Overloaded Path for normal motor drive styles
     * @param driveStyles the wanted drive style
     * @param leftMotors the motors on the left
     * @param rightMotors the motors on the right
     */
    public Path(DriveStyles driveStyles, DcMotor[] leftMotors, DcMotor[] rightMotors, double wheelDiameter, double encoderResolution, HardwareMap hardwareMap, IMUOrientation orientation){
        this.driveStyles = driveStyles;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;

        //Number of ticks per one revolution (of the output shaft)
        this.encoderResoulution = encoderResolution;

        this.wheelCircumfrance = wheelDiameter * Math.PI;

        imu = new IMU(hardwareMap, orientation);

    }

    /**
     * This method is used to load the file into a list of waypoints
     */
    public List<Waypoint> load(String className){

        //List to hold all waypoints in the path
        List<Waypoint> waypointsList = new ArrayList<>();

        //Creates a new class refernce and a new object reference
        Class<?> cls = null;
        Object classReference = null;

        //Will attempt to load the class by name (I didnt even know this was possible till now)
        try { cls = Class.forName("org.firstinspires.ftc.teamcode.Utilities.Pathing.Paths." + className); } catch (ClassNotFoundException e) { }

        //Create a new instance of the class and set it equal to the object
        try { classReference = cls.newInstance(); } catch (Exception e) { }

        //Will store the output of the overloaded toString method
        String initalOutput = classReference.toString();

        //Splits at the question marks to split up sections
        String[] splitString = initalOutput.split("\\?");

        //Creates strings to store the waypoint position lists
        String X = "";
        String Y = "";
        String Angles = "";
        String Distances = "";
        String Actions = "";

        int i = 0;

        /*
         * Loops through the slightly parsed list and trims each section to be just numbers and commas
         */
        for (String list : splitString){
            String tempStr = list.substring(list.indexOf("["), list.indexOf("]"));
            tempStr = tempStr.replace("[","");
            tempStr = tempStr.replace(" ", "");

            //Depending on which set we are in store it as a different value
            switch (i){
                case 0:
                    X = tempStr;
                    break;
                case 1:
                    Y = tempStr;
                    break;
                case 2:
                    Angles = tempStr;
                    break;
                case 3:
                    Distances = tempStr;
                    break;
                case 4:
                    Actions = tempStr;
                    break;
            }
            i++;
        }

        //Make 5 different arrays to hold all the information that is output by the program, split each at the commas to get individual values
        String[] xArray = X.split(",");
        String[] yArray = Y.split(",");
        String[] AngleArray = Angles.split(",");
        String[] DistanceArray = Distances.split(",");
        String[] ActionArray = Actions.split(",");

        //Take all that information and create a new waypoint in a list and add that to the list
        for(int c=0;c<xArray.length;c++){
            waypointsList.add(new Waypoint(xArray[c],yArray[c],AngleArray[c], DistanceArray[c], ActionArray[c], false, false));
        }

        return waypointsList;
    }

    /**
     * Simple high level method that can be called to tell the robot to start following the path
     * Using if statements because the breaks in switch statements break the control loop
     */
    public void followPath(List<Waypoint> waypointList){
        if(driveStyles == DriveStyles.TANK)
            TankDriveFollow.follow(leftMotors,rightMotors,waypointList,encoderResoulution,wheelCircumfrance, imu);
        else if(driveStyles == DriveStyles.SWERVE)
            SwerveDriveFollow.follow(controller, waypointList);
    }

}
