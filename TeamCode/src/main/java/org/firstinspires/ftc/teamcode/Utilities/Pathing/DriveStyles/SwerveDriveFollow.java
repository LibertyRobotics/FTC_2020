package org.firstinspires.ftc.teamcode.Utilities.Pathing.DriveStyles;

import org.firstinspires.ftc.teamcode.Utilities.Pathing.Utilities.Waypoint;

import java.util.List;

/**
 * Class created to integrate the path following into using swerve
 * @author Will Richards
 */
public class SwerveDriveFollow {

    private static int currentWaypoint = 0;

    /*
     * This method simply will run the if command and based on weather or not it returns true or false it will increment the current waypoint
     */
    public static void follow(SwerveController controller, List<Waypoint> waypointList){
        if(controller.autoControlModules(waypointList.get(currentWaypoint).getAngle(), waypointList.get(currentWaypoint).getDistance(), 0.8)){
            currentWaypoint++;
        }
    }

}
