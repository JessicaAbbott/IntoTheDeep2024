/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.IntoTheDeep.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous
@Disabled

public class PhillSampleAutoSquare extends LinearOpMode {
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
     private ElapsedTime timer =new ElapsedTime();
    private static final double DRIVE_TIMEOUT_SECONDS= 1.0; // set a time out limit
    private static final double STRAFE_TIMEOUT_SECONDS= 1.0; // set a time out limit

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        //  cRun Auto if stop was not pressed.
        if (opModeIsActive()) {
            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.

            // Drive the path again without turning.
            robot.strafe ( 10, 0.6, 0.15);
           robot.move(10, 10, 0.6, 0.15 );
            //sleep(100);
            //robot.turnTo(90,0.6,0.15);


            /* sleep(5);
            robot.drive( 10, 0.60, 0.15);
            sleep(5);
            robot.strafe( -10, 0.60, 0.15);
            sleep(5);
            robot.drive(-10, 0.60, 0.15);

            sleep(5);

            robot.strafe(  20, 0.60, 0.15);
            sleep(5);
            robot.turnTo(-90,0.6,0.15);
            sleep(5);
            robot.strafe( 20, 0.60, 0.15);
            sleep(5);
            robot.turnTo(-90,0.6,0.15);
            sleep(5);
            robot.strafe( 20, 0.60, 0.15);
            sleep(5);
            robot.turnTo(-90,0.6,0.15);
            sleep(5);
            robot.strafe(20, 0.60, 0.15);
            sleep(5);
            robot.turnTo(-90,0.6,0.15);
            sleep(5);
             */
        }
    }
    /*private boolean Timeout(Runnable command) {
        timer.reset();//reset timer for command
        command.run();//run command

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {

            if ( robot.driveController.inPosition()){
                return true; //command finished successfully
            }
            telemetry.addData("in timeout loop","in timeout loop");
            telemetry.update();
        }
        return false;
    }

     */
}
