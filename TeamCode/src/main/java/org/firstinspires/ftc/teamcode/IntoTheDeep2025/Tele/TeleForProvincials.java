package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@TeleOp
@Config
public class TeleForProvincials extends LinearOpMode {

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;

   public Servo climbServo;
   public DcMotor climbHook;
   private PIDController armController;

   boolean climbButtonPressed = false;

   boolean climbServoPressed = false;

   // PID constants
   public static double p = 0.052, i = 0.02, d = 0.001;  // Tuning constants for PID controller
   public static double f = 0.1;  // Feedforward constant

   private final double ticks_in_degree = 8192 / 360.0;  // Encoder ticks per degree

   private DcMotorEx armPivot;
   private double target = 40;  // Initial target position for the arm

   double newTarget = 0;
   DcMotor armExtension1;
   public DcMotor armExtension2;

   final double SAFE_DRIVE_SPEED = 1.0;
   final double SAFE_STRAFE_SPEED = 1.0;
   final double SAFE_YAW_SPEED = 1.0;
   final double HEADING_HOLD_TIME = 10.0;

   ElapsedTime stopTime = new ElapsedTime();
   boolean autoHeading = false;

   long lastTime = System.nanoTime();

   private double lastLF = 0.0;
   private double lastLB = 0.0;
   private double lastRF = 0.0;
   private double lastRB = 0.0;


   SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   @Override
   public void runOpMode() throws InterruptedException {

      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armExtension2 = hardwareMap.get(DcMotor.class, "lateral");
      armExtension2.setDirection(DcMotorSimple.Direction.REVERSE);
      armExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


      armExtension1 = hardwareMap.get(DcMotor.class, "armExtension");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.FORWARD);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.REVERSE);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");

      climbServo = hardwareMap.get(Servo.class, "climbServo");


      armController = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      climbHook = hardwareMap.get(DcMotor.class, "axial");
      climbHook.setDirection(DcMotorSimple.Direction.FORWARD);
      climbHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      robot.initialize(true);

      while (opModeInInit()) {
         robot.readSensors();
         telemetry.update();
      }

      while (opModeIsActive()) {
         robot.readSensors();
         long currentTime = System.nanoTime();

         // Allow the driver to reset the gyro by pressing both small gamepad buttons
         /*
         if (gamepad1.options && gamepad1.share) {
            robot.resetHeading();
            //robot.resetOdometry();
         }
          */

         if (gamepad1.a) {
            //robot.setPos(36,9,0);
            robot.resetHeading();
         }

         // read joystick values and scale according to limits set at top of this file
         double drive = gamepad1.right_stick_x * SAFE_DRIVE_SPEED;
         double strafe = -gamepad1.right_stick_y * SAFE_STRAFE_SPEED;
         double yaw =  gamepad1.left_stick_x * SAFE_YAW_SPEED;

         // Drive the robot
         robot.driveWithFieldCentric(drive, strafe, yaw);

         // If auto heading is on, override manual yaw with the value generated by the heading controller.
         if (autoHeading) {
            yaw = robot.yawController.getOutput(robot.getHeading());
         }

         // arm
         // Update the arm pivot position using PID control
         if (gamepad2.b) {
            target = 84; // low basket pos
            newTarget = 84;
         } else if (gamepad2.y) {
            target = 129;
            newTarget = 129;

         } else if (gamepad2.dpad_down) {
            target = -26;  // all the way down and in tucked pos for climb
            newTarget = -26;
         } else if (gamepad2.dpad_up) {
            target = 74;  // high bar for specimens
            newTarget = 74;
         } else if (gamepad2.a) {
            target = 14;
            newTarget = 14;
         } else if (gamepad2.dpad_left) {
            newTarget = target - 5;
         } else if (gamepad2.dpad_right) {
            newTarget = target + 5;
         }
         else {
            target = newTarget;
         }

         // intake

         if (gamepad2.left_bumper) { // out
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
         } else if (gamepad2.right_bumper) {
            //continue in and move servo out of way
            intakeServo.setPosition(1.0);
            intakeLeft.setPower(0.6);
            intakeRight.setPower(0.6);
         } else if (gamepad2.right_trigger > 0) { // in
            // down to pick up
            intakeServo.setPosition(0.3);
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            target = 0;
         }

         else {
            intakeServo.setPosition(0.3d);
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
         }


         // PID control for the arm pivot
         double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;  // Convert encoder ticks to degrees
         double pidOutput = armController.calculate(armPos, target);  // Calculate PID output
         double ff = Math.cos(Math.toRadians(target)) * f;  // Feedforward term
         double power = pidOutput + ff;  // Final motor power (PID + Feedforward)

         power = Math.max(-0.4, Math.min(1.75, power));


         armPivot.setPower(power);  // Set motor power to the calculated value

         // Arm extension control
         if (gamepad2.left_stick_y < 0) {
            armExtension1.setPower(1);
            armExtension2.setPower(1);
         } else if (gamepad2.left_stick_y > 0) {
            armExtension1.setPower(-1);
            armExtension2.setPower(-1);
         } else {
            armExtension1.setPower(0);
            armExtension2.setPower(0);
         }

         // climb


         if (gamepad1.y){
            climbServo.setPosition(0.5);
          }

         else if(gamepad1.a){
            climbServo.setPosition(1.0);


         }





         else if(climbServoPressed=false) {
            climbServo.setPosition(0.3);
         }


         if (gamepad1.x) {
            climbButtonPressed = true;
         }

         // climb
         if (gamepad1.left_trigger>0){
            //down
            climbHook.setPower(1);
         } else if (gamepad1.right_trigger > 0) {
            //up
            climbHook.setPower(-1);
         }

         else if (climbButtonPressed=true){
            climbHook.setPower(0.2);
         }

         else {
            climbHook.setPower(0);
         }

         double currentLF = robot.LFMotor.getCurrentPosition() * 0.0012186958;
         double currentLB = robot.LBMotor.getCurrentPosition() * 0.0012186958;
         double currentRF = robot.RFMotor.getCurrentPosition() * 0.0012186958;
         double currentRB = robot.RBMotor.getCurrentPosition() * 0.0012186958;
         double loopTime = (currentTime-lastTime) / 1E9;

         telemetry.addData("ArmPos", armPos);
         telemetry.addData("ArmTarget", target);
         telemetry.addData("PID Output", pidOutput);
         telemetry.addData("Motor Power", power);

         telemetry.addData("x",robot.pose.x);
         telemetry.addData("y",robot.pose.y);
         telemetry.addData("h",robot.pose.h);
         //telemetry.addData("heading",robot.getHeading());

         telemetry.addData("ms", (currentTime-lastTime)/1E6);
         telemetry.addData("lf", (currentLF-lastLF)/loopTime);
         telemetry.addData("lb", (currentRF-lastRF)/loopTime);
         telemetry.addData("lf", (currentLB-lastLB)/loopTime);
         telemetry.addData("lf", (currentRB-lastRB)/loopTime);
         telemetry.update();
         lastTime = currentTime;
         lastLF = currentLF;
      }
   }
}
