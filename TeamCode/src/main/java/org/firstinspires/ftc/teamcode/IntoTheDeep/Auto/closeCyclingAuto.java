package org.firstinspires.ftc.teamcode.IntoTheDeep.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
@Config
@Disabled

public class closeCyclingAuto extends LinearOpMode {

   private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;
   public DcMotor armExtension1;
   public DcMotor armExtension2;

   private PIDController armController;
   public static double p = 0.05, i = 0.02, d = 0.002;  // Adjust PID constants
   public static double f = 0.1; // Feedforward value for holding against gravity
   private final double ticks_in_degree = 8192 / 360;

   double target = 0;
   private DcMotorEx armPivot;

   @Override

   public void runOpMode() {

      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      // Initialize the robot hardware & Turn on telemetry
      robot.initialize(true);

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.FORWARD);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.REVERSE);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");

      armExtension1 = hardwareMap.get(DcMotor.class, "lateral");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      armExtension2 = hardwareMap.get(DcMotor.class, "armExtension");
      armExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      armController = new PIDController(p, i, d);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
      armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      telemetry.addData(">", "Touch Play to run Auto");
      telemetry.update();

      waitForStart();
      robot.resetHeading();
      armController.reset();

      if (opModeIsActive()) {

         // bring arm to resting position

         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

        //getting to high basket with pre loaded sample
         robot.strafe(24,0.6,0.15);

         //bringing arm up to basket height
         setArmTarget(150);
         intakeServo.setPosition(1.0);
         runArmPID();
         waitForArmToReachTarget();

         //  out take sample 1
         sleep(50);
         intakeLeft.setPower(1);
         intakeRight.setPower(1);
         sleep (300);
         intakeLeft.setPower(0);
         intakeRight.setPower(0);


         // set arm target to rest pos
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();


         // position for far sample
         robot.drive(20,0.6,0.1);
         robot.strafe(24,0.6,0.1);
         robot.turnTo(90,0.5,0.1);

         // turn on intake
         intakeServo.setPosition(0.3d);
         intakeLeft.setPower(0.7);
         intakeRight.setPower(0.7);

         // bring arm down to far floor sample
         setArmTarget(23);
         runArmPID();
         waitForArmToReachTarget();

         // turn off intake, after grabbed
         intakeLeft.setPower(0);
         intakeRight.setPower(0);

         // arm back up to resting position
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

         //bring  fars sample to high basket
        robot.turnTo(0,0.5,0.1);
        robot.drive(9,0.6,0.1);
        robot.strafe(24,0.6,0.1);


         // arm in position for high basket for far sample
         setArmTarget(150);
         intakeServo.setPosition(1.0);
         runArmPID();
         waitForArmToReachTarget();

         // out take sample far
         sleep(50);
         intakeLeft.setPower(1);
         intakeRight.setPower(1);
         sleep (300);
         intakeLeft.setPower(0);
         intakeRight.setPower(0);

         // bring arm back to rest position
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

         // position for middle sample
        robot.drive(20,0.6,0.1);
         robot.strafe(12,0.6,0.1);
         robot.turnTo(90,0.5,0.1);

         // turn on intake
         intakeServo.setPosition(0.3d);
         intakeLeft.setPower(0.7);
         intakeRight.setPower(0.7);

         // arm down to pick up middle sample
         setArmTarget(23);
         runArmPID();
         waitForArmToReachTarget();

         // turn off intake, after grabbed
         intakeLeft.setPower(0);
         intakeRight.setPower(0);

         // arm to rest pos
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

         // go to  high basket to deposit middle sample
         robot.turnTo(0,0.5,0.1);
         robot.drive(14,0.6,0.1);
         robot.strafe(12,0.6,0.1);

         // put middle sample in high basket
         setArmTarget(150);
         intakeServo.setPosition(1.0);
         runArmPID();
         waitForArmToReachTarget();

         //out take middle sample
         sleep(50);
         intakeLeft.setPower(1);
         intakeRight.setPower(1);
         sleep (300);
         intakeLeft.setPower(0);
         intakeRight.setPower(0);

         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

        //position for wall sample

         //intake on


         // bring arm, down on sample


         // resting pos


         // intake off


        //bring wall sample to high basket


         // arm up to high basket


         // out take wall sample


         // go park





      }
   }

   public void runArmPID() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      double pidOutput = armController.calculate(armPos, target);
      double ff = Math.cos(Math.toRadians(armPos)) * f;
      double power = pidOutput + ff;

      power = Math.max(-0.4, Math.min(1.0, power));

      armPivot.setPower(power);

      telemetry.addData("ArmPos", armPos);
      telemetry.addData("ArmTarget", target);
      telemetry.addData("PID Output", pidOutput);
      telemetry.addData("Feedforward", ff);
      telemetry.addData("Motor Power", power);
   }

   public void setArmTarget(double newTarget) {
      target = newTarget;
   }

   public void waitForArmToReachTarget() {
      double armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
      while (Math.abs(armPos - target) > 6) {
         runArmPID();
         armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
         telemetry.update();
      }
      runArmPID();
      telemetry.addData("Arm reached target", target);
   }

   public void intake() {
      intakeServo.setPosition(0.7);
      intakeLeft.setPower(1);
      intakeRight.setPower(1);
      setArmTarget(23);


   }

   public void outtake() {
      intakeServo.setPosition(1.0);
      intakeLeft.setPower(0.8);
      intakeRight.setPower(0.8);
   }

   public void stopIntake(){
      intakeLeft.setPower(0);
      intakeRight.setPower(0);
   }


   public void Extend(){
      armExtension1.setPower(1);
      armExtension2.setPower(1);
   }
   public void stopExtension(){
      armExtension1.setPower(0);
      armExtension2.setPower(0);
   }

}





