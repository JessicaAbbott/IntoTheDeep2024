package org.firstinspires.ftc.teamcode.IntoTheDeep.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
@Config
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

         //getting to basket
         robot.strafe(22, 0.6, 0.1);
         Extend();
         robot.turnTo(15, 0.5, 0.1);
         robot.move(2,2,0.6,0.1);


         //bringing arm up to basket height
         setArmTarget(156);
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

         // position to grab middle sample THIS IS OFF
         robot.turnTo(79,0.5,0.1);// THIS VALYUE WOULD NEED TUNING
         stopExtension();
         robot.strafe(-8, 0.6,0.1);

         // set arm target to rest pos
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

         robot.move(4,4,0.5,0.1);
         robot.strafe(-5,0.5,0.1);// THIS VALUE WOULD ALSO MAYBE NEED TUNING


         // turn on intake
         intakeServo.setPosition(0.3d);
         intakeLeft.setPower(0.7);
         intakeRight.setPower(0.7);

         // bring arm down to middle floor sample
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

         //bring to high basket
         robot.strafe(8,0.5,0.1);
         robot.turnTo(-90,0.5,0.1);

         // arm in position for high basket
         setArmTarget(157);
         intakeServo.setPosition(1.0);
         runArmPID();
         waitForArmToReachTarget();

         // out take sample 2
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

         // position for farthest sample THIS IS OFF
         robot.strafe(-7,0.5,0.1);
         robot.turnTo(91,0.5,0.1);// THIS VALUE
         robot.strafe(-8.5,0.55,0.1);// AND THIS VALUE ARE THE ONES THAT WOULDNEED TUNING

         // turn on intake
         intakeServo.setPosition(0.3d);
         intakeLeft.setPower(0.7);
         intakeRight.setPower(0.7);

         // arm down to pick up far sample
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

         // go to  high basket
         robot.move(6,8,0.6,0.1);
         robot.turnTo(-85,0.5,0.1);
         robot.strafe(2,0.5,0.5);
         robot.turnTo(10,0.5,0.5);

         // bring far sample to high basket
         setArmTarget(157);
         intakeServo.setPosition(1.0);
         runArmPID();
         waitForArmToReachTarget();

         //out take last sample
         sleep(50);
         intakeLeft.setPower(1);
         intakeRight.setPower(1);
         sleep (300);
         intakeLeft.setPower(0);
         intakeRight.setPower(0);

         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         armExtension2.setPower(-1);
         armExtension1.setPower(-1);
         sleep (700);
         armExtension2.setPower(0);
         armExtension1.setPower(0);

         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         // go park
         robot.turnTo(90,0.5,0.1);
         robot.strafe(-54,0.6,0.1);
         robot.turnTo(90,0.5,0.1);
         robot.strafe(40,0.6,0.1);




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





