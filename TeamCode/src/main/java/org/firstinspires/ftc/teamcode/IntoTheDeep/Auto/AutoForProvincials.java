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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.SimplifiedOdometryRobot;

@Autonomous
@Config
public class AutoForProvincials extends LinearOpMode {

   private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;
   public DcMotor armExtension1;
   public DcMotor armExtension2;

   private PIDController armController;
   public static double p = 0.1, i = 0.0, d = 0.002;  // Adjust PID constants
   public static double f = 0.1; // Feedforward value for holding against gravity
   private final double ticks_in_degree = 8192 / 360;

   double close = 0.3;
   double open = 0.7;
   double target = 0;
   private DcMotorEx armPivot;

   double highBasket = 145;
   double rest = 45;

   @Override
   public void runOpMode() {

      telemetry.addData("x", robot.pose.x);
      telemetry.addData("y", robot.pose.y);
      telemetry.addData("h", robot.pose.h);
      telemetry.update();


      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      // Initialize the robot hardware & Turn on telemetry
      robot.initialize(true);

      intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
      intakeLeft.setDirection(CRServo.Direction.REVERSE);

      intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
      intakeRight.setDirection(CRServo.Direction.FORWARD);

      intakeServo = hardwareMap.get(Servo.class, "intakeServo");

      armExtension1 = hardwareMap.get(DcMotor.class, "lateral");
      armExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
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

      robot.setPos(36, 9, 0);

      waitForStart();
      //robot.resetHeading();
      robot.setPos(36, 9, 0);
      armController.reset();

      if (opModeIsActive()) {

         // sample one, preloaded

         intakeServo.setPosition(close);
         armExtension1.setPower(1);
         armExtension2.setPower(1);
         //bring arm to initial resting position and extend arm
         setArmTarget(rest);
         runArmPID();
         waitForArmToReachTarget();


         // to high basket to place preloaded sample and stop extension
         robot.moveToPose(16, 11, 20, 1.0, 1.0, 0.1);
         armExtension2.setPower(0);
         armExtension1.setPower(0);


         // bring it up to high basket
         setArmTarget(highBasket);
         runArmPID();
         waitForArmToReachTarget();


         // put sample 1 in high basket
         outtake();

         // arm back to resting position and then stop intake
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         stopIntake();


         // sample two ( far from wall)


         // position to pick up farthest sample from wall
         robot.moveToPose(17, 19, 92, 1.0, 1.0, 0.1);


         // intake on, arm down, arm back up, stop intake
         intake();
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();


         //position for high basket
         robot.moveToPose(14,18,45,1.0,1.0,0.1);


         // arm up to high basket position
         setArmTarget(highBasket);
         runArmPID();
         waitForArmToReachTarget();

         // outtake sample 2
         outtake();
         /*

         // arm back to resting position and intake off
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         stopIntake();


         // sample three ( middle from wall)


         // position to pick up middle sample from wall
         robot.moveToPose(12,18,90,1.0,1.0,0.1);

         // intake on, arm down, arm back up, stop intake
         intake();
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();

         //position for high basket
         robot.moveToPose(16,15,225,1.0,1.0,0.1);


         // arm up to high basket position
         setArmTarget(150);
         runArmPID();
         waitForArmToReachTarget();

          //outtake sample 3
         outtake();

         // arm back to resting position and intake off
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         stopIntake();


         // sample four, (close to wall)

         // position to pick up sample close to wall ( may have to do some maneuvering to actually get this one)
         robot.moveToPose(12,18,135,1.0,1.0,0.1);


         // intake on, arm down, arm back up, stop intake
         intake();
         setArmTarget(rest);
         runArmPID();
         waitForArmToReachTarget();

         // position for high basket
         robot.moveToPose(16,15,225,1.0,1.0,0.1);


         // arm up to high basket position
         setArmTarget(150);
         runArmPID();
         waitForArmToReachTarget();

         // outtake sample 4
         outtake();

         // arm back  to resting position, and intake off
         setArmTarget(45);
         runArmPID();
         waitForArmToReachTarget();
         stopIntake();


         // park and load sample 5

         //park
         robot.moveToPose(24,60,90,1.0,1.0,0.1);
         robot.moveToPose(48,60,0,1.0,1.0,0.1);

         //  attempt to pick up another sample
         intake();

          */


      }
   }

   private void stopIntake() {
      intakeLeft.setPower(0);
      intakeRight.setPower(0);
   }

   private void outtake() {
      intakeServo.setPosition(open);
      intakeLeft.setPower(0.8);
      intakeRight.setPower(0.8);
      sleep(300);
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
      while (Math.abs(armPos - target) > 1) {
         runArmPID();
         armPos = -armPivot.getCurrentPosition() / ticks_in_degree;
         telemetry.update();
      }
      runArmPID();
      telemetry.addData("Arm reached target", target);
   }

   public void intake() {
      intakeServo.setPosition(close);
      intakeLeft.setPower(1);
      intakeRight.setPower(1);
      setArmTarget(23);
      runArmPID();
      waitForArmToReachTarget();
   }
}





