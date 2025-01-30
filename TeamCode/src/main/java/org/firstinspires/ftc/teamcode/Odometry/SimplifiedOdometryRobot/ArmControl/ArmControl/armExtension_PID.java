package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.ArmControl.ArmControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@TeleOp


public class armExtension_PID extends OpMode{

   private PIDController extensionController;
   public static double extensionKp = 0.32, extensionKi = 0, extensionKd = 0.0;  // Tuning constants for extension PID controller
   public static double extensionKf = 0.0;
   private final double ticks_in_inch=(8192 * 0.5249330709);//change this constant

   double ExtensionTarget =15;
   private DcMotorEx armExtension1;
   private DcMotorEx armExtension2;

   @Override
   public void init(){

      extensionController=new PIDController(extensionKp,extensionKi, extensionKd);
      telemetry=new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

      armExtension1=hardwareMap.get(DcMotorEx.class,"armExtension");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      armExtension2=hardwareMap.get(DcMotorEx.class,"lateral");
      armExtension2.setDirection(DcMotorSimple.Direction.REVERSE);
      armExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

   @Override
   public void loop(){
      extensionController.setPID(extensionKp,extensionKi, extensionKd);
      double extensionPos= armExtension2.getCurrentPosition()/ticks_in_inch;
      double pid= extensionController.calculate(extensionPos,ExtensionTarget);
      double ff= ExtensionTarget*extensionKf;
      double power=pid + ff;

      armExtension1.setPower(power);
      armExtension2.setPower(power);

      telemetry.addData("ExtensionPos", extensionPos);
      telemetry.addData("ExtensionTarget",ExtensionTarget);
      telemetry.addData("encoder", armExtension2.getCurrentPosition());
      telemetry.update();
   }
}