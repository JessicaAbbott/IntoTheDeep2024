package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.ArmControl.ArmControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@TeleOp
@Disabled


public class armExtension_PID extends OpMode{

   private PIDController extensionController;
   public static double p=0.0,i=0,d=0.00;
   // p is whether or not it is in the thing properly, second
   //d is oscillations do that third

   public static double f=0; // can it hold against gravity, tune first
   public static int target=45;
   private final double Inches = 0; // FIGURE OUT THIS MATH


   private DcMotorEx armExtension1;
   private DcMotorEx armExtension2;



   @Override
   public void init(){

      extensionController=new PIDController(p,i,d);
      telemetry=new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

      armExtension2=hardwareMap.get(DcMotorEx.class,"lateral");
      armExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      armExtension1=hardwareMap.get(DcMotorEx.class,"armExtension");
      armExtension1.setDirection(DcMotorSimple.Direction.FORWARD);
      armExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armExtension1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


   }

   @Override
   public void loop(){
      extensionController.setPID(p,i,d);
      double extensionPos= -armExtension1.getCurrentPosition()/Inches;
      double pid= extensionController.calculate(extensionPos,target);
      double ff= Math.cos(Math.toRadians(target))*f;
      double power=pid +ff;

      armExtension1.setPower(power);
      armExtension2.setPower(power);


      telemetry.addData("extensionPos", extensionPos);
      telemetry.addData("ExtensionTarget",target);
      telemetry.update();


   }
}