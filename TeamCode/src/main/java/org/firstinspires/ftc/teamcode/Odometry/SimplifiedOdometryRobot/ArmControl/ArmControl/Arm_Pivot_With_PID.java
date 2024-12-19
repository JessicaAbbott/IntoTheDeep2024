package org.firstinspires.ftc.teamcode.Odometry.SimplifiedOdometryRobot.ArmControl.ArmControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@TeleOp

public class Arm_Pivot_With_PID extends OpMode{

   private PIDController armController;
   public static double p = 0.0695, i = 0, d = 0.0025;  // Tuning constants for PID controller
   public static double f = 0.375;
   private final double ticks_in_degree=8192/360;

    double target =140;
   private DcMotorEx armPivot;


  @Override
   public void init(){

     armController=new PIDController(p,i,d);
     telemetry=new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

     armPivot=hardwareMap.get(DcMotorEx.class,"armPivot");
     armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
     armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


   }

   @Override
   public void loop(){
     armController.setPID(p,i,d);
     double armPos= -armPivot.getCurrentPosition()/ticks_in_degree;
     double pid= armController.calculate(armPos,target);
     double ff= Math.cos(Math.toRadians(target))*f;
     double power=pid +ff;

     armPivot.setPower(power);

     telemetry.addData("ArmPos", armPos);
     telemetry.addData("armTarget",target);
     telemetry.update();


   }
}