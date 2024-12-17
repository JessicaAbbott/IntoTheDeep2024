package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicTeleSample extends OpMode {

   DcMotor RFMotor;
   DcMotor LFMotor;
   DcMotor RBMotor;
   DcMotor LBMotor;

   CRServo IntakeLeft;
   CRServo IntakeRight;

   DcMotor armPivot;
   DcMotor armExtend;


   public void moveDriveTrain() {

      double drive = 0;
      double strafe = 0;
      double TurnTo = 0;

      drive = -gamepad1.right_stick_y;
      strafe = -gamepad1.right_stick_x;
      TurnTo = gamepad1.left_stick_x;

      RFMotor.setPower(TurnTo + (drive + strafe));
      RBMotor.setPower(TurnTo + (drive - strafe));
      LFMotor.setPower(TurnTo + (drive - strafe));
      LBMotor.setPower(TurnTo + (drive + strafe));

      // turn function
      if(gamepad1.left_stick_x > 0){

         RFMotor.setPower(-0.9);
         RBMotor.setPower(-0.9);
         LFMotor.setPower(0.9);
         LBMotor.setPower(0.9);
      }
      else if (gamepad1.left_stick_x < 0){

         RFMotor.setPower(0.9);
         RBMotor.setPower(0.9);
         LFMotor.setPower(-0.9);
         LBMotor.setPower(-0.9);
      }
      else{

         RFMotor.setPower(0);
         RBMotor.setPower(0);
         LFMotor.setPower(0);
         LBMotor.setPower(0);
      }



      // arm pivot
      if (gamepad2.right_stick_y>0){armPivot.setPower(0.9);}

     else if (gamepad2.right_stick_y<0){armPivot.setPower(-0.9);}

     else {armPivot.setPower(0);}



     //arm extend
      if (gamepad2.dpad_up){armExtend.setPower(0.9);}

      else if (gamepad2.dpad_down){armExtend.setPower(-0.9);}

      else{armExtend.setPower(0);}



      if(gamepad2.right_bumper){
        IntakeLeft.setPower(0.9); //intake
         IntakeRight.setPower(-0.9);
      }

      else if (gamepad2.left_bumper){
         IntakeLeft.setPower(-0.9); //out
         IntakeRight.setPower(0.9);
      }

      else{
         IntakeLeft.setPower(0); //off
         IntakeRight.setPower(0);
      }
   }

   @Override
   public void init() {
      RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
      LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
      LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
      RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
      IntakeLeft=hardwareMap.get(CRServo.class,"IntakeLeft");
      IntakeRight=hardwareMap.get(CRServo.class,"IntakeRight");

      RFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      LFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      LBMotor.setDirection(DcMotorSimple.Direction.FORWARD);

      IntakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

   }

   @Override
   public void loop() {
      moveDriveTrain();
   }

}
