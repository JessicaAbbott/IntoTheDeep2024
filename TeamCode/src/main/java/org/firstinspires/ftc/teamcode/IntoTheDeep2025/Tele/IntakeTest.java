package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends OpMode {
   public CRServo intakeLeft;
   public CRServo intakeRight;
   public Servo intakeServo;

@Override
   public void init(){
   intakeLeft=hardwareMap.get(CRServo.class,"intakeLeft");
   intakeRight=hardwareMap.get(CRServo.class,"intakeRight");
   intakeServo=hardwareMap.get(Servo.class,"intakeServo");

   intakeLeft.setDirection(CRServo.Direction.REVERSE);
   intakeRight.setDirection(CRServo.Direction.FORWARD);
}

   @Override
   public void loop() {


      // INTAKE

      if (gamepad2.left_bumper) { // out
         intakeLeft.setPower(-1);
         intakeRight.setPower(-1);
      }
      else if(gamepad2.right_bumper) {
         //continue in and move servo out of way
         intakeServo.setPosition(0.7);
         intakeLeft.setPower(0.8);
         intakeRight.setPower(0.8);
      }
      else if (gamepad2.right_trigger>0) { // in
         // down to pick up
         intakeServo.setPosition(0.3d);

         intakeLeft.setPower(1);
         intakeRight.setPower(1);
      }

      else{
         intakeServo.setPosition(0.3d);
         intakeRight.setPower(0);
         intakeLeft.setPower(0);
      }

   }
}
