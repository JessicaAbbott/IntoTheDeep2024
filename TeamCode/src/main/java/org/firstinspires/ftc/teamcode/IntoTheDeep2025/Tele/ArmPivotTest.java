package org.firstinspires.ftc.teamcode.IntoTheDeep2025.Tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled

public class ArmPivotTest extends OpMode {
   public DcMotor armPivot;

   @Override
   public void init(){
      armPivot=hardwareMap.get(DcMotor.class,"armPivot");

      armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      armPivot.setDirection(CRServo.Direction.FORWARD);
   }

   @Override
   public void loop() {


      armPivot.setPower(-1);


   }
}
