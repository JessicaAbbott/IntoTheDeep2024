package org.firstinspires.ftc.teamcode.IntoTheDeep.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class sampleCode extends OpMode {

   DcMotor left;

   DcMotor right;

   @Override
   public void init() {

      left = hardwareMap.get(DcMotor.class, "left");
      left.setDirection(DcMotor.Direction.FORWARD);

      right = hardwareMap.get(DcMotor.class, "right");
      right.setDirection(DcMotor.Direction.FORWARD);
   }

   @Override
   public void loop() {

      left.setPower(1);
      right.setPower(1);

   }
}
