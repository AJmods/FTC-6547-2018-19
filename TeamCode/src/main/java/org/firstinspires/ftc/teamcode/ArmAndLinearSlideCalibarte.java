package org.firstinspires.ftc.teamcode.DrewsPrograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm and Linear Slide Calibrate")
public class ArmAndLinearSlideCalibarte extends theColt {

    public void runOpMode()
    {
        telemetry.log().add("Arm and Linear Slide Calibrate");
        telemetry.log().add("use the left stick to move the arm");
        telemetry.log().add("use the right stick to move the linear slide");
        telemetry.log().add("A zeros the arm, B zeros the linear slide");
        waitForStart();
        while (opModeIsActive())
        {
            linearSlide.setPower(gamepad1.right_stick_y);
            arm.setPower(gamepad1.left_stick_x);
            if (gamepad1.a) zeroArm();
            if (gamepad1.b) zeroLinearSlide();
            telemetry.addData("ARM current Position", arm.getCurrentPosition());
            telemetry.addData("LINEAR SLIDE current Position", linearSlide.getCurrentPosition());
        }
    }
}
