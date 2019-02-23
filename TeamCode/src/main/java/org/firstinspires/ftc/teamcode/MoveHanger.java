package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move Hanger")
public class MoveHanger extends theColt {

    @Override
    public void runOpMode() {

        INIT(hardwareMap);

        telemetry.log().add("Ready to start");
        telemetry.log().add("Use the left stick on gamepad 2 to move the hanger");
        //telemetry.log().add("Then, push A to zero the hanger.");
        waitForStart();

        while (opModeIsActive())
        {
            hanger.setPower(gamepad2.left_stick_y/2);
            if (gamepad2.a) zeroHanger();
            telemetry.addData("Hanger current Position", hanger.getCurrentPosition());
            telemetry.update();
        }
    }
}
