package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Hungor Cailbrate")
public class HangerCaliabrate extends theColt {

    @Override
    public void runOpMode() {

        INIT(hardwareMap);

        telemetry.log().add("Ready to start");
        waitForStart();
        
        hanger.setPower(.5);
        while (opModeIsActive() && !isLimitSwitchPressed())
        {
            telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
            telemetry.update();
        }
        hanger.setPower(0);
        zeroHanger();
        telemetry.log().add("zeroed hanger");
        telemetry.log().add("Use the left stick on gamepad 2 to move the hanger");
        telemetry.log().add("Then, push A to zero the hanger.");
        while (opModeIsActive())
        {
            hanger.setPower(gamepad2.left_stick_y/2);
            if (gamepad2.a) zeroHanger();
            telemetry.addData("Hanger current Position", hanger.getCurrentPosition());
            telemetry.update();
        }
    }
}
