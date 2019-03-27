package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Drew from 11874 on 3/26/2019.
 */

@Autonomous(name = "Reset Robot")
public class ResetArmAndHanger extends theColt {

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        linearSlide = hardwareMap.get(DcMotor.class, "liner slide");

        telemetry.log().add("Ready to start");

        waitForStart();

        telemetry.log().add("Lowering Arm...");

        for (int i = 0; i < 3 && opModeIsActive(); i++)
        {
            lowerArm(0, 0, 2);
            sleep(.1);
        }
        telemetry.log().add("Lowering Hanger");
        hanger.setPower(.5);
        while (opModeIsActive() && !isLimitSwitchPressed())
        {
            telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
            telemetry.update();
        }
        hanger.setPower(0);
        zeroHanger();

    }
}
