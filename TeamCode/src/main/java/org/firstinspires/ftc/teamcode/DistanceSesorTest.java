package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class DistanceSesorTest extends theColt {

    @Override
    public void runOpMode()
    {
        INIT(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
             telemetry.addData("Pos X", getRobotPositionX());
             telemetry.addData("Pos Y", getRobotPositionY());
             telemetry.update();
        }
    }
}
