package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp

public class LimitSwitchTest extends theColt {
    
    @Override
    public void runOpMode()
    {
        INIT(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
            telemetry.update();
        }
    }
    
}
