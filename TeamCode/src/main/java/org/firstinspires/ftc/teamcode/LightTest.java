package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Drew from 11874 on 1/20/2019.
 */

@TeleOp(name="Light test")
public class LightTest extends theColt {

    @Override
    public void runOpMode() {

        INIT(hardwareMap);
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;

        lights.setPattern(pattern);

        telemetry.log().add("Push A and B to cycle through light patterns");
        waitForStart();

boolean pressed=false;
boolean pressedB=false;
        while (opModeIsActive())
        {
            if (gamepad1.a && !pressed) 
            {
                pattern=pattern.next();
                pressed=true;
            }
            else if (pressed && !gamepad1.a) pressed=false;
            if (gamepad1.b && pressedB) 
            {
                pattern=pattern.previous();
                pressedB=true;
            }  else if (pressedB && !gamepad1.b) pressedB=false;
            

            lights.setPattern(pattern);
            telemetry.addData("Pattern name", pattern.name());
            //telemetry.addData("Pattern toString", pattern.toString());
            telemetry.update();
        }
    }
}
