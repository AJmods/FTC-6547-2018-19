package org.firstinspires.ftc.teamcode.DrewsPrograms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class ArmPIDtest extends theColt {

    /*
    How the lower arm works: (if you can't figure out what I'm saying, DO NOT MODIFY THESE METHODS
    there are two important values for each the arm and the linear slide: 0% and 100%
    0% is all the way retracted and 100% is moved all the way
    in the lowerArm method, it using PID control to move the arms because hey, it turns out it's easier to work with
    the lowerArm method takes in two parameters: percent the arm is lowered down and percent the linear slide is extracted.
    If you wish to move only one, ut getCurrentPercentArmLowardMethod for the first parameter (the arm won't move)
    Or for the second parameter, use the getCurrentPercentArmExtended method and the linear slide will not move
     */
    @Override
    public void runOpMode()
    {
        telemetry.log().add("ARM PID test");
        waitForStart();
        //the line below moves the arm to 95% of the way
        lowerArm(.95, getCurrentPercentArmExtended(),5);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        //below is moving the linear slide, above is moving the arm.
        telemetry.log().add("Preparing to test Linear Slide");
        runtime.reset();
        sleep(2000); //this just adds a 2 second delay.  Comment it out if you don't want it
        //the line below moved the linear slide 95% of it's full capacity.
        lowerArm(getCurrentPercentArmLowered(), .95,5);
    }
}
