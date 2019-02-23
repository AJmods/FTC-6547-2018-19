package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TestPID extends LinearOpMode {


    public void runOpMode()
    {

    MiniPID miniPID;

    miniPID = new MiniPID(.05, 0, 0);
    miniPID.setOutputLimits(10);
    //miniPID.setMaxIOutput(2);
    //miniPID.setOutputRampRate(3);
    //miniPID.setOutputFilter(.3);
        miniPID.setSetpointRange(40);

waitForStart();

    double target=24;

    double actual=48;
    double output=0;

        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);

        telemetry.log().add("Target\tActual\tOutput\tError\n");
    //System.err.printf("Output\tP\tI\tD\n");
    
        telemetry.log().setCapacity(100);

    // Position based test code
        for (int i = 0; i < 100; i++){
            
            if (i==80) actual=12;

        output = miniPID.getOutput(actual, target);
        actual = actual + output;

        telemetry.log().add("%3.2f\t%3.2f\t%3.2f\t%3.2f\n", target, actual, output, (target-actual));
    }
    
    while (opModeIsActive());
}
}
