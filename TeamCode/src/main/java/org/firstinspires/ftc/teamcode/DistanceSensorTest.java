package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Strafe PID test")

public class DistanceSensorTest extends theColt {

    @Override
    public void runOpMode()
    {
        INIT(hardwareMap);
        
        initIMU();
        
        telemetry.log().add("ready to start");
        
        waitForStart();
        strafeToDistanceYPID(24,10);
        //setEquation(new double[] {15}); //25
        //driveOverLinePID(30, 5, 3);
        //strafeToDistanceYPID(24,3);
        //strafeToDistanceXPID(30,3);
        //strafeToDistanceXPID(10,3);
        //strafeToDistanceYPID(5,2);
        //strafeToDistanceXPID(25,2);
        //strafeToDistanceYPID(20,2);
    }
}
