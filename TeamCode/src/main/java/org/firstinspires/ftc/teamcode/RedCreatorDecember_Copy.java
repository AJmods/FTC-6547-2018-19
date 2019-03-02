package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Drew from 11874 on 10/20/2018.
 */

@Autonomous(name = "Creater 2019")
public class RedCreatorDecember_Copy extends theColt {

    @Override
    public void runOpMode() {
        telemetry.addData("You broke it Harrison","");
        telemetry.update();

        INIT(hardwareMap);
        switchXandY();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
        
        initIMU();
    
        initTfod();
    
        zeroEncoders();
        //zeroHanger();
        angleZzeroValue=45;

        goldMineralLocation=GOLD_MINERAL_UNKNOWN;
        
        telemetry.log().add("UPDATED ","");
         hanger.setPower(.7);
        while (opModeIsActive() && !isLimitSwitchPressed())
        {
            telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
            telemetry.addData("limit switch volatage", limitSwitch.getVoltage());
            telemetry.update();
        }
        hanger.setPower(0);
        telemetry.log().add("hanger calibrated");
        tfod.activate();
        while (!isStarted())
        {
            scanMinerals();
        }
        if (goldMineralLocation==GOLD_MINERAL_UNKNOWN)
        {
            goldMineralLocation=GOLD_MINERAL_LEFT;
            telemetry.log().add("gold mienral UNKNOWN, defaulting to "+ goldMineralLocation);
        }
        else telemetry.log().add("gold mienral location "+ goldMineralLocation);

        waitForStart();
        
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);

        runtime.reset();
        while (goldMineralLocation==GOLD_MINERAL_UNKNOWN && opModeIsActive() && runtime.seconds()<2)
        {
            scanMinerals();
        }
        if (goldMineralLocation==GOLD_MINERAL_UNKNOWN)
        {
            goldMineralLocation=GOLD_MINERAL_LEFT;
            telemetry.log().add("gold mienral UNKNOWN, defaulting to "+ goldMineralLocation);
        }
        else telemetry.log().add("gold mienral location "+ goldMineralLocation);
        //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //Start IMU
    
        outputTelemetry();
        lowerRobot();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        sleep(.3);
        DriveforLength(1.1, -.3); //drive forward
        sleep(.3);
        driveIntoMineralCreator(45, goldMineralLocation);
        TurnPID(90, 1); //used to turn to 180 degrees, there's issues if it's one turn so it will be two turns
        TurnPID(180, 3);
        sleep(.3);
        //if (goldMineralLocation==GOLD_MINERAL_LEFT) DriveFieldRealtiveDistance(1, 90, 3.8);
        //if (goldMineralLocation==GOLD_MINERAL_RIGHT)
        //{
        //    DriveFieldRealtiveDistance(1, 90, .3);
        //    DriveFieldRealtiveDistance(1, 45, 1);
        //     DriveFieldRealtiveDistance(1, 135, 1);
        //      DriveFieldRealtiveDistance(1, 90, .4);
        //}
        //else 
        DriveFieldRealtiveDistance(1, 90, 3); 
        TurnPID(150, .5);
        deployTeamMarker(); //lines below drive to the crator
        TurnPID(90, 1);
        TurnPID(10, 2);
        //if (goldMineralLocation==GOLD_MINERAL_LEFT)  DriveFieldRealtiveDistance(.9, 280, 5.3);
        DriveFieldRealtiveDistance(.9, 280, 4.2);
        //DriveToPointPID(46,3,3);
        DriveFieldRealtiveDistance(.2, 280, .4);
        teamMarker.setPosition(1);
        stopRobot();
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle()-90);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        raiseRobot();
        while (opModeIsActive()) //after program is done
        {
            //maverick.DriveFoward(.05);
            telemetry.addData("PROGRAM DONE Encoder position", RightBack.getCurrentPosition());
            outputTelemetry();
        }


    }
}
