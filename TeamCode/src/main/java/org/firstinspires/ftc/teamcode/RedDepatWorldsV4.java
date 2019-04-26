package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Drew from 6547 on 3/24/2019.
 */

@Autonomous(name = "Dapet WORLDS V4")
public class RedDepatWorldsV4 extends theColt {

    @Override
    public void runOpMode() {
        telemetry.addData("You broke it Harrison", "");
        telemetry.update();

        INIT(hardwareMap);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

        initIMU();

        initTfod();

        zeroEncoders();

        angleZzeroValue = 45;

        //  hanger.setPower(.7);
        // while (opModeIsActive() && !isLimitSwitchPressed())
        // {
        //     telemetry.addData("limit switch pressed?", isLimitSwitchPressed());
        //     telemetry.addData("limit switch volatage", limitSwitch.getVoltage());
        //     telemetry.update();
        // }
        // hanger.setPower(0);
        // telemetry.log().add("hanger calibrated");
        tfod.activate();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);

        while (!isStarted()) //scan until the program starts
        {
            scanMinerals();
        }
        //if the program was started and the gold mineral wasn't detected, default to the GOLD_MINERAL_LEFT position
        if (goldMineralLocation == GOLD_MINERAL_UNKNOWN) {
            goldMineralLocation = GOLD_MINERAL_LEFT;
            telemetry.log().add("gold mineral UNKNOWN, defaulting to LEFT");
        } else telemetry.log().add("gold Mineral location " + goldMineralLocation);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //Start IMU

        outputTelemetry();

        intake(.2); //intake to keep the team marker in and not fall out

        lowerRobot();

        DriveFieldRealtiveDistanceAndLowerArm(.4,325,.45,.95,.95);
        //DriveforLength(.5,-.4); //drive away from lander

        //turn to face to depot, prepare to deplot team marker
        //TurnPIDandLowerArm(325,2,.7,.7);
        //TurnPID(330,2,1.25);

        // DriveFieldRealtiveDistanceAndLowerArm(.5,325, 1,.95,1); //strafe to depot, extend arm to deplot team marker
        // telemetry.log().add("1");
        // lowerArm(.95,1,1.5);
        // telemetry.log().add("2");
        // //strafeLeft(-.3,600);

        // outtake(1); //deploy team marker into depot
        // sleep(.5); //wait one second
        // stopIntake(); //stop outtaking
        // lowerArm(.8,.8,1);
        // telemetry.log().add("3");
        // DriveFieldRealtiveDistanceAndLowerArm(.4,145, .7,.9,.1); //drive back
        // //strafeLeft(.3,600);
        if (goldMineralLocation==GOLD_MINERAL_RIGHT) TurnPID(320,.5);
        turnToFaceMineral(5,325,290,2.5);
        if (goldMineralLocation==GOLD_MINERAL_CENTER) lowerArm(1,.1,2); //lower to grab mineral
        else lowerArm(1.05,.43,2);
        intake(1); //intake mineral
        runtime.reset();
        turnToFaceMineral(350,310, 300,1);
        intake(.5);
        //LowerArm(1,.1,1); //turn a bit, will at least knock the gold mineral off

        //Score in lander

        lowerArm(.5,.5,2); //raise the arm a bit
        //DriveFieldRealtiveDistance(.5,315,.15);
        telemetry.log().add("4");

        stopIntake(); //stop the intake, the mineral is already in there
        telemetry.log().add("5.");

        //TurnPIDandLowerArm(145,2,.5,.96);
        if (goldMineralLocation==GOLD_MINERAL_LEFT) 
        {
            TurnPID(315,1,1.25);
            TurnPID(150,2,1);
        }
        else TurnPID(156,3,1);
        DriveFieldRealtiveDistanceAndLowerArm(.4,325,.1,.50,.9);
        runtime.reset();
        lowerArm(.53,.905,2); //turn to face lander, raise are to prepare to score

        outtake(.3); //deploy mineral in lander
        sleep(.3);
        outtake(.5);
        sleep(.9); //wait one second
        stopIntake(); //stop outtaking
        //lowerArm(.52, .97,1); //raise arm to not get stuck in lander

        //go to creator

        //DriveFieldRealtiveDistance(.7,315,.18);
        //strafeLeft(.3,400);
        if (goldMineralLocation==GOLD_MINERAL_LEFT) DriveFieldRealtiveDistanceAndLowerArm(1,35,1.6,.1,.1);
        else if (goldMineralLocation==GOLD_MINERAL_RIGHT) DriveFieldRealtiveDistanceAndLowerArm(1,45,1.9,.1,.1);
        else DriveFieldRealtiveDistanceAndLowerArm(1,45,1.8,.1,.1);
        //DriveforLength(2,-.6);

        //DriveFieldRealtiveDistanceAndLowerArm(1,0,1.3,.1,.1);
        DriveFieldRealtiveDistanceAndTurnPID(1,15,3,0,1);
         DriveFieldRealtiveDistance(1,15,.4);
        TurnPID(0,.2);
        if (goldMineralLocation!=GOLD_MINERAL_CENTER) DriveFieldRealtiveDistanceAndLowerArm(1.1,290,2.4,.1,.1);
        else DriveFieldRealtiveDistanceAndLowerArm(1.1,295,2.7,.1,.1);
        deployTeamMarker();
        TurnPID(110,.5,2);
        TurnPID(110,1);
        //runtime.reset();
        // TurnPID(0,3);  //turn to face wall
        // DriveToPointPID(20,3,1); //to halfway creater
       
        //setEquation(new double[] {3});
        //do {
       // DriveToPointPID(48,2.5,1.5); //to halfway creater
    
        //TurnPID(100,2);
        //DriveforLength(1.5,-.6);
        
        intake(1);

        DriveFieldRealtiveDistanceAndLowerArm(1,90,5.5,.85,.7);
        //strafeLeft(-.6,1000);

        lowerArm(1.05,.7,.7); //lower arm to grab mineral
        
        TurnPID(110,3);
        
         writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle());

        telemetry.log().add("DONE");
        while (opModeIsActive());


    }
    void turnToFaceMineral(double degreesLEFT, double degreesCENTER, double degreesRIGHT,double time)
    {
        // if (goldMineralLocation==GOLD_MINERAL_RIGHT) TurnPIDandLowerArm(degreesRIGHT, time,armlowerPercent,armExtendpercent);
        // else if (goldMineralLocation==GOLD_MINERAL_CENTER) TurnPIDandLowerArm(degreesCENTER, time,armlowerPercent,armExtendpercent);
        // else if (goldMineralLocation==GOLD_MINERAL_LEFT) TurnPIDandLowerArm(degreesLEFT, time,armlowerPercent,armExtendpercent);
        if (goldMineralLocation==GOLD_MINERAL_RIGHT) TurnPID(degreesRIGHT, time);
        else if (goldMineralLocation==GOLD_MINERAL_CENTER) TurnPID(degreesCENTER, time);
        else if (goldMineralLocation==GOLD_MINERAL_LEFT) TurnPID(degreesLEFT, time);
        //lowerArm(armlowerPercent, armExtendpercent, time);
    }
}

