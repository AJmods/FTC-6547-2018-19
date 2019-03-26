package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Drew from 11874 on 3/24/2019.
 */

@TeleOp(name = "dapat WORLDS V2")
public class RedDepatWorldsV2 extends theColt {

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

        lowerRobot();

        DriveforLength(.5,-.4); //drive away from lander

        TurnPIDandLowerArm(315,1,.7,.5); //turn to face to depot, prepare to deplot team marker
        lowerArm(.7,.95,2);

        outtake(1); //deploy team marker into depot
        sleep(1.0); //wait one second
        stopIntake(); //stop outtaking

        turnToFaceMineral(0,315,270,.7,.1,2);
        lowerArm(1,.1,1); //lower to grab mineral

        intake(1); //intake mineral
        sleep(.2); //wait a bit
        turnToFaceMineral(10,305, 280,1,.1,1); //turn a bit, will at least knock the gold mineral off

        //Score in lander

        lowerArm(.5,.1,.5); //raise the arm a bit

        stopIntake(); //stop the intake, the mineral is already in there

        TurnPIDandLowerArm(135,2,.5,.95); //turn to face lander, raise are to prepare to score

        outtake(1); //deploy mineral in lander
        sleep(1.0); //wait one second
        stopIntake(); //stop outtaking

        //go to creator

        DriveforLength(2,.6);

        TurnPID(90,2);  //turn to face wall

        DriveToPointPID(60,3,2,90); //go to wall

        DriveToPointPID(50,3,2,90); //go to creator

        lowerArm(1,.7,2); //lower arm to grab mineral

        intake(1);

        telemetry.log().add("DONE");
        while (opModeIsActive());


    }
    void turnToFaceMineral(double degreesLEFT, double degreesCENTER, double degreesRIGHT, double armlowerPercent, double armExtendpercent, double time)
    {
        if (goldMineralLocation==GOLD_MINERAL_RIGHT) TurnPIDandLowerArm(degreesRIGHT, time, armlowerPercent,armExtendpercent);
        else if (goldMineralLocation==GOLD_MINERAL_CENTER) TurnPIDandLowerArm(degreesCENTER, time, armlowerPercent,armExtendpercent);
        else if (goldMineralLocation==GOLD_MINERAL_LEFT) TurnPIDandLowerArm(degreesLEFT, time, armlowerPercent,armExtendpercent);
    }
}
