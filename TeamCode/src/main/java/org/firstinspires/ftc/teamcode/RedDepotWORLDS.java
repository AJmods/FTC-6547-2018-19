package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Dapet Worlds")
public class RedDepotWORLDS extends theColt {

    ElapsedTime gametime = new ElapsedTime();

    int totalMinerals=0;

    boolean turningRight=false;

    double timeToGetGold=6;
    double gyroAngle=0;
    double armExtentionPos=0;
    double armPos=0;
    double gyroAngle1=0;
    double armExtentionPos1=0;
    double armPos1=0;
    @Override
    public void runOpMode() {
        telemetry.addData("You broke it Harrison","");
        telemetry.update();
        timeToGetGold=30-timeToGetGold;

        INIT(hardwareMap);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

        initIMU();

        initTfod();

        zeroEncoders();

        angleZzeroValue=45;

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
        gametime.reset();
        //if the program was started and the gold mineral wasn't detected, default to the GOLD_MINERAL_LEFT position
        if (goldMineralLocation==GOLD_MINERAL_UNKNOWN)
        {
            goldMineralLocation=GOLD_MINERAL_LEFT;
            telemetry.log().add("gold mineral UNKNOWN, defaulting to LEFT");
        }
        else telemetry.log().add("gold Mineral location "+ goldMineralLocation);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //Start IMU

        outputTelemetry();
        lowerRobot();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
        sleep(.3);
        DriveforLength(1,  -.4); //drive forward out of the lander
        driveIntoMineral(-45, goldMineralLocation); //this method knocks over the gold mineral and goes to depot
        deployTeamMarker();
        if (goldMineralLocation==GOLD_MINERAL_CENTER) 
        {
            TurnPID(0,1);
            DriveforLength(1,  .4);
        }
        DriveToPointPID(30,3,1); //get out of depot
        //below are old methods used for going to the crator and stoping.
        DriveToPointPID(48,2.5,1.5); //get halfway to depot
        TurnPID(0, .5);
        //} while (getRobotPositionX()<40 && (distanceSensorX.getDistance(DistanceUnit.INCH))!=0 && (distanceSensorY.getDistance(DistanceUnit.INCH))!=0 && gametime.seconds()<20);
        DriveFieldRealtiveDistance(.5, 90, 3.3);
        TurnPID(90,2);
        //TurnPIDandLowerArm(90,2,.5,.5);
        //DriveFieldRealtiveDistanceAndLowerArm(.7, 91, 4.6, .6, .6);
        //sleep(1);
        //TurnPID(90,2);
        //sleep(1);
        sleep(1);
        lowerArm(1, 1,2);
        sleep(1);
        //driveToPointPIDandLowerArm(3,48,.6,.6,6);
        sleep(1);
        //lowerArm(.9, .9,5);
        //while (opModeIsActive());
        runtime.reset();
        intake(1);
        //sortMinerals(); //grab balls and move them out of the way, end of auton the robot grabs golds for tele-op
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle()); //save the current gyro angle for later use in the field realtive teleop
        teamMarker.setPosition(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        //raiseRobot();
        telemetry.log().add("Autonomous is done, if might have failed but hopefully it didn't");
        while (opModeIsActive()) //after program is done
        {
            telemetry.addData("current Angle:", getIMUAngle());
            telemetry.update();
        }
    }
    void sortMinerals()
    {
        //if (getCurrentPercentArmLowered()<.9) lowerArm(.95, .95,1); //make sure arm is lowered
        while (opModeIsActive() && !isTimeToGetGold() && !turningRight)
        {
            if (turnLeftUntilBall(.05,135))
            {
                //lowerArm(getCurrentPercentArmLowered(), (getCurrentPercentArmExtended()-.1),.5);
                intake(1);
                runtime.reset();
                while (opModeIsActive() && !isTimeToGetGold() && runtime.seconds()<.8)
                {
                    telemetry.addData("Grabbing ball minerals","");
                    telemetry.update();
                }
                stopIntake();
                totalMinerals++;
                if (totalMinerals<=1 && getIMUAngle()<=130) continue; //go to beginging of loop, contine turning left
            }
            //turned left returned false, turn right.
            else if (!isTimeToGetGold() && opModeIsActive() && totalMinerals!=2)
            {
                turningRight=true;
                if (turnRightUntilBall(.05,90))
                {
                    //lowerArm(getCurrentPercentArmLowered(), (getCurrentPercentArmExtended()-.1),.5);
                    intake(1);
                    runtime.reset();
                    while (opModeIsActive() && !isTimeToGetGold() && runtime.seconds()<.8)
                    {
                        telemetry.addData("Grabbing ball minerals","");
                        telemetry.update();
                    }
                    stopIntake();
                    totalMinerals++;
                }
                if (getIMUAngle()<=95) turningRight=false;
            }
            if (totalMinerals>=2)
            {
                telemetry.log().add("dropping off balls");
                double oldArmPos=getCurrentPercentArmLowered();
                double oldArmExtendingPos=getCurrentPercentArmExtended();
                double oldIMUAngle=getIMUAngle();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
                TurnPIDandLowerArm(140, 1, .6, .6);
                outtake(1);
                sleep(1);
                totalMinerals=0;
                stopIntake();
                TurnPIDandLowerArm(oldIMUAngle, 1, oldArmPos, oldArmExtendingPos);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
            }

        }
        //time is almost up, time to grab GOLD
        telemetry.log().add("Getting gold minerals with " + (30-gametime.seconds() + " remaining"));
        while (opModeIsActive() && runtime.seconds()<.8) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        TurnPIDandLowerArm(gyroAngle, 1,armPos, armExtentionPos);
        intake(1);
        runtime.reset();
        while (runtime.seconds()<.7);
        stopIntake();
        TurnPIDandLowerArm(gyroAngle1, 1, armPos1, armExtentionPos1);
        intake(1);
    }
    boolean turnRightUntilBall(double power, double angle)
    {
        turnLeft(Math.abs(power));
        while (opModeIsActive() && !isTimeToGetGold() && !isSilverMineral() && getIMUAngle()>angle)
        {
            telemetry.addData("Color sensor value", colorSensor.argb());
            telemetry.update();
            if (isGoldMineral())
            {
                if (gyroAngle==0)
                {
                    gyroAngle=getIMUAngle();
                    armExtentionPos=getCurrentPercentArmExtended()-1.;
                    armPos=getCurrentPercentArmLowered()-.1;
                }
                if (gyroAngle1==0)
                {
                    gyroAngle1=getIMUAngle();
                    armExtentionPos1=getCurrentPercentArmExtended()-1.;
                    armPos1=getCurrentPercentArmLowered()-.1;
                }
            }
        }
        //found silver mineral
        stopRobot();
        if (isSilverMineral())
        {
            telemetry.log().add("Found Ball");
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
            return true;
        }
        return false;
    }
    boolean turnLeftUntilBall(double power, double angle)
    {
        turnLeft(-Math.abs(power));
        while (opModeIsActive() && !isTimeToGetGold() && !isSilverMineral() && getIMUAngle()<angle)
        {
            if (isGoldMineral())
            {
                if (gyroAngle==0)
                {
                    gyroAngle=getIMUAngle();
                    armExtentionPos=getCurrentPercentArmExtended()-1.;
                    armPos=getCurrentPercentArmLowered()-.1;
                }
                if (gyroAngle1==0)
                {
                    gyroAngle1=getIMUAngle();
                    armExtentionPos1=getCurrentPercentArmExtended()-1.;
                    armPos1=getCurrentPercentArmLowered()-.1;
                }
            }
            telemetry.addData("Color sensor value", colorSensor.argb());
            telemetry.update();
        }
        //found silver mineral
        stopRobot();
        if (isSilverMineral())
        {
            telemetry.log().add("Found Ball");
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
            return true;
        }
        return false;
    }
    boolean isTimeToGetGold()
    {
        return (gametime.seconds()<timeToGetGold);
    }
}
