package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Drew from 11874 on 10/20/2018.
 */

@Autonomous(name = "Dapet 2019")
public class RedDepotDecember_Copy extends theColt {

    ElapsedTime gametime = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addData("You broke it Harrison","");
        telemetry.update();
        
        INIT(hardwareMap);
        
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        
        initIMU();
        
        initTfod();
        
        zeroEncoders();
        
        angleZzeroValue=45;
        
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
        //make sure the robot is facing straight towards the depot for deploying the team marker
        //if (goldMineralLocation==GOLD_MINERAL_LEFT && getRobotPositionX()>24) do
        //{
            //setEquation(new double[] {3});
            //driveOverLine(.5, 24, 72, true);
        //     DriveToPointPID(24,3,1);
        //     TurnPID(0,2);
        //} while (getRobotPositionX()>24);
        //if (goldMineralLocation==GOLD_MINERAL_RIGHT) DriveforLength(.8, -.5);
        deployTeamMarker();
        if (goldMineralLocation==GOLD_MINERAL_CENTER) TurnPID(0,1);
        DriveToPointPID(30,3,1); //to halfway creater
        TurnPID(0,1);
        //setEquation(new double[] {3});
        //do {
        DriveToPointPID(48,2.5,1.5); //to halfway creater
        TurnPID(0, .5);
        //} while (getRobotPositionX()<40 && (distanceSensorX.getDistance(DistanceUnit.INCH))!=0 && (distanceSensorY.getDistance(DistanceUnit.INCH))!=0 && gametime.seconds()<20);
        DriveFieldRealtiveDistance(.5, 90, 3.5);
        TurnPID(0,.5);
        DriveFieldRealtiveDistance(.1, 90,.13);
        stopRobot();                                                    //at long distances, distance sensors are unreliable.
        writeFile(GYRO_ANGLE_FILE_NAME, getIMUAngle()); //save the current gyro angle for later use in the field realtive teleop
        teamMarker.setPosition(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        raiseRobot();
        telemetry.log().add("Autonomous is done, if might have failed but hopefully it didn't");
        while (opModeIsActive()) //after program is done
        {
            telemetry.addData("current angle", getIMUAngle());
            telemetry.update();
        }


    }
}
