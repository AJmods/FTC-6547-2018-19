package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "DLYAN AND HERRIOSON Tele-op 2019")
public class TeleOp_DYLANandHERIOSON extends theColt {

    double speedModifer=.6;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    boolean feildRealtive=true;

    boolean isGamepadPressed=false;
    @Override
    public void runOpMode() {
        INIT(hardwareMap);
         lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        telemetry.addData("You broke it Harrison","");
        telemetry.update();
        initIMU();

        angleZzeroValue=0;

        //zeroEncoders();

        angleZzeroValue=readFile(GYRO_ANGLE_FILE_NAME);
        writeFile(GYRO_ANGLE_FILE_NAME, 0);
        double angleZzeroValueMinus45=angleZzeroValue+135;
        double tempZeroValue= angleZzeroValue;

        ElapsedTime gameTime=new ElapsedTime();
        telemetry.log().add("Ready to start");
        waitForStart();
        gameTime.reset();
        mineralArm.setPosition(1);

        boolean at30=true;
        boolean at15=false;
        boolean offset=false;
        boolean pressed=false;
        while (opModeIsActive()) {

            if (gamepad1.y) speedModifer=.3;
            if (gamepad1.b && !gamepad1.start) speedModifer=.6;
            if (gamepad1.a && !gamepad1.start) speedModifer=1;

            if (feildRealtive)
            {
                if (gameTime.seconds()<90) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                if (offset) LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4)+Math.toRadians(45);
                double robotAngle = Math.toRadians(getIMUAngle());
                double rightX=gamepad1.right_stick_x;
                leftFrontPower =  speed * Math.cos(LeftStickAngle-robotAngle) + rightX;
                rightFrontPower =  speed * Math.sin(LeftStickAngle-robotAngle) - rightX;
                leftBackPower =  speed * Math.sin(LeftStickAngle-robotAngle) + rightX;
                rightBackPower =  speed * Math.cos(LeftStickAngle-robotAngle) - rightX;

                if (gamepad1.x && !isGamepadPressed)
                {
                    feildRealtive = false;
                    isGamepadPressed=true;
                } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            }
            else
            {
                if (gameTime.seconds()<90) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                leftFrontPower=-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightFrontPower=-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                leftBackPower=-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                rightBackPower=-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

                if (gamepad1.x && !isGamepadPressed)
                {
                    feildRealtive = true;
                    isGamepadPressed=true;
                } else if (!gamepad1.x && isGamepadPressed) isGamepadPressed=false;

            }

            LeftFront.setPower(leftFrontPower*speedModifer);
            RightFront.setPower(rightFrontPower*speedModifer);
            LeftBack.setPower(leftBackPower*speedModifer);
            RightBack.setPower(rightBackPower*speedModifer);

            if (gamepad2.left_stick_y>0 && hanger.getCurrentPosition()>-300) hanger.setPower(0);
            else if (gamepad2.left_stick_y<0 && hanger.getCurrentPosition()<-7200) hanger.setPower(0);
            else hanger.setPower(gamepad2.left_stick_y);

           
            if (pressed)
            {
                if (gamepad1.right_trigger<.5 && gamepad1.left_trigger<.5) pressed=false;
            }
            else if (gamepad1.right_trigger>=.7 && gamepad1.left_trigger>=.7 && !offset)
            {
                //angleZzeroValue=angleZzeroValueMinus45;
                offset=true;
                pressed=true;
            }
            if (pressed)
            {
                if (gamepad1.right_trigger<.5 && gamepad1.left_trigger<.5) pressed=false;
            }
            else if (gamepad1.right_trigger>=.7 && gamepad1.left_trigger>=.7 && offset)
            {
                offset=false;
                //angleZzeroValue=tempZeroValue;
                pressed=true;
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper)
            {
                offset=false;
                angleZzeroValue=0;
                angleZzeroValue=-getIMUAngle();
            }

            arm.setPower(-gamepad2.right_stick_y);
            linearSlide.setPower(-gamepad2.right_stick_x);

            intake.setPower(gamepad2.right_trigger/1.5-gamepad2.left_trigger);


            if (gamepad2.a) teamMarker.setPosition(0);
            else teamMarker.setPosition(1);
            
            
            if (gameTime.seconds()>=122) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            else if (gameTime.seconds()>=107) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
            else if (gameTime.seconds()>=92) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);



            // telemetry.addData("Left Front current Pos", LeftFront.getCurrentPosition());
            // telemetry.addData("Left Back current Pos", LeftBack.getCurrentPosition());
            // telemetry.addData("Right Front current Pos", RightFront.getCurrentPosition());
            // telemetry.addData("Right Back current Pos", RightBack.getCurrentPosition());
            telemetry.addData("A is full speed, B is half speed, Y is quarter speed","");
            telemetry.addData("TIME", gameTime.seconds());
            telemetry.addData("Field Realitive Driving ", feildRealtive);
            // telemetry.addData("Left Trigger", gamepad1.left_trigger);
            // telemetry.addData("Right Trigger", gamepad1.right_trigger);
            // telemetry.addData("Left Front Power",leftFrontPower);
            // telemetry.addData("Right Front Power",rightFrontPower);
            // telemetry.addData("Left Back Power",leftBackPower);
            // telemetry.addData("Right Back Power",rightBackPower);
            telemetry.addData("intake/outake motor power", intake.getPower());
            telemetry.addData("Team marker servo pos" , teamMarker.getPosition());
            telemetry.addData("Speed modifer", speedModifer);
            telemetry.addData("hanger motor current position", hanger.getCurrentPosition());
            telemetry.update();
        }
        stopRobot();
    }
}
