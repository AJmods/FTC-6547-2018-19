package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "YASEEN and DYLAN Teleop 2019")
public class TeleOp_YASEENandDYLAN extends theColt {

    double speedModifer=.7;
    
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    
    boolean feildRealtive=true;
    
    boolean isGamepadPressed=false;
    @Override
    public void runOpMode() {
        INIT(hardwareMap);

        initIMU();

        angleZzeroValue=0;
        
        //zeroEncoders();
        
        angleZzeroValue=-readFile(GYRO_ANGLE_FILE_NAME);
        writeFile(GYRO_ANGLE_FILE_NAME, 0);

        telemetry.log().add("Ready to start");
        waitForStart();
        mineralArm.setPosition(1);

        while (opModeIsActive()) {
            
            if (gamepad1.y) speedModifer=.25;
            if (gamepad1.b && !gamepad1.start) speedModifer=.7;
            if (gamepad1.a && !gamepad1.start) speedModifer=1;
            
            if (feildRealtive)
            {
                double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
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
            
            if (gamepad1.right_trigger>=.7 && gamepad1.left_trigger>=.7) angleZzeroValue=-22.5;
            
            if (gamepad1.left_bumper && gamepad1.right_bumper && angleZzeroValue==-45) angleZzeroValue=0;
            else if (gamepad1.left_bumper && gamepad1.right_bumper) 
            {
                angleZzeroValue=0;
                angleZzeroValue=-getIMUAngle();
            }
        
            arm.setPower(-gamepad2.right_stick_y);
            linearSlide.setPower(-gamepad2.right_stick_x);
            
            intake.setPower(gamepad2.right_trigger/1.5-gamepad2.left_trigger);
            
            if (gamepad2.a) teamMarker.setPosition(0);
            else teamMarker.setPosition(1);
        
            telemetry.addData("A is full speed, B is half speed, Y is quarter speed","");
            telemetry.addData("Field Realitive Driving ", feildRealtive);
            telemetry.addData("Speed modifer", speedModifer);
            telemetry.update();
        }
        stopRobot();
    }
}
