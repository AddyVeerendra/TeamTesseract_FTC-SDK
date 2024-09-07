package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SliderClassTest {
    private final DcMotorEx motor;

    private final Servo leftServo;
    private final Servo rightServo;

    double lastSetArmPos = 0;
    boolean armMotorBusy = false;

    public SliderClassTest(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo = hardwareMap.get(Servo.class, "leftGripperServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public void moveArmToPosition(double targetPos) {
        lastSetArmPos = targetPos;

        double error = targetPos - motor.getCurrentPosition(); // Change error calculation

        if (Math.abs(error) > 20) { // Ensure error threshold is handled correctly
            motor.setPower(Range.clip((error * 0.005), -1, 1)); // Adjust power direction
            armMotorBusy = true;
        } else {
            motor.setPower(0);
            armMotorBusy = false;
        }
    }

    public void correctArmPosition() {
        if (!armMotorBusy) {
            double error = lastSetArmPos - motor.getCurrentPosition(); // Change error calculation
            motor.setPower(Range.clip((error * 0.005), -0.5, 0.5)); // Adjust power direction
        }
    }

    public void update() {
        if (isArmMotorBusy()) {
            moveArmToPosition(lastSetArmPos);
        } else {
            correctArmPosition();
        }
    }

    public boolean isArmMotorBusy() {
        return armMotorBusy;
    }

    public void OpenServos() {
        leftServo.setPosition(0.4);
        rightServo.setPosition(0.6);
    }
}