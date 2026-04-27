package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private final DcMotorEx intakeMotor1;
    private final DcMotorEx intakeMotor2;
    private final Servo gate;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake2");
        gate = hardwareMap.get(Servo.class, "gate");

        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
    }

    // call in opmode start() to zero everything out
    public void start() {
        intakeMotor1.setVelocity(0.0);
        intakeMotor2.setVelocity(0.0);
        gate.setPosition(0.3);
    }

    public void intakeOn() {
        intakeMotor1.setVelocity(10000);
        intakeMotor2.setVelocity(10000);
    }

    public void intakeReverse() {
        intakeMotor1.setVelocity(-10000);
        intakeMotor2.setVelocity(-10000);
    }

    public void intakeOff() {
        intakeMotor1.setVelocity(0.0);
        intakeMotor2.setVelocity(0.0);
    }

    public void gateOpen() {
        gate.setPosition(0.6);
    }

    public void gateClose() {
        gate.setPosition(0.3);
    }
}