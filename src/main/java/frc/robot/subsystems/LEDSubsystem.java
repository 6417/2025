// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Controls.HubturmState;
import frc.robot.Controls.IntakeState;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds;

    private AddressableLEDBuffer ledsBuffer;

    private static class RGB {
        public int red;
        public int green;
        public int blue;

        public RGB(int gray) {
            red = gray;
            green = gray;
            blue = gray;
        }

        public RGB(int red, int green, int blue) {
            this.red = green;
            this.green = red;
            this.blue = blue;
        }
    }

    private static RGB coral = new RGB(255, 255, 255);
    private static RGB climb = new RGB(255, 0, 0);
    private static RGB coralIn = new RGB(0, 255, 0);
    private static RGB noLight = new RGB(0);

    public LEDSubsystem() {
        leds = new AddressableLED(Constants.LEDs.ledPort);
        leds.setLength(Constants.LEDs.ledBufferLength);

        ledsBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);
        leds.start();

        normalLeds();
        setData();
    }

    private void setData() {
        leds.setData(ledsBuffer);
    }

    @Override
    public void periodic() {
    }

    private void setLEDsInBlocks(int blockLen, int blockOffsetFromStart, RGB color1, int lenCol1, RGB color2) {
        for (int i = blockOffsetFromStart; i < Constants.LEDs.ledBufferLength; i += blockLen) {
            for (int j = 0; j < blockLen; j++) {
                // Für die ersten drei LEDs die Farbe von "algue", danach aus.
                if (j < lenCol1) {
                    ledsBuffer.setRGB(i + j, color1.red, color1.green, color1.blue);
                } else if (j < blockLen) {
                    ledsBuffer.setRGB(i + j, color2.red, color2.green, color2.blue);
                }
            }
        }
        setData();
    }

    private void setLEDsVertical(RGB ledColorBottom, RGB ledColorTop) {
        for (int i = 0; i < 4; i++) {
            if (i % 2 == 0) {
                ledsBuffer.setRGB(i, ledColorBottom.red, ledColorBottom.green, ledColorBottom.blue);
            } else {

                ledsBuffer.setRGB(i, ledColorTop.red, ledColorTop.green, ledColorTop.blue);
            }
        }
    }

    private void setAllLEDs(RGB color) {
        for (int i = 0; i < Constants.LEDs.ledBufferLength; i++) {
            ledsBuffer.setRGB(i, color.red, color.green, color.blue);
        }
        setData();
    }

    public void coralIntakeLEDs() {
        setLEDsVertical(coral, new RGB(255, 0, 0));
        setData();
    }

    public void coralL2OuttakeLEDs() {
        setLEDsVertical(coral, new RGB(0, 220, 0));
        setData();
    }

    public void coralL3OuttakeLEDs() {
        setLEDsVertical(new RGB(220, 50, 220), coral);
        setData();
    }

    public void coralL4OuttakeLEDs() {
        setAllLEDs(coral);
        setData();
    }

    // public void rainbowAnimationLEDs() {
    //     LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);
    //     Distance kLedSpacing = Distance.ofBaseUnits(1 / 10, Meters);
    //     LEDPattern m_scrollingRainbow = rainbowPattern
    //             .scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(1, MetersPerSecond), kLedSpacing);
    //     m_scrollingRainbow.applyTo(ledsBuffer);
    //     setData();
    // }

    public void climbLEDs() {
        setAllLEDs(climb);
    }

    public void normalLeds() {
        setAllLEDs(climb);
    }

    public void coralLEDs() {
        setAllLEDs(coralIn);
    }

    // TODO: Implement this method
    public void synchronizeLEDsWithStates() {
        HubturmState hubturmState = RobotContainer.controls.getActiveLiftingTowerState();
        IntakeState intakeState = RobotContainer.controls.activeIntakeState;

        if (intakeState == IntakeState.INTAKE) {
            coralIntakeLEDs();
        } else {
            switch (hubturmState) {
                case LTWO:
                    coralL2OuttakeLEDs();
                    break;
                case LTHREE:
                    coralL3OuttakeLEDs();
                    break;
                case LFOUR:
                    coralL4OuttakeLEDs();
                    break;
                default:
                    break;
            }
        }
    }
}