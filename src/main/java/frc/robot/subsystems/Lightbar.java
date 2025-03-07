// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;


public class Lightbar extends SubsystemBase {

  // Create an LED pattern that sets the entire strip to solid red
      LEDPattern red = LEDPattern.solid(Color.kRed);
      LEDPattern orange_red = LEDPattern.solid(Color.kOrangeRed);
      LEDPattern orange = LEDPattern.solid(Color.kOrange);
      LEDPattern yellow = LEDPattern.solid(Color.kYellow);
      LEDPattern blue = LEDPattern.solid(Color.kBlue);
      LEDPattern purple = LEDPattern.solid(Color.kPurple);
      LEDPattern pink = LEDPattern.solid(Color.kPink);
      // Create an LED pattern that sets the entire strip to solid red
      LEDPattern green = LEDPattern.solid(Color.kGreen);
      // Create an LED pattern that sets the entire strip to solid red
      LEDPattern black = LEDPattern.solid(Color.kBlack);
      // LEDPattern brightnessX = red.atBrightness(Units.Percent.of(100*Math.abs(xSpeed)));

  public final AddressableLED m_led;
  // public final AddressableLEDBuffer ledStrip; 

  /** Creates a new Lightbar. */
        // The goal of this is to autmatically create an array of AddressableLEDBufferViews:
        //      one for each segment of an LED strip.
        // We just need to supply the lengths of each segment.
        //
        // The reasons for doing it this way:
        //   - There is no need to calculate the starting and ending indexes initially or each time there is a change.
        //   - Avoids errors with the calculations.
        //   - Provides an indexable array of buffer views for easy referencing. One just needs to supply the index.
        //   - Allow for meaningful names for the indexes.

        int totalBufferLength = 0; 
        // Create an array of buffer views - one view for each section of the strip.
    
        // Create the buffer
        AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(100);
        AddressableLEDBufferView [] ledStripSegs = new AddressableLEDBufferView[ledSegLens.length];
        // new AddressableLEDBufferView(ledSegLens.length);
    
        int inxNext = 0; 
        // This contains the starting index for the next view to be created

 
    public Lightbar() {
        // Add up the lengths
        for (int len : ledSegLens){
            totalBufferLength += len;
        }

        // Create a view corresponding to each of the elements in ledSegLens
        // The parameters to createView are the start-index and the end-index.
        // The end-index is one less than the start-index plus the length.
        // Note that the "inxNext+=ledSegLens[seg]" updates inxNext to start-inx of the next view
        for (int seg =0; seg<ledSegLens.length; seg++){
            ledStripSegs[seg] = ledStrip.createView(inxNext, ((inxNext+=ledSegLens[seg])-1));
            }
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDPWMport);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(ledStrip.getLength());
    m_led.setColorOrder(ColorOrder.kBRG);
    // m_led.setColorOrder
    // setRBG(5, 100, 0, 0);
    // setRBG(6, 0, 100, 0);
    // setRBG(7, 0, 0, 100);

    // Set the data
    m_led.setData(ledStrip);
    m_led.start();
  
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  // public void setRBG( int index, int r, int b, int g) {
  //   m_ledBuffer.setRGB(index, r, b, g);
  // }

  public void setRBG( int first, int last, int r, int b, int g) {
    int index; 
    for (index = first; index <= last; index++){
      ledStrip.setRGB(index, r, b, g);
    }
  }

  public void clearLEDs() {
      for (int index = 0; index < ledStrip.getLength(); index++) {
        ledStrip.setRGB(index, 0, 0, 0);
      }
    }
  
  

    public void SetSegmentByValue(int seg, double xSpeed, double triggerLevelLow, double triggerLevelHigh, LEDPattern patternLow, LEDPattern patternMid, LEDPattern patternHigh, double brightness) {
      if (xSpeed > triggerLevelHigh) {
        LEDPattern NewPattern = patternHigh.atBrightness(Units.Percent.of(brightness));
        // Apply the LED pattern to the data buffer
        NewPattern.applyTo(ledStripSegs[seg]);
      } 
      else if (xSpeed < triggerLevelLow) { 
        LEDPattern NewPattern = patternLow.atBrightness(Units.Percent.of(brightness));
        NewPattern.applyTo(ledStripSegs[seg]);
      }
      else {
        LEDPattern NewPattern = patternMid.atBrightness(Units.Percent.of(brightness));
        NewPattern.applyTo(ledStripSegs[seg]);
      } 

    }

  @Override
  public void periodic() {
    // // Update the buffer with the rainbow animation
    // m_scrollingRainbow.applyTo(m_ledBuffer);
    // // Set the LEDs
    m_led.setData(ledStrip);
    // This method will be called once per scheduler run
  }
}
