package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDSubsystem extends SubsystemBase{
    DigitalInput limitSwitch = new DigitalInput(0);

    private final CANdle m_candle = new CANdle(LEDConstants.CANdleID);
    private final int ledCount = LEDConstants.LEDcount;

    private double brightness;
    private double ledAnimSpeed;
    private int ledOffset;
    private boolean ledReversed;
    private Color ledColor = new Color(0, 0, 0);
    private int ledWhite;

    private int tempVarDeleteLaterPlease = 0;
    
    private Animation currentAnimation = null;

    public enum animations {
        SET_ALL,
        FIRE_ANIM,
        LARSON_ANIM,
        RAINBOW_ANIM,
        RGB_FADE_ANIM, 
        SINGLE_FADE_ANIM,
        STROBE_ANIM,
        COLOR_FLOW_ANIM,
        TWINKLE_OFF_ANIM,
        TWINKLE_ANIM
    }

    public LEDSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        configAll.v5Enabled = true;
        m_candle.configAllSettings(configAll, 100);

        // setLEDs(0, 0, 100);
        // m_candle.animate(new RainbowAnimation(1, 0.5, 16, false, 0));
    }

    public boolean getLimit() {
        return limitSwitch.get();
    }

    public int getR(Color color) {
        return (int)(color.red*255);
    }

    public int getG(Color color) {
        return (int)(color.green*255);
    }

    public int getB(Color color) {
        return (int)(color.blue*255);
    }

    /**
     * Sets the color of the LEDs
     */
    public LEDSubsystem setColor(Color color) {
        ledColor = color;
        return this;
    }

    /**
     * Sets the white Offset if applicable for the animation
     * 
     * @param white - [0, 255]
     */
    public LEDSubsystem setWhite(int white) {
        ledWhite = white;
        return this;
    }

    public LEDSubsystem setBrightness(double brightness) {
        this.brightness = brightness;
        return this;
    }

    public LEDSubsystem setAnimSpeed(double animSpeed) {
        ledAnimSpeed = animSpeed;
        return this;
    }

    public LEDSubsystem setOffset(int offset) {
        ledOffset = offset;
        return this;
    }

    public void setAnimation(animations animation) {
        switch(animation) {
            case SET_ALL:
                clear();
                currentAnimation = null;
                break;

            case FIRE_ANIM:
                clear();
                currentAnimation = new FireAnimation(brightness, ledAnimSpeed, ledCount, 1, 1, ledReversed, ledOffset);
                break;

            case LARSON_ANIM:
                clear();
                currentAnimation = new LarsonAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset);
                break;

            case RAINBOW_ANIM:
                clear();
                currentAnimation = new RainbowAnimation(brightness, ledAnimSpeed, ledCount, ledReversed, ledOffset);
                break;

            case RGB_FADE_ANIM: 
                clear();
                currentAnimation = new RgbFadeAnimation(brightness, ledAnimSpeed, ledCount, ledOffset);
                break;

            case SINGLE_FADE_ANIM:
                clear();
                currentAnimation = new SingleFadeAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, ledOffset);
                break;

            case STROBE_ANIM:
                clear();
                currentAnimation = new StrobeAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, ledOffset);
                break;

            case COLOR_FLOW_ANIM:
                clear();
                currentAnimation = new ColorFlowAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset);
                break;

            case TWINKLE_OFF_ANIM:
                clear();
                currentAnimation = new TwinkleOffAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset);
                break;

            case TWINKLE_ANIM:
                clear();
                currentAnimation = new TwinkleAnimation(getR(ledColor), getG(ledColor), getB(ledColor), ledWhite, ledAnimSpeed, ledCount, null, ledOffset);
                break;
        }
    }

    public void clear() {
        m_candle.clearAnimation(0);
    }

    @Override
    public void periodic() {

        
        if (getLimit()) {
            tempVarDeleteLaterPlease++;
            m_candle.setLEDs(255, 0, 0);
            clear();
        }
        else {
            m_candle.setLEDs(getR(ledColor), getG(ledColor), getB(ledColor));
            m_candle.animate(currentAnimation);
        }
        
        SmartDashboard.putNumber("LED_R", getR(ledColor));
        SmartDashboard.putNumber("LED_G", getG(ledColor));
        SmartDashboard.putNumber("LED_B", getB(ledColor));
        SmartDashboard.updateValues();
    }
}
