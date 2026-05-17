package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.DigitalChannelValueParameters;

@TeleOp
public class magnetic_limit_sensor {

    private DigitalChannelValueParameters magnetic_limit_sensor;

  public void init(@NonNull HardwareMap hwMap) {
      magnetic_limit_sensor = hwMap.get(DigitalChannel.class, "magnet_limit_sensor");
      magnetic_limit_sensor. setMode(DigitalChannel.Mode.INPUT);
}

public boolean getmagneticlimitsensor(){
      return !LimitMagnet.getState();

}

