within IndustrialControlSystems.Controllers.Examples;
model TestP_tracking "Test of the Proportional controller -- Tracking mode"
  extends Modelica.Icons.Example;
  parameter Real Ts = 0
    "if Ts>=0 then discrete time controller, otherwise continuous time"
    annotation(Dialog(group = "Sampling time"));
  Modelica.Blocks.Sources.Step step(startTime=10)
                                    annotation (
      Placement(transformation(extent={{-94,70},{-74,90}}, rotation=0)));
  IndustrialControlSystems.Controllers.P R(
    AntiWindup=true,
    CSmin=0,
    CSmax=2,
    Ts=Ts,
    Kp=20)
    annotation (Placement(transformation(extent={{-50,40},{-30,60}})));
  IndustrialControlSystems.Controllers.P R_track(
    useTS=true,
    useBIAS=false,
    AntiWindup=true,
    CSmin=0,
    CSmax=2,
    Ts=Ts,
    Kp=20)
    annotation (Placement(transformation(extent={{-52,-40},{-32,-20}})));
  Modelica.Blocks.Sources.BooleanTable TSsignal(startValue=false, table={40,85})
               annotation (                         Placement(
        transformation(extent={{-100,0},{-80,20}}, rotation=0)));
  Modelica.Blocks.Sources.TimeTable TrackRef(
    offset=0,
    startTime=0,
    table=[0,0; 40,0; 40,0.94; 60,0.6; 100,0.6])
                 annotation (                            Placement(
        transformation(extent={{-100,-32},{-80,-12}}, rotation=0)));
  LinearSystems.Continuous.TransferFunction process(num={12,1}, den={20,12,1})
            annotation (Placement(transformation(extent={{0,38},{24,62}})));
  LinearSystems.Continuous.TransferFunction processTrack(num={12,1}, den={20,12,
        1}) annotation (Placement(transformation(extent={{0,-42},{24,-18}})));
equation
  connect(step.y,R. SP)  annotation (Line(
      points={{-73,80},{-60,80},{-60,56},{-48,56}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(step.y, R_track.SP)
                           annotation (Line(
      points={{-73,80},{-68,80},{-68,-24},{-50,-24}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(TSsignal.y, R_track.TS)
                               annotation (Line(
      points={{-79,10},{-38,10},{-38,-22}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(TrackRef.y, R_track.TR)
                               annotation (Line(
      points={{-79,-22},{-72,-22},{-72,-10},{-42,-10},{-42,-22}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(R.CS, process.u) annotation (Line(
      points={{-31,50},{2.4,50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(process.y, R.PV) annotation (Line(
      points={{22.8,50},{34,50},{34,22},{-60,22},{-60,49},{-48,49}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(R_track.CS, processTrack.u) annotation (Line(
      points={{-33,-30},{2.4,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(processTrack.y, R_track.PV) annotation (Line(
      points={{22.8,-30},{34,-30},{34,-58},{-60,-58},{-60,-31},{-50,-31}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(graphics),
    experiment(StopTime=120),
    experimentSetupOutput,
    Documentation(revisions="<html>
<dl><dt>Industrial Control Systems (v 1.0.0) : April-May 2012</dt>
<dl><dt>List of revisions:</dt>
<p><ul>
<li>11 May 2012 (author: Marco Bonvini)</li>
</ul></p>
<dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Marco Bonvini; &lt;<a href=\"mailto:bonvini@elet.polimi.it\">bonvini@elet.polimi.it</a>&gt;</dd>
<dd>Alberto Leva &lt;<a href=\"mailto:leva@elet.polimi.it\">leva@elet.polimi.it</a>&gt;<br/></dd>
<dd>Politecnico di Milano</dd>
<dd>Dipartimento di Elettronica e Informazione</dd>
<dd>Via Ponzio 34/5</dd>
<dd>20133 Milano - ITALIA -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2010-2012, Marco Bonvini and Alberto Leva.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>", info="<HTML>
  <h4>Description</h4>
  <p>
  In this example have been tested the tracking mode of the proportional controller.<br>
  The process to be controlled has the following transfer function
  <pre>
   Y(s)          (1+15s)
   ----  =  ----------------
   U(s)       (1+10s)(1+2s)
  </pre>
  There are two processes:<br>
  <ul>
  <li>with a P controller,</li> 
  <li>and a P controller with tracking mode</li> 
  </ul><br>
  The output signal of the process controlled without tracking is the red line, while the green line is the 
  output of the process controlled with the tracking mode.<br>
  <img src=\"modelica://IndustrialControlSystems/help/images/Controllers/Examples/P_track_PV.png\"><br><br>
  The CS of the controller becomes equal to the track reference signal TR when the Track Switch signal becomes true.
  <br>
  <img src=\"modelica://IndustrialControlSystems/help/images/Controllers/Examples/P_track_CS.png\"><br><br>
  <br>
  <h4>Discrete time</h4>
  If the model parameter <FONT FACE=Courier>Ts</FONT> is <FONT FACE=Courier>>=0</FONT> the continuous time controllers are 
  replaced by their discrete time versions.<br>
  The effect of various discretisation method can be studied.
  </HTML>"));
end TestP_tracking;
