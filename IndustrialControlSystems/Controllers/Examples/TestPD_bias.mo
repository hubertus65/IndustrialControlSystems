within IndustrialControlSystems.Controllers.Examples;
model TestPD_bias "Test of the Proportional + Derivative controller with bias"
  extends Modelica.Icons.Example;
  parameter Real Ts = 0
    "if Ts>=0 then discrete time controller, otherwise continuous time"
    annotation(Dialog(group = "Sampling time"));
  Modelica.Blocks.Sources.Step SPstep(startTime=10)
                                    annotation (
      Placement(transformation(extent={{-100,70},{-80,90}},rotation=0)));
  PD                           R(
    AntiWindup=true,
    CSmin=0,
    CS_start=0,
    Ts=Ts,
    CSmax=2,
    Td=1,
    N=5,
    Kp=5)
    annotation (Placement(transformation(extent={{-50,40},{-30,60}})));
  Modelica.Blocks.Math.Feedback sub2
                                    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-16,50})));
  PD                           R_Bias(
    useBIAS=true,
    AntiWindup=true,
    CSmin=0,
    CS_start=0,
    Ts=Ts,
    CSmax=2,
    Td=1,
    N=5,
    Kp=5)
    annotation (Placement(transformation(extent={{-52,-40},{-32,-20}})));
  Modelica.Blocks.Math.Feedback sub3
    annotation (Placement(transformation(extent={{-24,-40},{-4,-20}})));
  Modelica.Blocks.Sources.TimeTable
                               Disturbance(
    offset=0,
    startTime=0,
    table=[0,0; 50,0; 50,0.5; 52,0.5; 52,0; 100,0])
                  annotation (                          Placement(
        transformation(extent={{-100,-20},{-80,0}}, rotation=0)));
  LinearSystems.Continuous.TransferFunction ProcessCtrl(num={12,1}, den={20,12,1,
        0})
    annotation (Placement(transformation(extent={{20,38},{44,62}})));
  LinearSystems.Continuous.TransferFunction processNoCtrl(num={12,1}, den={20,12,
        1,0})
            annotation (Placement(transformation(extent={{20,68},{44,92}})));
  LinearSystems.Continuous.TransferFunction ProcessBias(num={12,1}, den={20,12,1,
        0})
    annotation (Placement(transformation(extent={{20,-42},{44,-18}})));
  Modelica.Blocks.Math.Feedback sub1
                                    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,80})));
equation
  connect(SPstep.y, R.SP)
                         annotation (Line(
      points={{-79,80},{-64,80},{-64,56},{-48,56}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(R.CS, sub2.u1) annotation (Line(
      points={{-31,50},{-24,50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(Disturbance.y, sub2.u2)
                                 annotation (Line(
      points={{-79,-10},{-16,-10},{-16,42}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(SPstep.y, R_Bias.SP)
                             annotation (Line(
      points={{-79,80},{-64,80},{-64,-24},{-50,-24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(R_Bias.CS, sub3.u1)     annotation (Line(
      points={{-33,-30},{-22,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(Disturbance.y, sub3.u2)     annotation (Line(
      points={{-79,-10},{-72,-10},{-72,-50},{-14,-50},{-14,-38}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(Disturbance.y, R_Bias.BIAS) annotation (Line(
      points={{-79,-10},{-72,-10},{-72,-2},{-46,-2},{-46,-22}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sub2.y, ProcessCtrl.u) annotation (Line(
      points={{-7,50},{22.4,50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(ProcessCtrl.y, R.PV) annotation (Line(
      points={{42.8,50},{56,50},{56,20},{-60,20},{-60,49},{-48,49}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sub3.y, ProcessBias.u) annotation (Line(
      points={{-5,-30},{22.4,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(ProcessBias.y, R_Bias.PV) annotation (Line(
      points={{42.8,-30},{56,-30},{56,-60},{-60,-60},{-60,-31},{-50,-31}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sub1.u1, SPstep.y) annotation (Line(
      points={{-8,80},{-79,80}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sub1.y, processNoCtrl.u) annotation (Line(
      points={{9,80},{22.4,80}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(Disturbance.y, sub1.u2) annotation (Line(
      points={{-79,-10},{6.66134e-16,-10},{6.66134e-16,72}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(graphics),
    experiment(StopTime=100),
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
  In this example have been tested the proportional + Derivative controller.<br>
  The process to be controlled has the following transfer function
  <pre>
   Y(s)          (1+15s)
   ----  =  ----------------
   U(s)       s(1+10s)(1+2s)
  </pre>
  There are three processes:<br>
  <ul>
  <li>without controller,</li> 
  <li>with a PD controller,</li> 
  <li>and a PD controller with bias signal</li> 
  </ul><br>
  <img src=\"modelica://IndustrialControlSystems/help/images/Controllers/Examples/PD_bias.png\"><br><br>
  The output signal of the process without control diverges (due to the presence of an integrator) and is not reported.<br>
  In the closed loop system, the proportional + derivative controller cannot reach the SP reference, but it remains in a neighborhood of it.
  In this case theparameter Kp can reduce the amplitude of such a neighborhood. The disturb (red signal) can be rejected using the bias signal 
  of the controller (pink line), otherwise the action does not reach a satisfactory performance (the Process Variable move away from the Set Point reference).
  <br>
  <h4>Discrete time</h4>
  If the model parameter <FONT FACE=Courier>Ts</FONT> is <FONT FACE=Courier>>=0</FONT> the continuous time controllers are 
  replaced by their discrete time versions.<br>
  The effect of various discretisation method can be studied.
  </HTML>"));
end TestPD_bias;
