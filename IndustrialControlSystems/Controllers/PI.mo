within IndustrialControlSystems.Controllers;
model PI "Proportional + Integral controller"
  extends Interfaces.Controller;
  parameter Real Kp = 5 "Proportional gain"
    annotation(Evaluate = true, Dialog(group = "Parameters"));
  parameter Real Ti = 1 "Integral time"
    annotation(Evaluate = true, Dialog(group = "Parameters"));
  Real Ti_act "Actual integral time";
  Real Kp_act "Actual proportianal gain";
  Real FBout "output (and state) of the feedback block 1/(1+sTi)";
  Real cs "Pre-biased Control Signal";
  Real satin "input of the saturation block";
  Modelica.Blocks.Interfaces.RealInput Kp_GS if useGS annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-92}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-20,-80})));
  Modelica.Blocks.Interfaces.RealInput Ti_GS if useGS annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-92}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={42,-80})));
protected
  Modelica.Blocks.Interfaces.RealInput Ti_in "Internal protected integral time real input";
  Modelica.Blocks.Interfaces.RealInput Kp_in "Internal protected Kp gain real input";
initial equation

  if Ts > 0 then
    // Discrete time initialisation
    pre(FBout) = if AntiWindup then max(CSmin,min(CSmax,CS_start)) else CS_start - bias - Kp*(SP - PV);
  else
    // Continuous time initialisation
    FBout = if AntiWindup then max(CSmin,min(CSmax,CS_start)) else CS_start - bias - Kp*(SP - PV);
  end if;

equation

  connect(Kp_in,Kp_GS);
  connect(Ti_in,Ti_GS);
  if not useGS then
    Kp_in = Kp;
    Ti_in = Ti;
  end if;
  Kp_act = Kp_in;
  Ti_act = Ti_in;

  if Ts > 0 then
    // Discrete time controller
    when sample(0,Ts) then
      satin =  pre(FBout) + Kp*(SP-PV);
      cs    =  if ts then tr else (if AntiWindup then max(CSmin,min(CSmax,satin)) else satin);
      FBout = IndustrialControlSystems.LinearSystems.Discrete.Functions.f1Pole(
        alfa,
        cs,
        pre(cs),
        pre(FBout),
        Ts,
        1,
        if not ts then Ti else eps);
      CS    =  if AntiWindup then max(CSmin,min(CSmax,cs + (if ts then 0 else pre(bias)))) else cs + (if ts then 0 else pre(bias));
    end when;
  else
    // Continuous time controller
    satin = if ts then tr else FBout + Kp_act*(SP-PV);
    //cs    = if AntiWindup then max(CSmin,min(CSmax,satin)) + bias else satin + bias;
    cs    = if AntiWindup then max(CSmin,min(CSmax,satin)) else satin;
    FBout = cs + (if not ts then -Ti_act*der(FBout) else -eps*der(FBout));
    CS    = if AntiWindup then max(CSmin,min(CSmax,cs + bias)) else cs + bias;
  end if;

  annotation (Icon(graphics={Text(
          extent={{-34,30},{46,-40}},
          lineColor={0,0,255},
          textString="PI")}),Documentation(info="
  <HTML>
  <h4>Description</h4>
  <p>
  Proportional + Integral controller with Automatic, Tracking mode and bias signal.<br/>
  The control law is defined as
  <pre>
              1+sTi 
  CS(s) = Kp ------- (SP(s) - PV(S))
               sTi
  </pre>
  <br/><br/>
  <b>Scheme</b><br/><br/>
  <img src=\"modelica://IndustrialControlSystems/help/images/Controllers/PropInt.png\">
  <br/><br/>
  <TABLE BORDER=1 CELLSPACING=0 CELLPADDING=2 >
  <TR bgcolor=#e0e0e0><TH >Name</TH><TH>Description</TH><TH>Conditional?</TH></TR>
  <tr>  <td>SP</td><td>Set Point</td><td>NO</td></tr>
  <tr>  <td>PV</td><td>Process Variable</td><td>NO</td></tr>
  <tr>  <td>CS</td><td>Control Signal</td><td>NO</td></tr>
  <tr>  <td>TR</td><td>Track Reference signal</td><td>YES (useTS)</td></tr>
  <tr>  <td>TS</td><td>Track Switch signal</td><td>YES (useTS)</td></tr>
  <tr>  <td>Bias</td><td>Biasing signal</td><td>YES (useBIAS)</td></tr>
  <tr>  <td>ATreq</td><td>AutoTuning request</td><td>YES (*)</td></tr>
  </table>
  (*) AutoTuning Not available here, please see the package<br>
  <a href=\"Modelica://IndustrialControlSystems.Controllers.AutoTuning\">AutoTuning</a>
  <br/><br/>
  The controller can have various operating conditions:
  <ul>
  <li>Automatic,</li>
  <li>and Tracking</li>
  </ul><br/>
  
  In the Automatic mode, the control output is computed with the proportional+integral control law, while in the tracking mode<br>
  it is defined by the input TR.<br/><br/>
  To note that in the tracking mode the output signal CS is equal to TR, and the first order filter 
  <pre>
     1
  -------- TR(S)
   1+s*Ti
  </pre>
  is replaced by
  <pre>
     1
  -------- TR(S)
   1+s*eps
  </pre>
  where eps is a small time constant. With such a scheme, the integrator does not diverge while the tracking mode<br>
  is enabled.
  </p>
  </HTML>",
          revisions="<html>
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
</dl></html>"));
end PI;
