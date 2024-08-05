within IndustrialControlSystems.LinearSystems.Discrete;
model Integrator "Integrator: mu/s"
  extends Interfaces.DiscreteBaseBlock;
  parameter Real mu = 1 "Gain"
    annotation(Dialog(group = "Block parameters"));
  parameter Real y_start = 0 "Output initial value"
    annotation(Dialog(group = "Initial conditions"));
initial equation
  pre(y) = y_start;
equation
  when sample(0,Ts) then
    y = Functions.fIntegrator(alfa,u,pre(u),pre(y),Ts,mu);
  end when;
  annotation (Icon(graphics={
        Text(
          extent={{-30,40},{36,6}},
          lineColor={0,0,0},
          fillColor={213,255,170},
          fillPattern=FillPattern.Solid,
          textString="mu"),
        Text(
          extent={{-30,-4},{36,-38}},
          lineColor={0,0,0},
          fillColor={213,255,170},
          fillPattern=FillPattern.Solid,
          textString="s"),
        Line(
          points={{-44,0},{50,0}},
          color={0,0,0},
          smooth=Smooth.None)}),Documentation(info="
  <HTML>
  <h4>Description</h4>
  <p>
  Discretised version of the continuous time transfer function of an integrating process.
  <pre>
   Y(s)      mu
   ----  = ------
   U(s)      s
  </pre>
  </p>
  <p>
  <h4>Discretisation</h4>
  The discretisation of the continuos time transfer function has been performed with the bilinear transformation formula
  <pre>
                 z - 1 
  s = ------------------------------
        z*alpha*Ts - (alpha - 1)*Ts
  </pre>
  that is equivalent to<br>
  <ul>
  <li>Forward Euler (FE) if <pre><FONT FACE=Courier>alpha = 0</FONT></pre></li>
  <li>Backward Euler (BE) if <pre><FONT FACE=Courier>alpha = 1</FONT></pre></li>
  <li>Tustin (TU) if <pre><FONT FACE=Courier>alpha = 0.5</FONT></pre></li>
  </ul>
  </p>
  </HTML>", revisions="<html>
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
end Integrator;
