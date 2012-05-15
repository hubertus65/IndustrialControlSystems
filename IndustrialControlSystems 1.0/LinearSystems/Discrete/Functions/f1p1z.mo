within IndustrialControlSystems.LinearSystems.Discrete.Functions;
function f1p1z "Function lead lag process: k(1+sT1)/(1+sT2)"
  input Real alfa "Parametro di discretizzazione";
  input Real T1 "Zero";
  input Real T2 "Polo";
  input Real u "Current input";
  input Real u_pre "Previous input";
  input Real y_pre "Previous output";
  input Real Ts "Sampling time [s]";
  input Real k "Gain";
  output Real y "Output";
protected
  Real A;
  Real B;
  Real C;
  Real D;
  Real E;
  Real F;
algorithm
  A := alfa*Ts;
  B := Ts - alfa*Ts;
  C := A + T1;
  D := B - T1;
  E := A + T2;
  F := B - T2;
  y := (C*k*u + D*k*u_pre - F*y_pre)/E;
  annotation (Documentation(revisions="<html>
<dl><dt>Industrial Control Systems (v 1.0.0) : April-May 2012</dt>
<dl><dt>List of revisions:</dt>
<p><ul>
<li>11 May 2012 (author: Marco Bonvini)</li>
</ul></p>
<dl><dt><b>Main Authors:</b> <br/></dt>
<dd>Marco Bonvini; &LT;<a href=\"mailto:bonvini@elet.polimi.it\">bonvini@elet.polimi.it</a>&GT;</dd>
<dd>Alberto Leva &LT;<a href=\"mailto:leva@elet.polimi.it\">leva@elet.polimi.it</a>&GT;<br/></dd>
<dd>Politecnico di Milano</dd>
<dd>Dipartimento di Elettronica e Informazione</dd>
<dd>Via Ponzio 34/5</dd>
<dd>20133 Milano - ITALIA -<br/></dd>
<dt><b>Copyright:</b> </dt>
<dd>Copyright &copy; 2010-2012, Marco Bonvini and Alberto Leva.<br/></dd>
<dd><i>The IndustrialControlSystems package is <b>free</b> software; it can be redistributed and/or modified under the terms of the <b>Modelica license</b>.</i><br/></dd>
</dl></html>"));
end f1p1z;
