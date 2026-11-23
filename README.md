<a id="TMP_93ba"></a>
<a id="TMP_62fe"></a>

# Pendulum cart control

The system consists of a cart of mass $M$ carrying a pendulum of mass $m$ through a mass\-less rod with a length $l$ . The cart displacement is driven by a horizontal force u.  

![image_0.png](./README_media/image_0.png)

The goal of the system is to stabilize the pendulum upright, keeping the swing angle q2 = 0 by controlling the displacement q1 of the car. 

<!-- Begin Toc -->

## Table of Contents
&#8195;&#8195;[1. Model of the system](#TMP_1494)
 
&#8195;&#8195;[2. Model linearization](#TMP_5155)
 
&#8195;&#8195;[3. Simulink Model of the system](#TMP_0400)
 
&#8195;&#8195;[4. System analysis](#TMP_9f18)
 
&#8195;&#8195;&#8195;[Stability](#TMP_8702)
 
&#8195;&#8195;&#8195;[Controllability and observability](#TMP_560b)
 
&#8195;&#8195;[5.Control](#TMP_21f9)
 
&#8195;&#8195;&#8195;[Loop shaping with PID controller](#TMP_6de2)
 
&#8195;&#8195;&#8195;[State feedback control](#TMP_0e02)
 
&#8195;&#8195;&#8195;[State feedback controller with cart position (q\_1) as output](#TMP_1364)
 
&#8195;&#8195;&#8195;[State feedback controller with swing angle (q\_2) as output](#TMP_3e49)
 
&#8195;&#8195;&#8195;[LQR state feedback control](#TMP_24bf)
 
&#8195;&#8195;&#8195;[Observer based state feedback control](#TMP_8889)
 
&#8195;&#8195;&#8195;[LQR State feedback with integrale action](#TMP_5877)
 
&#8195;&#8195;&#8195;[Sliding mode control](#TMP_1f8f)
 
&#8195;&#8195;&#8195;[RST control](#TMP_944b)
 
<!-- End Toc -->

<a id="TMP_1494"></a>

## 1. Model of the system

The system has 2 degrees of freedom q1 and q2. 

Considering the (x,y) frame the position of the masses are given as 

 $$ \left\lbrace \begin{array}{ll} \vec{\mathrm{OM}\;}  & =q_1 \vec{\;x} \newline \vec{\;\mathrm{Om}}  & =\left(q_1 -l\;\sin \left(q_2 \right)\right)\;\vec{x} +l\;\cos \left(q_2 \right)\;\vec{\;y}  \end{array}\right. $$ 

The velocities are:

 $$ \left\lbrace \begin{array}{ll} \frac{d\vec{\;\mathrm{OM}} }{\mathrm{dt}} & =\dot{\;q_1 } \;\vec{\;x} \newline \frac{d\;\vec{\;\mathrm{Om}} }{\mathrm{dt}} & =\left(\dot{\;q_1 } +l\dot{\;q_2 } \;\cos \left(q_2 \right)\right)\vec{\;x} -l{\dot{\;q} }_2 \sin \left(q_2 \right)\;\vec{\;y}  \end{array}\right. $$ 

The kinetic energy of the system is given as 

 $$ \begin{array}{l} T=\frac{1}{2}M\times v_M^2 +\frac{\;1}{2}m\times v_{m\;}^2 =\frac{\;1}{2}M{\dot{\;q_1 } }^2 +\frac{1}{2}{m\left(\left(\dot{\;q_1 } +l\dot{\;q_2 } \;\cos \left(q_2 \right)\right)\vec{\;x} -l{\dot{\;q} }_2 \sin \left(q_2 \right)\;\vec{\;y} \right)}^2 \newline T=\frac{1}{2}\left(M+m\right){\dot{\;q_1 } }^2 +\frac{1}{2}m\left({l^2 \dot{\;q_2 } }^2 -2l\dot{\;q_1 } \dot{\;q_2 } \cos \left(q_2 \right)\right) \end{array} $$ 

The potential energy of the system is produce only by the mass m as the mass M is already grounded. So we have

 $$ V=\mathrm{m}\mathrm{g}\mathrm{L}\mathrm{c}\mathrm{o}\mathrm{s}\left(q_2 \right)\; $$ 

From the about energies equation, we obtain the Lagrangian

 $$ L=T-V=\frac{1}{2}\left(M+m\right){\dot{\;q_1 } }^2 +\frac{1}{2}m\left({l^2 \dot{\;q_2 } }^2 -2l\dot{\;q_1 } \dot{\;q_2 } \cos \left(q_2 \right)\right)-\mathrm{mgLcos}\left(q_2 \right)\;\; $$ 

As the system is not only conservative. It also have dissipation force (friction of the contact between wheel and ground), generalized Lagrange equations apply as follows: 

 $$ \frac{d}{\mathrm{dt}}\left(\frac{\partial \;L}{\partial \;\dot{\;q_i } }\right)-\frac{\partial \;L}{\partial \;q_i }=Q_i $$ 

Rayleigh dissipation function will be used to introduce the friction force proportional to velocity. 

 $$ R\left(\dot{\;q} \right)=\frac{1}{2}\sum_i^N k_i {\;\dot{\;q_i } }^2 =\frac{1}{2}\;\left(k_1 {\dot{\;q_1 } }^2 +k_2 \;{\dot{\;q_2 } }^2 \right) $$ 

the dissipation force is given as $F_i =-\frac{\partial R}{\partial \dot{\;q_i } }\;\;\;;F_1 =-\frac{\partial R}{\partial \dot{\;q_1 } }=-\frac{1}{2}*\left({2*k}_1 \;\dot{\;q_1 } \right)=-k_1 \;\dot{q_1 }$ 

Thus $Q_i \;=F_i +<\mathrm{other}\;\mathrm{external}\;\mathrm{forces}\;\mathrm{applied}\;\mathrm{to}\;\mathrm{particle}\;i>$ 

- Lagrange equation w.r.t. $q_1$

 $$ \frac{d}{\mathrm{dt}}\left(\frac{\partial \;L}{\partial \;\dot{\;q_1 } }\right)-\frac{\partial \;L}{\partial \;q_1 }=-k_1 \;\dot{\;q_1 } +u $$ 

 $\frac{\partial \;L}{\partial \;\dot{\;q_1 } }=\left(M+m\right)\dot{\;q_1 } -m\;l\;\dot{q_2 } \;\cos \left(q_2 \right)\;$;   

 $$ \frac{d}{\mathrm{dt}}\left(\frac{\partial \;L}{\partial \;\dot{\;q_1 } }\right)=\left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;\cos \left(q_2 \right)+m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right) $$ 

 $$ \frac{\partial \;L}{\partial \;q_1 }=0 $$ 

 $$ \Rightarrow \frac{d}{\mathrm{dt}}\left(\frac{\partial \;L}{\partial \;\dot{\;q_1 } }\right)-\frac{\partial \;L}{\partial \;q_1 }=-k\;\dot{\;q_1 } \;\;\Rightarrow \left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;\cos \left(q_2 \right)+m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right)+k_1 \;\dot{\;q_1 } =u $$ 

- Lagrange equation w.r.t. $q_2$

 $$ \;\Rightarrow -m\;l\;\ddot{\;q_1 } \;\cos \left(q_2 \right)+m\;l^{2\;} \ddot{\;q_2 } -m\;g\;l\;\sin \left(q_2 \right)=-k_2 \;\dot{\;q_2 } \; $$ 

The model of the system is given as 

 $$ \left\lbrace \begin{array}{ll} \;\left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;\cos \left(q_2 \right)+m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right)+k_1 \;\dot{\;q_1 }  & =u\newline -m\;l\;\ddot{\;q_1 } \;\cos \left(q_2 \right)+m\;l^{2\;} \ddot{\;q_2 } -m\;g\;l\;\sin \left(q_2 \right)+k_2 \;\dot{\;q_2 } \; & =0 \end{array}\right. $$ 

<a id="TMP_5155"></a>

## 2. Model linearization

 

 $$ \begin{array}{l} q_2 \approx 0\approx \epsilon \;\newline \dot{\;q_2 \;} \approx \epsilon \;\; \end{array} \Rightarrow {\dot{q_2 } }^2 =\epsilon^2 \;\;;{\dot{q_2 } }^2 q_2 =\epsilon^3 =0 $$ 

 $$ q_2 \approx 0\;\Rightarrow \left\lbrace \begin{array}{ll} \cos \left(q_2 \right) & \approx 1\newline \sin \left(q_2 \right) & \approx q_2  \end{array}\right. $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{ll} \;\left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;+k_1 \;\dot{\;q_1 }  & =u\newline -m\;l\;\ddot{\;q_1 } \;+m\;l^{2\;} \ddot{\;q_2 } -m\;g\;l\;q_2 +k_2 \;\dot{\;q_2 } \; & =0 \end{array}\right. $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{cc} \ddot{\;q_1 }  & =\frac{\;m\;l\;}{M+m}\ddot{\;q_2 } -\frac{k_1 }{M+m}\;\dot{\;q_1 } \;+\frac{1}{M+m}u\;\;\;\;\;\;\;\;\;\;\;\newline \ddot{\;q_2 }  & \;=\frac{\;1}{l}\ddot{\;q_1 } -\frac{\;k_2 }{m\;l^2 }\dot{\;q_2 } +\frac{\;g}{l}q_2 \;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\; \end{array}\right. $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{cc} \ddot{\;q_1 } = & \frac{\;m\;}{M+m}\ddot{\;q_1 } \;-\frac{k_2 }{\left(M+m\right)l}\dot{\;q_2 } \;+\frac{\;m\;g}{\left(M+m\right)}q_2 \;-\frac{k_1 }{M+m}\dot{\;q_1 } +\frac{1}{M+m}u\newline \ddot{\;q_2 } = & \frac{\;m}{M+m}\ddot{\;q_2 } \;-\frac{\;k_1 }{\left(M+m\right)l}\dot{\;q_1 } -\frac{\;k_2 }{m\;l^2 }\dot{\;q_2 } +\frac{\;g}{l}q_2 +\frac{\;1}{\left(M+m\right)l\;}\;u\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\; \end{array}\right. $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{cc} M\ddot{\;q_1 } = & -k_1 \dot{\;q_1 } -\frac{k_2 }{l}\dot{\;q_2 } \;+m\;g\;q_2 \;+u\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\newline M\ddot{\;q_2 } = & \;-\frac{\;k_1 }{l}\dot{\;q_1 } -\frac{\left(M+m\right)\;k_2 }{m\;l^2 }\dot{\;q_2 } +\frac{\;\left(M+m\right)g}{l}q_2 +\frac{1}{l}u\; \end{array}\right. $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{cc} \ddot{\;q_1 } = & -\frac{k_1 }{M}\dot{\;q_1 } -\frac{k_2 }{M\;l\;}\dot{\;q_2 } \;+\frac{\;m\;g}{M\;}q_2 \;+\frac{\;1}{M}u\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\newline \ddot{\;q_2 } = & \;-\frac{\;k_1 }{M\;l}\dot{\;q_1 } -\frac{\left(M+m\right)\;k_2 }{M\;m\;l^2 }\dot{\;q_2 } +\frac{\;\left(M+m\right)g}{M\;l}q_2 +\frac{1}{M\;l}u\;\; \end{array}\right. $$ 

 $$ \; $$ 

 $$ \Rightarrow \left\lbrace \begin{array}{ll} \dot{\;q_1 }  & =\dot{\;q_1 } \newline \dot{\;q_2 }  & =\dot{\;q_2 } \newline \ddot{\;q_1 }  & =-\frac{k_1 }{M}\dot{\;q_1 } -\frac{k_2 }{M\;l}\dot{\;q_2 } \;+\frac{\;m\;g}{M\;l}q_2 \;+\frac{\;1}{M\;}u\newline \ddot{\;q_2 }  & =\;-\frac{\;k_1 }{M\;l}\dot{\;q_1 } -\frac{\left(M+m\right)\;k_2 }{M\;m\;l^2 }\dot{\;q_2 } +\frac{\;\left(M+m\right)g}{M\;l}q_2 +\frac{1}{M\;l}u\;\;\;\;\; \end{array}\;\right. $$ 

 $$ \left\lbrack \begin{array}{c} \dot{\;q_1 } \newline \dot{\;q_2 } \newline \ddot{\;q_1 } \newline \ddot{\;q_2 }  \end{array}\right\rbrack =\left\lbrack \begin{array}{cccc} 0 & 0 & 1 & 0\newline 0 & 0 & 0 & 1\newline 0 & \frac{\;m\;g}{M\;} & -\frac{\;k_1 }{M} & -\frac{\;k_2 }{M\;l}\newline 0 & \frac{\;\left(M+m\right)g}{M\;l} & -\frac{\;k_1 }{M\;l} & -\frac{\left(M+m\right)\;k_2 }{M\;m\;l^2 } \end{array}\right\rbrack \left\lbrack \begin{array}{c} q_1 \newline q_2 \newline \dot{\;q_1 } \newline \dot{\;q_2 }  \end{array}\right\rbrack +\left\lbrack \begin{array}{c} 0\newline 0\newline \frac{\;1}{M\;}\newline \frac{1}{M\;l} \end{array}\right\rbrack u $$ 

<a id="TMP_0400"></a>

## 3. Simulink Model of the system

The following values will taken for the system parameters

 $M=1\ldotp 5\;\mathrm{kg}\;,m=0\ldotp 5\;\mathrm{kg},\;g=9\ldotp 81\;m/s\;,\;l=1\;m,{\;k}_1 =k_2 =0\ldotp 01$, 

We will take $q_2 =y$ as the output of the system

```matlab


run("parameters.m");


[plant_ss, plant_tf, plant_zpk] = syst_repr_generator(params, 'q2');
plant_ss.StateName = {'q_1', 'q_2', '\dot{q_1}', '\dot{q_2}'}; 
q20 = 10;
Tsim = 10;
% plant_ss
plant_tf
```

```matlabTextOutput
plant_tf = 
     0.6667 s^2 + 1.4e-17 s + 6.598e-16
  ----------------------------------------
  s^4 + 0.03333 s^3 - 13.08 s^2 - 0.0654 s
 
Continuous-time transfer function.
Model Properties
```

```matlab
dcgain(plant_tf)
```

```matlabTextOutput
ans = Inf
```

```matlab


plant_zpk
```

```matlabTextOutput
plant_zpk = 
            0.66667 s
  -----------------------------
  (s+3.631) (s-3.602) (s+0.005)
 
Continuous-time zero/pole/gain model.
Model Properties
```

![image_1.png](./README_media/image_1.png)

![image_2.png](./README_media/image_2.png)

<a id="TMP_9f18"></a>

## 4. System analysis
<a id="TMP_3f67"></a>
 

<a id="TMP_8702"></a>

### Stability

```matlab
poles_syst = pole(plant_ss)
```

```matlabTextOutput
poles_syst = 4x1
   -3.6308
    3.6025
   -0.0050
         0


```

```matlab
zeros_syst = zero(plant_ss)
```

```matlabTextOutput
zeros_syst = 2x1
1.0e-16 *


    0.4043
         0


```

```matlab
pzmap(plant_ss)
```

The system is unstable because the second pole of the system is a real positive.

The only one zero of the system is negative making the system minimum phase.

<a id="TMP_963b"></a>
```matlab


rlocus(plant_ss)
```

![figure_0.png](./README_media/figure_0.png)

![figure_1.png](./README_media/figure_1.png)

 $$ O\;L\;\left(s\right)=\frac{N}{D}\;\;\;;P\;\mathrm{ctrl}\;;\Rightarrow C\;L\;\left(s\right)=\frac{P\;N}{D+P\;N\;\;} $$ 

From the root locus we can conclude that a finite P controller only cannot stabilize the system. In fact, the system has 3 poles and 1 zero, so only one pole is pulled toward the zero as gain increases toward infinity. Even the fact that the two stable poles which diverge on imaginary axis, their centroid point is in left half plan, keeping them stable. However the unstable pole approaches the zero position in the left half plan only when P is infinitely high. So no finite P controller can stabilize the system. 

Thus a solution of control is to design a controller that take into account integral, derivative and proportional effects through loop shaping. 

<a id="TMP_794e"></a>
<a id="TMP_560b"></a>

### Controllability and observability

```matlab
run parameters.m
[plant_ss_q1] = syst_repr_generator(params,'q1');
[plant_ss_q2] = syst_repr_generator(params,'q2');
plant_ss = plant_ss_q1;
Qc = ctrb(plant_ss);


rc = rank(Qc)
```

```matlabTextOutput
rc = 4
```

```matlab
ro_q1 = rank(obsv(plant_ss_q1))
```

```matlabTextOutput
ro_q1 = 4
```

```matlab
ro_q2 = rank(obsv(plant_ss_q2))
```

```matlabTextOutput
ro_q2 = 3
```

The system is controllable as the rank of the controllability matrix is equal to the dimension of the A. 

 $r_o =3\not= \dim \left(A\right)=4$, the system is not observable when $q_2$ is chosen as the output of the system. However it become observable for $q_1$ as output 

```matlab
plant_ss_diag = diagonalize(plant_ss)
```

```matlabTextOutput
plant_ss_diag = 
  A = 
           x1      x2      x3      x4
   x1       0       0       0       0
   x2       0  -3.631       0       0
   x3       0       0   3.602       0
   x4       0       0       0  -0.005
 
  B = 
           u1
   x1     100
   x2  0.3583
   x3  0.3547
   x4    -100
 
  C = 
             x1        x2        x3        x4
   y1         1  -0.06448   0.06479         1
 
  D = 
       u1
   y1   0
 
Continuous-time state-space model.
Model Properties
```

By diagonalizing the LTI model, we can directly see that all the state are controllable as B matrix is full. 

<a id="TMP_21f9"></a>

## 5.Control 
<a id="TMP_6de2"></a>

### Loop shaping with PID controller

In this section simulink Control System Designer tool is used to tune a PID controller that stabilize the pendulum. 

![image_3.png](./README_media/image_3.png)

For the simulation $q_{20} =10°$ 

```matlab
Tsim = 10;
Ts = 1e-4;
q2_f = deg2rad(-10);
params.IC = [0,deg2rad(1),0,0];
model = 2; % 1 : linear model,   2: non linear model  , 3: Matlab fct non linear model
[plant_ss] = syst_repr_generator(params,'q2');
input_select = plant_ss.C(1:2);
sim("models/mdl_with_PID.slx");
```

```matlabTextOutput
Warning: Model 'mdl_with_PID' contains 1 algebraic loops. 
Suggested Actions:
    • Highlight and view information about the algebraic loops in the model using the Simulink.BlockDiagram.getAlgebraicLoops function. - Open
    • Suppress this diagnostic by setting the 'Algebraic Loop' diagnostic parameter to 'none'. - Fix


Found algebraic loop that contains: 
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Sum7
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Sum8
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Gain12
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Gain13
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Divide3
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Sum9
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Sum10
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Gain8
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Divide
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Gain9
mdl_with_PID/Variant Subsystem plant/Non-linear model/Subsystem Reference/Sum (algebraic variable)
```

```matlab
refs = {'q1_ref', 'q2_ref'};
n = 1; 
figure(n);
animate_pendulum_cart(x.Time, x.Data(:,1), x.Data(:,2), u.Data,  params, 'fps', 100, string(refs(input_select==1)), ref.Data*input_select')
```

![figure_2.png](./README_media/figure_2.png)

By using simulink blocks to model the non\-linear system, we encounter algebraic loops because ${\ddot{q} }_1$ requires  ${\ddot{q} }_2$ and vice\-versa. To solve that, we will use Matlab function block, to solve the following ode. 

 $$ \left\lbrace \begin{array}{ll} \;\left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;\cos \left(q_2 \right)+m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right)+k_1 \;\dot{\;q_1 }  & =u\newline -m\;l\;\ddot{\;q_1 } \;\cos \left(q_2 \right)+m\;l^{2\;} \ddot{\;q_2 } -m\;g\;l\;\sin \left(q_2 \right)+k_2 \;\dot{\;q_2 } \; & =0 \end{array}\;\;\;\;\;=\Rightarrow \left\lbrace \begin{array}{ll} \;\left(M+m\right)\ddot{\;q_1 } -m\;l\ddot{\;q_2 } \;\cos \left(q_2 \right) & =u-m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right)-k_1 \;\dot{\;q_1 } \newline -m\;l\;\ddot{\;q_1 } \;\cos \left(q_2 \right)+m\;l^{2\;} \ddot{\;q_2 } \; & =m\;g\;l\;\sin \left(q_2 \right)-k_2 \;\dot{\;q_2 }  \end{array}\;\;\;\;\;\;\right.\right. $$ 

 $$ \left\lbrack \begin{array}{c} \ddot{\;q_1 } \newline \ddot{\;q_2 }  \end{array}\right\rbrack ={\left\lbrack \begin{array}{cccc} \left(M+m\right) & -m\;l\;\cos \left(q_2 \right)\newline -m\;l\;\cos \left(q_2 \right) & m\;l^{2\;}  \end{array}\right\rbrack }^{-1} \;\left\lbrack \begin{array}{c} \dot{\;u-m\;l\;{\dot{q_2 } }^2 \;\sin \left(q_2 \right)-k_1 \;\dot{\;q_1 } } \newline \dot{\;m\;g\;l\;\sin \left(q_2 \right)-k_2 \;\dot{\;q_2 } }  \end{array}\right\rbrack $$ 

```matlab
Tsim = 10;
Ts = 1e-4;
q2_f = deg2rad(-10);
params.IC = [0,deg2rad(1),0,0];
model = 3; % 1 : linear model,   2: non linear model  , 3: Matlab fct non linear model
[plant_ss] = syst_repr_generator(params,'q2');
input_select = plant_ss.C(1:2);
sim("models/mdl_with_PID.slx");
refs = {'q1_ref', 'q2_ref'};
clf("reset")
figure
animate_pendulum_cart(x.Time, x.Data(:,1), x.Data(:,2), u.Data,  params, 'fps', 100, string(refs(input_select==1)), ref.Data*input_select')
```

![figure_3.png](./README_media/figure_3.png)

<a id="TMP_0e02"></a>

### State feedback control

This control law is expressed as $u=-Kx+k_{\mathrm{s}\mathrm{c}\mathrm{a}\mathrm{l}\mathrm{e}} \;y_{\mathrm{r}\mathrm{e}\mathrm{f}}$ 

Classical state feedback controller is only applicable to SISO system, so we have to choose one of the two possible outputs

<a id="TMP_1364"></a>
### State feedback controller with cart position (q\_1) as output 

By default we will set $k_{\mathrm{s}\mathrm{c}\mathrm{a}\mathrm{l}\mathrm{e}} =1$ 

```matlab
[plant_ss]  = syst_repr_generator(params,'q1');
poles_des = [-3, -3, -3, -3];
rank(plant_ss.B);
K = acker(plant_ss.A,plant_ss.B, poles_des);
Tsim = 20;
Ts = 1e-4;
q2_f = deg2rad(10);
params.IC = [0,deg2rad(20),0,0];
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model
k_scale = 1;
input_select = plant_ss.C(1:2);
refs = {'q1_ref', 'q2_ref'};
sim('models/mdl_with_State_Feedback.slx');


clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_4.png](./README_media/figure_4.png)

```matlab
%animate_pendulum_cart(x.Time, x.Data(:,1), x.Data(:,2), u.Data,  params, 'fps', 100, string(refs(input_select==1)), ref.Data*input_select')


```

With the compensator $k_{\mathrm{scale}}$ set to default 1, a state feedback controller cannot assure a null static error.

To illustrate the compensator effect, let's consider a simple first order system with a static gain of 0.5. The step plot show a response ceilling at 0.5 instead of 1, the reference. With the compensator, the output is scaled to catch the reference. The same principle is used in state feedback controller.

```matlab
s = zpk('s');
toy_system = 2/(4 + s)
```

```matlabTextOutput
toy_system = 
    2
  -----
  (s+4)
 
Continuous-time zero/pole/gain model.
Model Properties
```

```matlab
clf(gcf)
figure
step(toy_system)
```

![figure_5.png](./README_media/figure_5.png)

```matlab
figure
clf(gcf)
dcg = dcgain(toy_system)
```

```matlabTextOutput
dcg = 0.5000
```

```matlab
compensator = 1/dcg
```

```matlabTextOutput
compensator = 2
```

```matlab
step(compensator*toy_system)
```

![figure_6.png](./README_media/figure_6.png)

By default we will set $k_{\mathrm{s}\mathrm{c}\mathrm{a}\mathrm{l}\mathrm{e}} =\frac{1}{\mathrm{d}\mathrm{c}\mathrm{g}\mathrm{a}\mathrm{i}\mathrm{n}\left(\mathrm{c}\mathrm{l}\mathrm{o}\mathrm{s}\mathrm{e}\mathrm{d}\;\mathrm{l}\mathrm{o}\mathrm{o}\mathrm{p}\right)}$ 

```matlab
[plant_ss]  = syst_repr_generator(params,'q1');
poles_des = [-3, -3, -3, -3];
rank(plant_ss.B);
K = acker(plant_ss.A,plant_ss.B, poles_des);
Tsim = 20;
Ts = 1e-4;
q2_f = deg2rad(20);
params.IC = [0,deg2rad(20),0,0];
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model


A_cl = plant_ss.A - plant_ss.B*K;
B_cl = plant_ss.B;
closed_loop_ss =  ss(A_cl, B_cl, plant_ss.C, 0);
[b,a] = ss2tf(A_cl,B_cl, plant_ss.C,0);
closed_loop_tf = tf(b,a)
```

```matlabTextOutput
closed_loop_tf = 
    0.6667 s^2 + 0.01333 s - 6.54
  ----------------------------------
  s^4 + 12 s^3 + 54 s^2 + 108 s + 81
 
Continuous-time transfer function.
Model Properties
```

```matlab
dcg = dcgain(closed_loop_ss);
k_scale = 1/dcg;
input_select = plant_ss.C(1:2);
refs = {'q1_ref', 'q2_ref'};


sim('models/mdl_with_State_Feedback.slx');
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_7.png](./README_media/figure_7.png)

```matlab
Tsim = 10;
Ts = 1e-4;
q2_f = deg2rad(20);
params.IC = [0,deg2rad(20),0,0];
model = 3; % 1 : linear model       2: non linear model
input_select = plant_ss.C(1:2);
sim('models/mdl_with_State_Feedback.slx');
refs = {'q1_ref', 'q2_ref'};
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_8.png](./README_media/figure_8.png)

<a id="TMP_3e49"></a>
### State feedback controller with swing angle (q\_2) as output 

Taking $q_2$ as output of the system reveals one of the implication of non observability show above. Actually, state feedback control only changes system's poles (the denominator of the the transfer function). Looking closely to the transfer function's numerator when $q_2$ is the output, we notice that $b_0 =-1\ldotp 866e-16\approx 0$, which result in $\mathrm{dcgain}=\frac{b_0 }{a_0 }\;$ being null and an infinite compensator $k_{\mathrm{scale}} =\infty$ 

```matlab
[plant_ss]  = syst_repr_generator(params,'q2');
poles_des = [-3, -3, -3, -3];
rank(plant_ss.B);
K = acker(plant_ss.A,plant_ss.B, poles_des);
Tsim = 20;
Ts = 1e-4;
q2_f = deg2rad(20);
params.IC = [0,deg2rad(20),0,0];
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model


A_cl = plant_ss.A - plant_ss.B*K;
B_cl = plant_ss.B;
closed_loop_ss =  ss(A_cl, B_cl, plant_ss.C, 0);
[b,a] = ss2tf(A_cl,B_cl, plant_ss.C,0);
closed_loop_tf = tf(b,a)
```

```matlabTextOutput
closed_loop_tf = 
  0.6667 s^2 + 1.668e-15 s - 1.866e-16
  ------------------------------------
   s^4 + 12 s^3 + 54 s^2 + 108 s + 81
 
Continuous-time transfer function.
Model Properties
```

```matlab
dcg = dcgain(closed_loop_ss)
```

```matlabTextOutput
dcg = 0
```

```matlab
k_scale = 1/dcg
```

```matlabTextOutput
k_scale = Inf
```

<a id="TMP_24bf"></a>

### LQR state feedback control

```matlab
[plant_ss]  = syst_repr_generator(params,'q1');
Q = 1000*eye(4);       
R  =0.001;
K = lqr(plant_ss.A, plant_ss.B, Q, R);
A_cl = plant_ss.A - plant_ss.B*K;
B_cl = plant_ss.B;
closed_loop =  ss(A_cl, B_cl, plant_ss.C, 0);


dcg = dcgain(closed_loop);
k_scale = 1/dcg;
Tsim = 20;
Ts = 1e-4;
q2_f = deg2rad(10);
params.IC = [0,deg2rad(10),0,0];
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model


input_select = plant_ss.C(1:2);
sim('models/mdl_with_State_Feedback.slx');
refs = {'q1_ref', 'q2_ref'};
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_9.png](./README_media/figure_9.png)

<a id="TMP_0d42"></a>
<a id="TMP_8889"></a>

### Observer based state feedback control

In reality the state are not directly accessible. They are either measured using sensors or estimated with techniques such Luenberger for deterministic systems and Kalman filter for stochastic systems. 

In this exercise will first start with Luenberger observer.

This observer expressed the observed states as : 

 $\dot{x_o } =A{\;x}_o +B\;u\;+L\left(y-y_o \right)$  ; $y_o =C\;x_o \;\;\mathrm{a}\mathrm{n}\mathrm{d}\;y=C\;x$ $\;\Rightarrow \;$ $\dot{\;x_o } =\left(A-L\;C\right)\;x_o +B\;u+L\;y\;$ 

where $y_o \;\mathrm{and}\;y$ are respectively the observed and real output. L is the Luenberger gain that can be calculated via pole placement in order the guaranty a response time of the observer 5 to 10 times faster than the closed loop of the system. 

```matlab


[plant_ss]  = syst_repr_generator(params,'q1');
poles_des = [-3, -3, -3, -3];
rank(plant_ss.B);
K = acker(plant_ss.A,plant_ss.B, poles_des);
Tsim = 20;
Ts = 1e-5;
q2_f = deg2rad(20);
params.IC = [0,deg2rad(20),0,0];
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model


A_cl = plant_ss.A - plant_ss.B*K;
B_cl = plant_ss.B;
closed_loop =  ss(A_cl, B_cl, plant_ss.C, 0);
obsv_poles = 5*poles_des;
L = acker(plant_ss.A', plant_ss.C', obsv_poles);
dcg = dcgain(closed_loop);
k_scale = 1/dcg;
input_select = plant_ss.C(1:2);
refs = {'q1_ref', 'q2_ref'};
model = 1;  % 1 : linear model       2: non linear model    3: Matlab fct non linear model
sim('models/mdl_with_State_Feedback_and_observer.slx');
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_10.png](./README_media/figure_10.png)

<a id="TMP_5877"></a>

### LQR State feedback with integrale action

In order to assure that static error elimination on the pendulum angle, we are going to augment the controller with integral action. 

```matlab
% Augmented system (integrale action on q2)
[plant_ss]  = syst_repr_generator(params,'q2');
P = [plant_ss.A, zeros(4,1); -plant_ss.C, 0]; % -C_q2
Rin = [plant_ss.B; 0];
Q = 2*eye(5);
R = 2.5;
K_aug = lqr(P, Rin, Q, R)
```

```matlabTextOutput
K_aug = 1x5
   -0.8944   52.4061   -2.5289   14.9684   -0.8465


```

```matlab
P_cl = P - Rin*K_aug
```

```matlabTextOutput
P_cl = 5x5
         0         0    1.0000         0         0
         0         0         0    1.0000         0
    0.5963  -31.6674    1.6793   -9.9856    0.5643
    0.5963  -21.8574    1.6793  -10.0056    0.5643
         0   -1.0000         0         0         0


```

```matlab
R_cl = Rin 
```

```matlabTextOutput
R_cl = 5x1
         0
         0
    0.6667
    0.6667
         0


```

```matlab
ss_cl =  ss(P_cl, R_cl, [plant_ss.C 0], 0);
pole(ss_cl)
```

```matlabTextOutput
ans = 5x1 complex
  -3.9379 + 0.0000i
  -3.3400 + 0.0000i
  -0.5242 + 0.4122i
  -0.5242 - 0.4122i
  -0.0000 + 0.0000i


```

```matlab


Tsim = 20;
Ts = 1e-4;
params.IC = [0,deg2rad(1),0,0];
model = 3; % 1: linear model       2: non linear model    3: Matlab fct non linear model
input_select = [0,1];
q2_f = deg2rad(10);
sim('models/mdl_with_State_Feedback_integrale.slx')
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_11.png](./README_media/figure_11.png)

<a id="TMP_1f8f"></a>

### Sliding mode control

Sliding mode control is a robust control strategy that aims to drag the system dynamic to a sliding surface 

```matlab
run parameters.m
[plant_ss]  = syst_repr_generator(params,'q1');
Qc = ctrb(plant_ss.A,plant_ss.B);
e = [zeros(1,3) 1];
p = 3;
lambda = (plant_ss.A + p*eye(4))^3;
F = e*inv(Qc)*lambda; 
k0 = 20;
phi = 0.5;
q1_ref = 5; 
params.IC = [0,deg2rad(10),0,0];
q2_f = deg2rad(2);
M = [plant_ss.A, plant_ss.B; F, 0];
x_u_inf = M[zeros(4,1); 1];
alpha = plant_ss.C*x_u_inf(1:end-1);
r = q1_ref/alpha; 
input_select = plant_ss.C(1:2);
Tsim = 20;
Ts=1e-4;
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model
sim('models/mdl_with_SMC.slx');
refs = {'q1_ref', 'q2_ref'};


clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_12.png](./README_media/figure_12.png)

```matlab
run parameters.m
[plant_ss]  = syst_repr_generator(params,'q1');
Qc = ctrb(plant_ss.A,plant_ss.B);
e = [zeros(1,3) 1];
p = 5;
lambda = (plant_ss.A + p*eye(4))^3;
F = e*inv(Qc)*lambda; 
k0 = 50;
phi = 0.5;
q1_ref = 5; 
params.IC = [0,deg2rad(10),0,0];
q2_f = deg2rad(2);
M = [plant_ss.A, plant_ss.B; F, 0];
x_u_inf = M[zeros(4,1); 1];
alpha = plant_ss.C*x_u_inf(1:end-1);
r = q1_ref/alpha; 
input_select = plant_ss.C(1:2);
Tsim = 20;
Ts=1e-4;
model = 3; % 1 : linear model       2: non linear model    3: Matlab fct non linear model
sim('models/mdl_with_SMC.slx');
refs = {'q1_ref', 'q2_ref'};
clf("reset")
figure;
X = squeeze(x.Data);              
if size(X,1) == 4, X = X.'; end  
animate_pendulum_cart(x.Time, X(:,1), X(:,2), u.Data, params, 'fps', 100, ...
    string(refs(input_select==1)), ref.Data*input_select');
```

![figure_13.png](./README_media/figure_13.png)

<a id="TMP_944b"></a>

### RST control

```matlab
Ts = 1e-4
```

```matlabTextOutput
Ts = 1.0000e-04
```

```matlab
discrete_syst = c2d(plant_ss, Ts, "tustin")
```

```matlabTextOutput
discrete_syst = 
  A = 
               x1          x2          x3          x4
   x1           1   1.635e-08      0.0001  -3.252e-11
   x2           0           1  -3.333e-11      0.0001
   x3           0    0.000327           1  -6.503e-07
   x4           0    0.001308  -6.667e-07           1
 
  B = 
              u1
   x1  3.333e-09
   x2  3.333e-09
   x3  6.667e-05
   x4  6.667e-05
 
  C = 
               x1          x2          x3          x4
   y1           1   8.175e-09       5e-05  -1.626e-11
 
  D = 
              u1
   y1  1.667e-09
 
Sample time: 0.0001 seconds
Discrete-time state-space model.
Model Properties
```