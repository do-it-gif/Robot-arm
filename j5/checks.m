%% J5 CYCLOIDAL DRIVE - DESIGN CALCULATION (compact NEMA17 drive)
% Wrist pitch joint. Rotating housing = output, faceplate + carrier + M5 output pins fixed.
% 2x 6807 (35x47x7) housing | 2x 6702 (15x21x4) eccentric | 2x 6001 shaft ends | Zp=20 / Zd=19
%   geometry from the compact-drive design (PCD 48, e = 0.75, bare M5 on PCD 30)
%   J5 load case: gravity torque acts ON the joint axis (wrist pitch), not as tilt
%   Hertz uses the real disc curvature at each pin contact (no hard-coded rho_d)
%   bearing L10 at operating torque, static s0 at stall torque
%   6807 pair carries the two-disc reaction couple + link overhang
%   extra packaging checks (housing wall, bolt vs bearing seats, undercut, orbit clearance)
% SI units (m, N, kg, s, Pa). Section 14 gives MAX ALLOWABLE loads.
clear; clc;

% 0. GIVEN
% Cycloid geometry
Zp     = 20;          % ring pins
Zd     = 19;          % disc lobes
Rp     = 24e-3;       % ring pin circle RADIUS (PCD 48)
rp     = 2.5e-3;      % ring pin radius (dia 5 x 10). Dia 4 pins: rp = 2.0e-3, rr_prof = 2.1e-3
L_rp   = 10e-3;       % ring pin length
e      = 0.75e-3;     % eccentricity
t_d    = 4.75e-3;     % disc thickness (each)
n_d    = 2;           % discs, 180 deg phased
gap_d  = 0.5e-3;      % spacer between discs
rr_prof= rp + 0.1e-3; % pin radius used in disc profile (0.1 mm clearance)

% Housing
D_bore  = 47e-3;      % housing bore (= 6807 OD)
D_house = 54e-3;      % housing OD as modelled 
w_min   = 1.0e-3;     % min printed wall [m] 

% Output pins (bare M5 SHCS, threads in contact)
Zw    = 6;            % no. of output pins
Rw    = 15e-3;        % output pin circle RADIUS (PCD 30)
d_w   = 4.019e-3;     % bending dia = M5 minor dia
d_sl  = 4.83e-3;      % contact dia = measured M5 major dia
D_hole= 6.5e-3;       % disc output hole = 4.83 + 2e + 0.15 -> 6.5
f_thr = 0.5;          % bare thread crests: ~50% real bearing area vs smooth pin (ASSUMED)
D_bclr= 5.0e-3;       % bolt clearance hole in faceplate/carrier boss
D_ins = 7.0e-3;       % M5 heat-set insert OD
pin_support = 'simple';   % bolt clamped faceplate carrier plate
a_w   = 6e-3;         % (cantilever only)
L_w   = 11e-3;        % span boss face  carrier face (0.5 + 10 + 0.5)

% Eccentric shaft Ø5 motor shaft, 6702 lobes Ø15, 6001 seats Ø12)
d_lobe = 15e-3;       % 6702 bore
D_ecc  = 21e-3;       % 6702 OD = disc centre bore
d_sh   = 12e-3;       % smallest shaft OD (6001 seat)
d_sh_i = 5e-3;        % motor shaft bore (hollow section)
E_sh   = 70e9;        % 6061 Al. Printed PLA shaft: 3e9 
D_se   = 28e-3;       % shaft-end bearing OD (6001). 608 = 22e-3, 6702 = 21e-3
x_c1   = 6.875e-3;    % shaft-end bearing 1 centre
x_c2   = x_c1 + t_d + gap_d;  % disc 2 mid-plane
L_sh   = 19e-3;       % shaft-end bearing to bearing span

% Materials
E_pl   = 3.0e9;  nu_pl = 0.36;   % Sunlu PLA 2.0 (printed, ~3 GPa)
rho_pl = 1240;
infill = 0.20;
f_wall = 0.40;                    % solid wall/skin volume fraction
sig_contact_allow = 45e6;         % PLA Hertz allowable
sig_bear_allow    = 25e6;         % PLA crush allowable (projected area)
E_st = 200e9;   nu_st = 0.30;
Sy_bolt = 640e6;                  % 8.8 bolt
SF_req_bolt = 2.0;
s0_req = 2.0;                     % required static bearing SF

% Bearings (C / C0 [N]) - 6702 values are generic catalogue (NSK/ZWZ)
derate = 0.7;
C_6702 = 0.94e3*derate;  C0_6702 = 0.59e3*derate;
C_6001 = 5.40e3*derate;  C0_6001 = 2.36e3*derate;
C_6807 = 4.36e3*derate;  C0_6807 = 3.35e3*derate;   % SKF 61807
a_6807 = 18e-3;       % 6807 centre-to-centre (3.5 + 0.5 + 10 + 0.5 + 3.5)

% J5 load case: wrist pitch, link horizontal (worst case)
m_w5   = 0.60;        % J6 motor + J6 gearbox + gripper [kg]
L_w5   = 0.06;        % their COM from J5 axis [m]
m_pl   = 1.0;         % payload [kg]
L_pl   = 0.12;        % payload from J5 axis [m]
z_off  = 25e-3;       % axial offset: link load plane -> 6807 pair centre 

% Motor / motion
T_hold = 0.65;        % NEMA17 holding torque [Nm]
T_pull = 0.45;        % pull-out at ~100 rpm, TMC2209 @ 24 V
J_m    = 8.0e-6;      % rotor inertia [kg m^2] 
eta    = 0.80;        % gearbox efficiency
w_out_deg = 30;       % max output speed [deg/s]
t_acc  = 0.20;        % accel time [s]
T_fric = 0.10;        % output friction torque [Nm]
zeta   = 0.03;        % damping ratio
backlash_arcmin = 10; % upper bound
k_struct_factor = 0.5;% printed housing/carrier compliance knock-down
f_n_min = 10;         % min natural frequency for a wrist [Hz]

g = 9.81;

% 1. GEOMETRY / PACKAGING CHECKS
% R_tip = Rp - rr + e ; R_root = Rp - rr - e ; hole = d + 2e + clr
% w1 = bore->hole wall ; w2 = hole->root wall
R_tip  = Rp - rr_prof + e;   R_root = Rp - rr_prof - e;
w1 = Rw - D_hole/2 - D_ecc/2;
w2 = R_root - Rw - D_hole/2;
ax_cl = L_rp - (n_d*t_d + gap_d);
K1 = e*Zp/Rp;
w_house = D_house/2 - (Rp + rp);          % wall behind ring pins
pin_gap = 2*pi*Rp/Zp - 2*rp;              % web between pin grooves
orbit_cl= D_bore/2 - (R_tip + e);         % disc max reach vs housing bore
% bolt vs 6807 inner race (seat is Ø35 boss): bolt outer edge must stay inside r = 17.5
bolt_6807 = 35e-3/2 - (Rw + D_bclr/2);
% bolt/insert vs shaft-end bearing seat in the faceplate/carrier
bolt_se   = (Rw - max(D_bclr,D_ins)/2) - D_se/2;
% motor-bore wall in eccentric lobe
w_ecc = d_lobe/2 - d_sh_i/2 - e;

% Undercut: min convex radius of the pitch curve must exceed rr_prof
tt = linspace(0,2*pi,200001);
dx = -Rp*sin(tt) + e*Zp*sin(Zp*tt);       dy =  Rp*cos(tt) - e*Zp*cos(Zp*tt);
ddx= -Rp*cos(tt) + e*Zp^2*cos(Zp*tt);     ddy= -Rp*sin(tt) + e*Zp^2*sin(Zp*tt);
kap = (dx.*ddy - dy.*ddx)./(dx.^2 + dy.^2).^1.5;
rho_pitch = min(1./kap(kap>0));

fprintf('--- 1. Geometry ---\n disc tip/root dia = %.2f / %.2f mm | K1 = %.3f | rho_min pitch = %.2f mm (> rr %.2f)\n', ...
        2*R_tip*1e3, 2*R_root*1e3, K1, rho_pitch*1e3, rr_prof*1e3);
fprintf(' hole dia = %.2f mm | wall bore-hole = %.2f mm | wall hole-root = %.2f mm (want > %.1f)\n', ...
        D_hole*1e3, w1*1e3, w2*1e3, w_min*1e3);
fprintf(' housing wall behind pins = %.2f mm | pin web = %.2f mm | orbit clearance = %.2f mm\n', ...
        w_house*1e3, pin_gap*1e3, orbit_cl*1e3);
fprintf(' bolt -> 6807 seat = %.3f mm | bolt/insert -> shaft-end bearing seat = %.2f mm | eccentric wall = %.2f mm\n', ...
        bolt_6807*1e3, bolt_se*1e3, w_ecc*1e3);
fprintf(' ring pin axial clearance = %.2f mm\n', ax_cl*1e3);

% 2. KINEMATICS
i_g   = Zp/(Zp-Zd);
w_out = deg2rad(w_out_deg);
n_in  = w_out*i_g*60/(2*pi);
fprintf('--- 2. Kinematics ---\n i = %.0f | input = %.0f rpm\n', i_g, n_in);

% 3. J5 LOAD (gravity torque on the joint axis)
% T_grav = g (m_w5 L_w5 + m_pl L_pl) ; J_L = m_w5 L_w5^2 + m_pl L_pl^2 (point masses)
% Radial on 6807 pair F_r = g (m_w5 + m_pl) at axial offset z_off -> M_tilt = F_r z_off
J_L    = m_w5*L_w5^2 + m_pl*L_pl^2;
T_grav = g*(m_w5*L_w5 + m_pl*L_pl);
F_r    = g*(m_w5 + m_pl);
M_tilt = F_r*z_off;
J_ratio= J_L/i_g^2/J_m;
fprintf('--- 3. Load ---\n J_L = %.4f kg m^2 | inertia ratio = %.1f:1 | T_grav = %.2f Nm | F_r = %.1f N | M_tilt = %.2f Nm\n', ...
        J_L, J_ratio, T_grav, F_r, M_tilt);

% 4. TORQUE
% T_out = T_grav + J_L alpha + T_fric ; T_mot = T_out/(i eta) + J_m alpha i
% Strength checks use motor stall at output: T_des = T_hold i eta (crash / stall case)
alpha = w_out/t_acc;
T_out = T_grav + J_L*alpha + T_fric;
T_mot = T_out/(i_g*eta) + J_m*alpha*i_g;
T_des = T_hold*i_g*eta;
fprintf('--- 4. Torque ---\n alpha = %.2f rad/s^2 | T_out = %.2f Nm | T_motor = %.3f Nm | SF pull-out = %.2f\n', ...
        alpha, T_out, T_mot, T_pull/T_mot);
fprintf(' design (stall) torque at output T_des = %.2f Nm\n', T_des);

% 5. RING PIN FORCES (per disc)
% Contact normal of pin i passes through pitch point P = (e Zp, 0); disc centre (e,0)
% l_i = lever about disc centre ; F_i = Fmax l_i/l_max ; T_d = sum F_i l_i
T_d  = T_des/n_d*1.1;
phi  = (0:Zp-1)'*2*pi/Zp;
Pin  = Rp*[cos(phi) sin(phi)];
nvec = [e*Zp 0] - Pin;   nvec = nvec./sqrt(sum(nvec.^2,2));
rel  = Pin - [e 0];
l    = rel(:,1).*nvec(:,2) - rel(:,2).*nvec(:,1);
act  = l > 1e-9;
Fmax = T_d/sum(l(act).^2/max(l(act)));
F    = zeros(Zp,1);  F(act) = Fmax*l(act)/max(l(act));
Fvec = sum(F.*nvec,1);
F_ecc = norm(Fvec);                        % resultant on each 6702 (stall)
fprintf('--- 5. Ring pins ---\n T/disc = %.2f Nm | Fmax = %.0f N (approx %.0f) | loaded pins = %d | F on 6702 = %.0f N\n', ...
        T_d, Fmax, 4*T_d/(K1*Zd*Rp), nnz(act), F_ecc);

% 6. HERTZ CONTACT pin-disc (real disc curvature at each contact point)
% Disc profile (centre at origin) shifted by (e,0); contact point c_i = pin_i + rp n_i
% Signed disc radius rho (convex +, concave -): 1/R* = 1/rp + 1/rho
% p_i = sqrt(F_i E*/(pi L R*)) ; b_i = sqrt(4 F_i R*/(pi L E*))
t   = linspace(0,2*pi,20001);
psi = atan(sin((1-Zp)*t)./(Rp/(e*Zp) - cos((1-Zp)*t)));
xd  =  Rp*cos(t) - rr_prof*cos(t+psi) - e*cos(Zp*t) + e;
yd  = -Rp*sin(t) + rr_prof*sin(t+psi) + e*sin(Zp*t);
dxd = gradient(xd,t); dyd = gradient(yd,t);
kd  = (dxd.*gradient(dyd,t) - dyd.*gradient(dxd,t))./(dxd.^2+dyd.^2).^1.5;
sgn = sign(sum(xd(1:end-1).*diff(yd) - yd(1:end-1).*diff(xd)));    % profile runs CW -> flip so convex = +
Estar = 1/((1-nu_st^2)/E_st + (1-nu_pl^2)/E_pl);
p_pin = zeros(Zp,1); Rst_pin = inf(Zp,1);
for k = find(act)'
    c = Pin(k,:) + rp*nvec(k,:);
    [~,j] = min((xd-c(1)).^2 + (yd-c(2)).^2);
    rho = 1/(sgn*kd(j));                   % + convex, - concave
    Rst_pin(k) = 1/(1/rp + 1/rho);
    p_pin(k) = sqrt(F(k)*Estar/(pi*t_d*Rst_pin(k)));
end
[p_max,kp] = max(p_pin);
Rstar = Rst_pin(kp);
b_h   = sqrt(4*F(kp)*Rstar/(pi*t_d*Estar));
SF_contact = sig_contact_allow/p_max;
fprintf('--- 6. Contact ---\n worst pin #%d: F = %.0f N | R* = %.2f mm | b = %.3f mm | p_max = %.1f MPa | SF = %.2f\n', ...
        kp, F(kp), Rstar*1e3, b_h*1e3, p_max/1e6, SF_contact);

% 7. RING PIN GROOVE (housing)
% depth h = (Rp + rp) - bore radius ; beta = acos((rp-h)/rp) ; gam = force angle vs radial
h_gr  = (Rp + rp) - D_bore/2;
beta  = acosd((rp-h_gr)/rp);
rhat  = Pin./sqrt(sum(Pin.^2,2));
gam   = acosd(sum(-nvec.*rhat,2));
gam_max = max(gam(act));
sig_gr = 4*Fmax/(pi*2*rp*t_d);
SF_gr  = sig_bear_allow/sig_gr;
fprintf('--- 7. Housing groove ---\n depth = %.2f mm | half wrap = %.1f deg | max force angle = %.1f deg | crush = %.2f MPa | SF = %.1f\n', ...
        h_gr*1e3, beta, gam_max, sig_gr/1e6, SF_gr);

% 8. OUTPUT PINS
% Q_max = 4 T_d/(Zw Rw) ; simple: M = Q L/4, delta = Q L^3/(48EI)
% hole crush = Q/(d t f_thr) (bare thread -> reduced real area)
Q_max = 4*T_d/(Zw*Rw);
I_w   = pi*d_w^4/64;
if strcmp(pin_support,'cantilever')
    M_w = Q_max*a_w;   k_w = 3*E_st*I_w/a_w^3;
else
    M_w = Q_max*L_w/4; k_w = 48*E_st*I_w/L_w^3;
end
dl_w    = Q_max/k_w;
sig_w   = 32*M_w/(pi*d_w^3);
SF_w    = Sy_bolt/sig_w;
sig_hole= Q_max/(d_sl*t_d*f_thr);
SF_hole = sig_bear_allow/sig_hole;
fprintf('--- 8. Output pins ---\n Q_max = %.0f N | bending = %.0f MPa (SF %.1f) | defl = %.2f um | hole crush = %.1f MPa (SF %.2f)\n', ...
        Q_max, sig_w/1e6, SF_w, dl_w*1e6, sig_hole/1e6, SF_hole);

% 9. ECCENTRIC SHAFT (hollow Al sleeve)
% Simply supported between shaft-end bearings, +F at x1, -F at x2 (180 deg discs)
I_sh = pi*(d_sh^4 - d_sh_i^4)/64;
yP = @(Fp,a,x) ((x<=a).*(Fp*(L_sh-a).*x.*(L_sh^2-(L_sh-a)^2-x.^2)) + ...
                (x>a ).*(Fp*a.*(L_sh-x).*(L_sh^2-a^2-(L_sh-x).^2)))/(6*L_sh*E_sh*I_sh);
xs   = linspace(0,L_sh,401);
ysh  = yP(F_ecc,x_c1,xs) - yP(F_ecc,x_c2,xs);
R_se = F_ecc*(x_c2-x_c1)/L_sh;
fprintf('--- 9. Shaft ---\n max deflection (stall) = %.2f um | shaft-end bearing load = %.0f N each\n', ...
        max(abs(ysh))*1e6, R_se);

% 10. BEARINGS
% L10 [h] = (C/P)^3 1e6/(60 n) at OPERATING torque (forces scale with torque)
% s0 = C0/P0 at STALL torque
% 6807 pair: two-disc couple F_ecc (t_d+gap)/a + link load F_r/2 + M_tilt/a
kT     = T_out/T_des;                       % operating / stall
F_6702_op = F_ecc*kT;
L10_6702 = (C_6702/F_6702_op)^3*1e6/(60*n_in);   s0_6702 = C0_6702/F_ecc;
L10_6001 = (C_6001/max(R_se*kT,1))^3*1e6/(60*n_in); s0_6001 = C0_6001/max(R_se,1);
F_cpl  = F_ecc*(t_d+gap_d)/a_6807;          % disc couple reacted by 6807 pair (stall)
P_6807 = F_cpl + F_r/2 + M_tilt/a_6807;
s0_6807 = C0_6807/P_6807;
L10_6807 = (C_6807/(F_cpl*kT + F_r/2 + M_tilt/a_6807))^3*1e6/(60*n_in/i_g);
fprintf('--- 10. Bearings ---\n');
fprintf(' 6702: op load = %.0f N | L10 = %.0f h | stall s0 = %.2f (need %.1f)\n', F_6702_op, L10_6702, s0_6702, s0_req);
fprintf(' 6001: L10 = %.0f h | s0 = %.1f\n', L10_6001, s0_6001);
fprintf(' 6807: stall load = %.0f N (couple %.0f) | s0 = %.1f | L10 = %.0f h\n', P_6807, F_cpl, s0_6807, L10_6807);

% 11. TORSIONAL STIFFNESS
k_c    = pi/4*Estar*t_d;
k_mesh = sum(k_c*l(act).^2);
k_out  = k_w*Rw^2*Zw/4;
k_gear = n_d/(1/k_mesh + 1/k_out)*k_struct_factor;
k_mot  = T_hold*50*i_g^2;
k_t    = 1/(1/k_gear + 1/k_mot);
fprintf('--- 11. Stiffness [Nm/rad] ---\n mesh = %.0f | output pins = %.0f | gear = %.0f | motor(ref) = %.0f | total = %.0f\n', ...
        k_mesh, k_out, k_gear, k_mot, k_t);

% 12. RINGING
wn = sqrt(k_t/J_L);  fn = wn/(2*pi);  fd = fn*sqrt(1-zeta^2);
ts = 4/(zeta*wn);
f_in = n_in/60;  f_mesh = f_in*Zp/Zd;
fprintf('--- 12. Ringing ---\n fn = %.1f Hz | settling = %.2f s | f_in = %.2f Hz | f_mesh = %.2f Hz\n', fn, ts, f_in, f_mesh);
fprintf(' input rpm band to avoid: %.0f - %.0f rpm | ZV shaper delay = %.3f s\n', 0.8*fn*60, 1.2*fn*60, 1/(2*fd));

% 13. TIP ERROR (at payload radius)
d_wind = T_out/k_t*L_pl;
d_back = backlash_arcmin/60*pi/180*L_pl;
fprintf('--- 13. Tip error @ %.2f m ---\n wind-up = %.3f mm | backlash (%g arcmin) = %.3f mm | total = %.3f mm\n', ...
        L_pl, d_wind*1e3, backlash_arcmin, d_back*1e3, (d_wind+d_back)*1e3);

% 14. MAX ALLOWABLE
% Each limit linear in torque (contact: p ~ sqrt(F) -> T scales with SF^2)
T_lim = [ T_des*SF_contact^2;              % disc contact
          T_des*SF_gr;                     % housing groove
          T_des*SF_w/SF_req_bolt;          % output pin bending
          T_des*SF_hole;                   % disc hole crush
          T_des*s0_6702/s0_req;            % 6702 static
          T_des*s0_6807/s0_req;            % 6807 static (approx, couple-dominated)
          T_des ];                         % motor stall
names = {'disc contact','housing groove','output pin bend','disc hole crush','6702 static','6807 static','motor stall'};
[T_allow,kmin] = min(T_lim);
% Payload at L_pl: T_allow >= T_fric + g(m_w5 L_w5 + m L_pl) + alpha(m_w5 L_w5^2 + m L_pl^2)
den   = g*L_pl + alpha*L_pl^2;
base  = T_fric + g*m_w5*L_w5 + alpha*m_w5*L_w5^2;
m_pl_T   = (T_allow - base)/den;                                 % gearbox strength
T_mot_av = T_pull*i_g*eta - J_m*alpha*i_g^2*eta;                 % motor pull-out at output
m_pl_mot = (T_mot_av - base)/den;                                % motor (dynamic)
m_pl_hold= (T_hold*i_g*eta - T_fric - g*m_w5*L_w5)/(g*L_pl);     % motor holding, static
J_max_fn = k_t/(2*pi*f_n_min)^2;
m_pl_fn  = (J_max_fn - m_w5*L_w5^2)/L_pl^2;                      % stiffness
fprintf('--- 14. Max allowable ---\n');
for k = 1:numel(T_lim), fprintf(' %-18s T = %6.2f Nm\n', names{k}, T_lim(k)); end
fprintf(' >> allowable output torque = %.2f Nm (limited by %s)\n', T_allow, names{kmin});
fprintf(' >> max payload at %.2f m: gearbox %.2f kg | motor pull-out %.2f kg | motor hold %.2f kg | fn>%g Hz %.2f kg\n', ...
        L_pl, m_pl_T, m_pl_mot, m_pl_hold, f_n_min, m_pl_fn);

% 15. DISC MASS / UNBALANCE
A   = polyarea(xd-e,yd) - pi*(D_ecc/2)^2 - Zw*pi*(D_hole/2)^2;
m_d = A*t_d*rho_pl*(f_wall + (1-f_wall)*infill);
w_in = n_in*2*pi/60;
M_unb = m_d*e*w_in^2*(t_d+gap_d);
fprintf('--- 15. Disc ---\n mass = %.1f g each | unbalance couple = %.6f Nm\n', m_d*1e3, M_unb);

% SUMMARY
chk = {'Disc walls > w_min',          min(w1,w2) > w_min;
       'Housing wall behind pins',    w_house > w_min;
       'Pin web > w_min',             pin_gap > w_min;
       'Disc orbit clears bore',      orbit_cl > 0.3e-3;
       'No undercut',                 rho_pitch > rr_prof;
       'Bolt inside 6807 seat',       bolt_6807 >= -1e-6;
       'Bolt/insert clear shaft brg', bolt_se > w_min;
       'Motor pull-out SF > 1.5',     T_pull/T_mot > 1.5;
       'Disc contact SF > 1 (stall)', SF_contact > 1;
       'Disc contact OK (operating)', T_des*SF_contact^2 > T_out;
       'Housing groove SF > 1',       SF_gr > 1;
       'Pin seated (gam < beta)',     gam_max < beta;
       'Output pin SF > 2',           SF_w > SF_req_bolt;
       'Disc hole crush SF > 1',      SF_hole > 1;
       '6702 static s0 > 2',          s0_6702 > s0_req;
       '6807 static s0 > 2',          s0_6807 > s0_req;
       'fn > f_n_min',                fn > f_n_min;
       'Inertia ratio < 30',          J_ratio < 30};
fprintf('\n===== SUMMARY =====\n');
for k = 1:size(chk,1)
    s = 'FAIL'; if chk{k,2}, s = 'ok'; end
    fprintf(' %-28s %s\n', chk{k,1}, s);
end