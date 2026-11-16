%% J1 CYCLOIDAL DRIVE - DESIGN CALCULATION
% Base joint (rot. about z). Rotating housing = output, carrier/output pins fixed.
% 6812 output bearing | 2x 6001 eccentric | 2x 608 shaft ends | Zp=20 / Zd=19
%  SI units (m, N, kg, s, Pa).
% Section 14 gives MAX ALLOWABLE loads (each limit scaled to its allowable).
clear; clc;

% 0. GIVEN 
% Cycloid geometry
Zp    = 20;          % ring pins
Zd    = 19;          % disc lobes
Rp    = 37.5e-3;     % ring pin circle RADIUS (PCD 75)
rp    = 2.5e-3;      % ring pin radius (dia 5)
L_rp  = 20e-3;       % ring pin length
e     = 1.0e-3;      % eccentricity
t_d   = 9.5e-3;      % disc thickness (each)
n_d   = 2;           % discs, 180 deg phased
gap_d = 0.1e-3;      % grease gap between discs
rr_prof = 2.6e-3;    % pin radius used in disc profile (incl. clearance)

% Output pins (M5 SHCS)
Zw    = 6;           % no. of output pins
Rw    = 22.5e-3;     % output pin circle RADIUS 
d_w   = 4.019e-3;    % bending dia = M5 minor dia 
d_sl  = 5.2e-3;      % contact dia (sleeve)
pin_support = 'cantilever';   % 'cantilever' or 'simple'
a_w   = 15e-3;       % root -> far-disc mid-plane (cantilever)
L_w   = 25e-3;       % support span (if 'simple')

% Eccentric shaft (608 seats dia 8, 6001 cams dia 12)
d_sh  = 8e-3;        % smallest shaft dia (conservative)
L_sh  = 40e-3;       % 608 to 608 span 
x_c1  = 15.2e-3;     % 608 -> disc 1 mid-plane
x_c2  = x_c1 + t_d + gap_d;   % disc 2 mid-plane
D_ecc = 28e-3;       % 6001 OD = disc centre bore

% Materials
E_pl   = 3.0e9;  nu_pl = 0.36;   % Sunlu PLA 2.0 (printed, ~3 GPa)
rho_pl = 1240;
infill = 0.20;
f_wall = 0.40;                    % solid wall/skin volume fraction  
sig_contact_allow = 45e6;         % PLA Hertz allowable (short-term; creeps long-term)
sig_bear_allow    = 25e6;         % PLA crush allowable (projected area)
E_st = 200e9;   nu_st = 0.30;
Sy_bolt = 640e6;                  % 8.8 bolt
SF_req_bolt = 2.0;
s0_req = 2.0;                     % required static bearing SF

% Bearings (SKF C / C0 [N])
derate = 0.7;
C_6001 = 5.40e3*derate;  C0_6001 = 2.36e3*derate;
C_608  = 3.45e3*derate;  C0_608  = 1.37e3*derate;
C_6812 = 11.9e3*derate;  C0_6812 = 11.4e3*derate;
a_6812 = 20e-3;      % axial distance 6812 -> next tilt support

% Arm: one uniform rod (heavy) + payload at the tip, fully stretched (worst case)
m_arm = 3.0;         % whole arm beyond J1 kg
L_arm = 0.55;        % reach [m]
m_pl  = 1.0;         % payload [kg]

% Motor / motion
T_hold = 0.70;       % NEMA17 holding torque [Nm]
T_pull = 0.40;       % pull-out torque at op. speed [Nm]  
J_m    = 8.0e-6;     % rotor inertia [kg m^2]  
eta    = 0.80;       % gearbox efficiency (pins spin in greased PLA grooves) 
w_out_deg = 90;      % max output speed [deg/s]  
t_acc  = 0.30;       % accel time [s]   
T_fric = 0.3;        % output friction torque [Nm]   
zeta   = 0.03;       % damping ratio   
backlash_arcmin = 10;% upper bound of 0-10 arcmin  
k_struct_factor = 0.5;  % printed housing/carrier compliance knock-down
f_n_min = 5;         % minimum acceptable natural frequency [Hz]

g = 9.81;

%% 1. GEOMETRY CHECKS 
% Lobe tip/root: R_tip = Rp - rr + e ; R_root = Rp - rr - e
% Wall bore->hole: w1 = Rw - D_hole/2 - D_ecc/2 ; hole->root: w2 = R_root - Rw - D_hole/2
% Hole dia = pin contact dia + 2e ; axial clearance = L_rp - (n_d t_d + gap)
D_hole = d_sl + 2*e;
R_tip  = Rp - rr_prof + e;   R_root = Rp - rr_prof - e;
w1 = Rw - D_hole/2 - D_ecc/2;
w2 = R_root - Rw - D_hole/2;
ax_cl = L_rp - (n_d*t_d + gap_d);
K1 = e*Zp/Rp;
fprintf('--- 1. Geometry ---\n disc tip/root dia = %.1f / %.1f mm | K1 = %.3f\n', 2*R_tip*1e3, 2*R_root*1e3, K1);
fprintf(' hole dia = %.1f mm | wall bore-hole = %.2f mm | wall hole-root = %.2f mm (want > 2)\n', D_hole*1e3, w1*1e3, w2*1e3);
fprintf(' ring pin axial clearance = %.2f mm\n', ax_cl*1e3);

% 2. KINEMATICS 
% i = Zp/(Zp-Zd) (housing output, same direction) ; n_in = w_out*i
i_g   = Zp/(Zp-Zd);
w_out = deg2rad(w_out_deg);
n_in  = w_out*i_g*60/(2*pi);
fprintf('--- 2. Kinematics ---\n i = %.0f | input = %.0f rpm\n', i_g, n_in);

% 3. ARM INERTIA + TILT MOMENT 
% Rod about end: J = m L^2/3 ; payload: m_pl L^2
% Tilt (gravity, horizontal): M = g (m_arm L/2 + m_pl L) ; axial F = g(m_arm+m_pl)
% Reflected: J/i^2 ; inertia ratio = J_ref/J_m
J_L    = m_arm*L_arm^2/3 + m_pl*L_arm^2;
M_tilt = g*(m_arm*L_arm/2 + m_pl*L_arm);
F_ax   = g*(m_arm + m_pl);
J_ratio= J_L/i_g^2/J_m;
fprintf('--- 3. Load ---\n J_L = %.3f kg m^2 | inertia ratio = %.0f:1 | M_tilt = %.1f Nm | axial = %.0f N\n', ...
        J_L, J_ratio, M_tilt, F_ax);

% 4. TORQUE 
% alpha = w/t_acc ; T_out = J_L alpha + T_fric ; T_mot = T_out/(i eta) + J_m alpha i
% Strength checks use motor stall torque at output: T_des = T_hold i eta (worst case)
alpha = w_out/t_acc;
T_out = J_L*alpha + T_fric;
T_mot = T_out/(i_g*eta) + J_m*alpha*i_g;
T_des = T_hold*i_g*eta;
fprintf('--- 4. Torque ---\n alpha = %.2f rad/s^2 | T_out = %.2f Nm | T_motor = %.3f Nm | SF pull-out = %.2f\n', ...
        alpha, T_out, T_mot, T_pull/T_mot);
fprintf(' design (stall) torque at output T_des = %.2f Nm\n', T_des);

%  5. RING PIN FORCES (per disc) 
% Contact normal of pin i passes through pitch point P = (e Zp, 0); disc centre (e,0)
% l_i = lever about disc centre ; F_i = Fmax l_i/l_max ; T_d = sum F_i l_i
% T_d = T_des/n_d * 1.1 (unequal sharing)
% Approx check: Fmax ~ 4 T_d/(K1 Zd Rp)
T_d  = T_des/n_d*1.1;
phi  = (0:Zp-1)'*2*pi/Zp + pi/Zp/2;
Pin  = Rp*[cos(phi) sin(phi)];
nvec = [e*Zp 0] - Pin;   nvec = nvec./sqrt(sum(nvec.^2,2));
rel  = Pin - [e 0];
l    = rel(:,1).*nvec(:,2) - rel(:,2).*nvec(:,1);
act  = l > 0;
Fmax = T_d/sum(l(act).^2/max(l(act)));
F    = zeros(Zp,1);  F(act) = Fmax*l(act)/max(l(act));
F_ecc = norm(sum(F.*nvec,1));            % resultant on each 6001
fprintf('--- 5. Ring pins ---\n T/disc = %.2f Nm | Fmax = %.0f N (approx %.0f) | loaded pins = %d | F on 6001 = %.0f N\n', ...
        T_d, Fmax, 4*T_d/(K1*Zd*Rp), nnz(act), F_ecc);

%  6. HERTZ CONTACT pin-disc
% 1/R* = 1/rp + 1/rho_d (rho_d ~ 7.5 mm min convex lobe radius, e = 1)
% 1/E* = (1-nu1^2)/E1 + (1-nu2^2)/E2
% b = sqrt(4F R*/(pi L E*)) ; p_max = 2F/(pi b L) = sqrt(F E*/(pi L R*))
rho_d = 7.5e-3;
Rstar = 1/(1/rp + 1/rho_d);
Estar = 1/((1-nu_st^2)/E_st + (1-nu_pl^2)/E_pl);
b_h   = sqrt(4*Fmax*Rstar/(pi*t_d*Estar));
p_max = 2*Fmax/(pi*b_h*t_d);
SF_contact = sig_contact_allow/p_max;
fprintf('--- 6. Contact ---\n b = %.3f mm | p_max = %.1f MPa | SF = %.2f\n', b_h*1e3, p_max/1e6, SF_contact);

%  7. RING PIN GROOVE (housing) 
% Pins sit in a one-piece groove, depth h slightly < rp (pin sticks out a bit > half).
% Pin fully backed -> no pin bending.
% Half wrap angle: beta = acos((rp-h)/rp) ; force angle vs radial: gam = acos(-n . r_hat)
% Pin stays seated if gam_max < beta (no lip loading).
%  pin-in-groove (cosine pressure): p_max = 4F/(pi d L_eff), L_eff = t_d (load under one disc)
h_gr  = 2.3e-3;                                  % groove depth
beta  = acosd((rp-h_gr)/rp);
rhat  = Pin./sqrt(sum(Pin.^2,2));
gam   = acosd(sum(-nvec.*rhat,2));
gam_max = max(gam(act));
sig_gr = 4*Fmax/(pi*2*rp*t_d);
SF_gr  = sig_bear_allow/sig_gr;
fprintf('--- 7. Housing groove ---\n half wrap = %.1f deg | max force angle = %.1f deg | crush = %.2f MPa | SF = %.1f\n', ...
        beta, gam_max, sig_gr/1e6, SF_gr);
% Pins captured at both ends, free to spin in groove: pin rolls on disc if
% groove friction torque < disc friction torque -> mu_groove < mu_disc (same F, same rp).
% Sliding still happens, just in the groove (plain journal) instead of on the lobe.
fprintf(' note: pins spin in groove -> sliding moved to groove; rolls on disc only if mu_groove < mu_disc\n');

% 8. OUTPUT PINS 
% Q_max = 4 T_d/(Zw Rw)
% cantilever: M = Q a, delta = Q a^3/(3EI) | simple: M = Q L/4, delta = Q L^3/(48EI)
% sigma = 32M/(pi d^3) ; I = pi d^4/64 ; hole crush = Q/(d_sl t_d)
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
sig_hole= Q_max/(d_sl*t_d);
SF_hole = sig_bear_allow/sig_hole;
fprintf('--- 8. Output pins ---\n Q_max = %.0f N | bending = %.0f MPa (SF %.2f, need %.1f) | defl = %.1f um | hole crush = %.1f MPa (SF %.1f)\n', ...
        Q_max, sig_w/1e6, SF_w, SF_req_bolt, dl_w*1e6, sig_hole/1e6, SF_hole);

% 9. ECCENTRIC SHAFT 
% Simply supported 608-608, loads +F at x1 and -F at x2 (180 deg discs)
% Point load F at a (b = L-a):  x<=a: y = F b x (L^2-b^2-x^2)/(6 L E I)
%                               x> a: y = F a (L-x)(L^2-a^2-(L-x)^2)/(6 L E I)
% 608 reactions (couple): R = F (x2-x1)/L
I_sh = pi*d_sh^4/64;
yP = @(Fp,a,x) ((x<=a).*(Fp*(L_sh-a).*x.*(L_sh^2-(L_sh-a)^2-x.^2)) + ...
                (x>a ).*(Fp*a.*(L_sh-x).*(L_sh^2-a^2-(L_sh-x).^2)))/(6*L_sh*E_st*I_sh);
xs   = linspace(0,L_sh,401);
ysh  = yP(F_ecc,x_c1,xs) - yP(F_ecc,x_c2,xs);
R608 = F_ecc*(x_c2-x_c1)/L_sh;
fprintf('--- 9. Shaft ---\n max deflection = %.2f um | 608 load = %.0f N each\n', max(abs(ysh))*1e6, R608);

% 10. BEARINGS 
% L10 [h] = (C/P)^3 1e6/(60 n) ; static s0 = C0/P0
% 6812 under tilt: Fr = M_tilt/a_6812 ; P0 = max(Fr, 0.6Fr + 0.5Fa) (ISO 76 DGBB)
L10_6001 = (C_6001/F_ecc)^3*1e6/(60*n_in);   s0_6001 = C0_6001/F_ecc;
L10_608  = (C_608/max(R608,1))^3*1e6/(60*n_in); s0_608 = C0_608/max(R608,1);
Fr_6812  = M_tilt/a_6812;
P0_6812  = max(Fr_6812, 0.6*Fr_6812 + 0.5*F_ax);
s0_6812  = C0_6812/P0_6812;
fprintf('--- 10. Bearings (at stall torque, continuous) ---\n');
fprintf(' 6001: L10 = %.0f h | s0 = %.1f\n 608 : L10 = %.0f h | s0 = %.1f\n', L10_6001, s0_6001, L10_608, s0_608);
fprintf(' 6812: Fr = %.0f N from tilt | s0 = %.2f (need %.1f)\n', Fr_6812, s0_6812, s0_req);

% 11. TORSIONAL STIFFNESS 
% Contact (linearised line contact): k_c ~ (pi/4) E* L     
% k_mesh = sum k_c l_i^2 ; k_out = k_w Rw^2 Zw/4
% per disc in series, discs in parallel, * structural knock-down
% stepper magnetic stiffness k_m ~ T_hold*50, reflected * i^2
k_c    = pi/4*Estar*t_d;
k_mesh = sum(k_c*l(act).^2);
k_out  = k_w*Rw^2*Zw/4;
k_gear = n_d/(1/k_mesh + 1/k_out)*k_struct_factor;
k_mot  = T_hold*50*i_g^2;
k_t    = 1/(1/k_gear + 1/k_mot);
fprintf('--- 11. Stiffness [Nm/rad] ---\n mesh = %.0f | output pins = %.0f | gear = %.0f | motor(ref) = %.0f | total = %.0f\n', ...
        k_mesh, k_out, k_gear, k_mot, k_t);

% 12. RINGING 
% wn = sqrt(k_t/J_L) ; fd = fn sqrt(1-zeta^2) ; ts(2%) = 4/(zeta wn)
% excitations: f_in = n_in/60 ; f_mesh = f_in Zp/Zd ; avoid within +-20% of fn
% ZV input shaper delay = 1/(2 fd)
wn = sqrt(k_t/J_L);  fn = wn/(2*pi);  fd = fn*sqrt(1-zeta^2);
ts = 4/(zeta*wn);
f_in = n_in/60;  f_mesh = f_in*Zp/Zd;
fprintf('--- 12. Ringing ---\n fn = %.2f Hz | settling = %.2f s | f_in = %.2f Hz | f_mesh = %.2f Hz\n', fn, ts, f_in, f_mesh);
fprintf(' input rpm band to avoid: %.0f - %.0f rpm | ZV shaper delay = %.3f s\n', 0.8*fn*60, 1.2*fn*60, 1/(2*fd));

%  13. TIP ERROR 
% wind-up: d = T_out/k_t * L ; backlash: d = theta_b * L
d_wind = T_out/k_t*L_arm;
d_back = backlash_arcmin/60*pi/180*L_arm;
fprintf('--- 13. Tip error @ %.2f m ---\n wind-up = %.3f mm | backlash (%g arcmin) = %.3f mm | total = %.3f mm\n', ...
        L_arm, d_wind*1e3, backlash_arcmin, d_back*1e3, (d_wind+d_back)*1e3);

% 14. MAX ALLOWABLE
% Each limit is linear in torque (contact: p ~ sqrt(F) -> F_allow = p_allow^2 pi L R*/E*)
% T_allow,k = T_des * (allowable/actual) ; T_allow = min over limits
F_allow_c = sig_contact_allow^2*pi*t_d*Rstar/Estar;
T_lim = [ T_des*F_allow_c/Fmax;            % disc contact
          T_des*SF_gr;                     % housing groove
          T_des*SF_w/SF_req_bolt;          % output pin bending
          T_des*SF_hole;                   % disc hole crush
          T_des*s0_6001/s0_req;            % 6001 static
          T_des ];                         % motor stall
names = {'disc contact','housing groove','output pin bend','disc hole crush','6001 static','motor stall'};
[T_allow,kmin] = min(T_lim);
% Allowable load: J_max from torque and from fn_min ; tilt from 6812 static
J_max_T  = (T_allow - T_fric)/alpha;
J_max_fn = k_t/(2*pi*f_n_min)^2;
J_max    = min(J_max_T, J_max_fn);
M_allow  = C0_6812/s0_req*a_6812;                 % Fr = M/a  (Fa term ignored)
m_pl_J   = (J_max - m_arm*L_arm^2/3)/L_arm^2;     % max payload at tip (inertia)
m_pl_M   = (M_allow/g - m_arm*L_arm/2)/L_arm;     % max payload at tip (tilt)
fprintf('--- 14. Max allowable ---\n');
for k = 1:numel(T_lim), fprintf(' %-18s T = %6.2f Nm\n', names{k}, T_lim(k)); end
fprintf(' >> allowable output torque = %.2f Nm (limited by %s)\n', T_allow, names{kmin});
fprintf(' >> J_max = %.3f kg m^2 (torque %.3f | fn>%g Hz %.3f)\n', J_max, J_max_T, f_n_min, J_max_fn);
fprintf(' >> max tilt moment = %.1f Nm\n', M_allow);
fprintf(' >> max payload at %.2f m with %.1f kg arm: %.2f kg (inertia) | %.2f kg (tilt)\n', ...
        L_arm, m_arm, m_pl_J, m_pl_M);

% 15. DISC MASS / UNBALANCE 
% profile area (polyarea) - bore - holes ; rho_eff = rho (f_wall + (1-f_wall) infill)
% two discs 180 deg -> force cancels, couple M = m e w^2 (t_d + gap)
t   = linspace(0,2*pi,4000);
psi = atan(sin(-Zd*t)./(Rp/(e*Zp) - cos(-Zd*t)));
x   = Rp*cos(t) - rr_prof*cos(t+psi) - e*cos(Zp*t);
y   = -Rp*sin(t) + rr_prof*sin(t+psi) + e*sin(Zp*t);
A   = polyarea(x,y) - pi*(D_ecc/2)^2 - Zw*pi*(D_hole/2)^2;
m_d = A*t_d*rho_pl*(f_wall + (1-f_wall)*infill);
w_in = n_in*2*pi/60;
M_unb = m_d*e*w_in^2*(t_d+gap_d);
fprintf('--- 15. Disc ---\n mass = %.1f g each | unbalance couple = %.5f Nm\n', m_d*1e3, M_unb);

%  SUMMARY 
chk = {'Geometry walls > 2 mm',    min(w1,w2) > 2e-3;
       'Motor pull-out SF > 1.5',  T_pull/T_mot > 1.5;
       'Disc contact SF > 1',      SF_contact > 1;
       'Housing groove SF > 1',    SF_gr > 1;
       'Pin seated (gam < beta)',  gam_max < beta;
       'Output pin SF > 2',        SF_w > SF_req_bolt;
       'Disc hole crush SF > 1',   SF_hole > 1;
       '6001 static s0 > 2',       s0_6001 > s0_req;
       '6812 static s0 > 2',       s0_6812 > s0_req;
       'fn > f_n_min',             fn > f_n_min;
       'Inertia ratio < 30',       J_ratio < 30};
fprintf('\n===== SUMMARY =====\n');
for k = 1:size(chk,1)
    s = 'FAIL'; if chk{k,2}, s = 'ok'; end
    fprintf(' %-26s %s\n', chk{k,1}, s);
end