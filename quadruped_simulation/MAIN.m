% Representation-Free Model Predictive Control for Dynamic Quadruped Panther
% Author: Yanran Ding
% Last modified: 2020/12/21
% 
% Code accompanying the paper:
% Yanran Ding, Abhishek Pandala, Chuanzheng Li, Young-Ha Shin, Hae-Won Park
% "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds"
% Transactions on Robotics
% 
% preprint available at: https://arxiv.org/abs/2012.10002
% video available at: https://www.youtube.com/watch?v=iMacEwQisoQ&t=101s

%% initialization
%clear all; close all; clc
%addpath fcns fcns_MPC

% %% --- parameters ---
% % ---- gait ----
% % 0-trot; 1-bound; 2-pacing 3-gallop; 4-trot run; 5-crawl
% % ---- andatura --
% % 0-trotto; 1-vincolata dx/sx; 2-vincolata fw/bw 3-galoppo; 4-trotto corsa; 5-strisciare
% 
% gait = 5; 
% p = get_params(gait);
% p.playSpeed = 1;
% p.flag_movie = 1;       % 1 - make movie
% 
% dt_sim = p.simTimeStep;
% SimTimeDuration = 4.5;  % [sec]
% MAX_ITER = floor(SimTimeDuration/p.simTimeStep);
% 
% % desired trajectory
% p.acc_d = 1;
% p.vel_d = [0.5;0];
% p.yaw_d = 0;
% 
% % modified parameters
% p.mass = 5.5;
% p.vel_d = [0.5;0];
% p.mu = 1;            % friction coefficient


%% Model Predictive Control
% --- initial condition ---
% Xt = [pc dpc vR wb pf]': [30,1]
if gait == 1
    [p,Xt,Ut] = fcn_bound_ref_traj(p);
else
    [Xt,Ut] = fcn_gen_XdUd(0,[],[1;1;1;1],p);
end

% --- logging ---
tstart = 0;
tend = dt_sim;

[tout,Xout,Uout,Xdout,Udout,Uext,FSMout] = deal([]);

% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
for ii = 1:MAX_ITER
    % --- time vector ---
    t_ = dt_sim * (ii-1) + p.Tmpc * (0:p.predHorizon-1);
    
    % --- FSM ---
    if gait == 1

        [FSM,Xd,Ud,Xt] = fcn_FSM_bound(t_,Xt,p);
    else
        [FSM,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p);
    end

    % --- MPC ----
    % form QP
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);

    % Considering the matrices for the QP obtained from function fcn_get_QP_form_eta, use the QP solver qpSWIFT to 
    %  solve the quadratic problem with the following form 
    %  min. 0.5 * x' * H *x + g' * x
    %  s.t. Aineq *x <= bineq
    %      Aeq * x <= beq
    % 
    % The result of the QP problem should be stored in a variable called zval in order to be used in the following

%% 
    [zval, basic_info,adv_info] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);

%   [sol, basic_info, adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts)
%        minimize    0.5*x'Px + c'x
%        subject to  Ax = b
%                    Gx <= h
%    INPUT arguments:
%       P is a sparse matrix of dimension (n,n)
%       c is a dense column vector of size n
%       A is a sparse matrix of size (p,n); p is number of equality constraints
%       b is a dense column vector of size p
%       G is a sparse matrix of size (m,n); m is the number of inequality constraints
%       h is a dense column vector of size m
%       Opts is a structure with the following fields
%           -> MAXITER : maximum number of iterations needed
%           -> ABSTOL  : absolute tolerance
%           -> RELTOL  : relative tolerance
%           -> SIGMA   : maximum centering allowed
%           -> VERBOSE : PRINT LEVELS ||  0 -- No Print
%                                     || >0 -- Print everything
%           -> Permut  : permutation vector obtained as
%           	KKT = [P A' G';
%                    A 0   0;
%                    G 0 -I];
%             Permut = amd(KKT);
    
    
    %%
    
    
    Ut = Ut + zval(1:12);
    
    % --- external disturbance ---
    [u_ext,p_ext] = fcn_get_disturbance(tstart,p);
    p.p_ext = p_ext;        % position of external force
    u_ext = 0*u_ext;
    
    % --- simulate ---
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,0*u_ext,p),[tstart,tend],Xt);
    
    
    % --- update ---
    Xt = X(end,:)';
    tstart = tend;
    tend = tstart + dt_sim;
    
    % --- log ---  
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    Xdout = [Xdout;repmat(Xd(:,1)',[lent,1])];
    Udout = [Udout;repmat(Ud(:,1)',[lent,1])];
    Uext = [Uext;repmat(u_ext',[lent,1])];
    FSMout = [FSMout;repmat(FSM',[lent,1])];
    
    waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
end
close(h_waitbar)
fprintf('Calculation Complete!\n')
toc

%% Animation

[t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p,prefix);

%% Plot variables
t = (tout(1):p.simTimeStep:tout(end));
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);
Ue = interp1(tout,Uext,t);
t2 = repelem(t,2);
t2(1) = []; t2(end+1) = t2(end);
U2 = repelem(U,2,1);

%% states

%%%%%%%%% position %%%%%%%%%
legenda={'x', 'y', 'z', '$$x_d$$', '$$y_d$$', '$$z_d$$'};
nome_file = [prefix 'pos.pdf'];
multiplot(t,X(:,1:3),Xd(:,1:3),'time[sec]','position [m]',legenda,nome_file);

%%%%%%%%% velocity %%%%%%%%%
legenda={'$$v_x$$', '$$v_y$$', '$$v_z$$', '$$v_{xd}$$', '$$v_{yd}$$', '$$v_{zd}$$'};
nome_file = [prefix 'vel.pdf'];
multiplot(t,X(:,4:6),Xd(:,4:6),'time[sec]','velocity [m/s]',legenda,nome_file);

%%%%%%%%% Angular velocity %%%%%%%%%  
legenda={'$$\omega_x$$', '$$\omega_y$$', '$$\omega_z$$', '$$\omega_{xd}$$', '$$\omega_{yd}$$', '$$\omega_{zd}$$'};
nome_file = [prefix 'angVel.pdf'];
multiplot(t,X(:,16:18),Xd(:,16:18),'time[sec]','angular velocity [rad/s]',legenda,nome_file);

%% control
real = [U2(:,3) U2(:,6) U2(:,9) U2(:,12)];
desired = [Ud(:,3) Ud(:,6) Ud(:,9) Ud(:,12)];
legenda = {'$$Fz_1$$', '$$Fz_2$$', '$$Fz_3$$', '$$Fz_4$$', '$$Fz_{1d}$$', '$$Fz_{2d}$$', '$$Fz_{3d}$$', '$$Fz_{4d}$$'};
nome_file = [prefix 'Fz.pdf'];
singleplot(t2,t,real,desired,'time[sec]','Fz [N]',legenda,nome_file);



