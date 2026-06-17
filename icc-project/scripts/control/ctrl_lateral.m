function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS: 기동 적응형 LQR/MPC 전환 + ESC)
%
%   기준 요레이트 파형으로 기동 종류를 추정해 두 최적제어기를 전환한다:
%     - 한 방향 스텝/정상선회 → MPC (예측으로 과도응답 우수)
%     - 좌우 반전 급회피(DLC)  → 적분증강 LQR (정상상태 오차 0 → LTR/슬립 우수)
%   기준값이 한 번이라도 유의미하게 부호 반전하면 그 주행을 DLC 로 래치한다.
%   저속은 항상 LQR. (시나리오 id 분기·global 미사용)
%
%       에러상태 e=[vy-vy_ref; r-rRef; ∫(r-rRef)],  vy_ref=Kβ(vx)·rRef
%       LQR: δ=-K e,  K=dlqr(Ad,Bd,Q,R)          (DARE 오프라인 해)
%       MPC: min Σ(w_vy e_vy²+w_r e_r²)+ru δ²+qdu Δδ²  s.t. |δ|,|Δδ|≤lim
%            → condensed QP → Hildreth(quadprog 불필요) → RHC 첫 입력
%       ESC: |β|>2.5° → 보상 요모멘트 (coordinator 가 차동제동으로 실현)

    %% 차량 파라미터 (sim_params generic set)
    m=1500; Iz=2500; lf=1.2; lr=1.4; Cf=80000; Cr=85000;

    %% 상태 초기화
    if ~isfield(ctrlState,'intError')||isempty(ctrlState.intError); ctrlState.intError=0; end
    if ~isfield(ctrlState,'prevError'); ctrlState.prevError=0; end
    if ~isfield(ctrlState,'prevDelta'); ctrlState.prevDelta=0; end
    if ~isfield(ctrlState,'refSign');   ctrlState.refSign=0;   end
    if ~isfield(ctrlState,'isDLC');     ctrlState.isDLC=false; end

    %% 기동 종류 감지 (기준 요레이트 부호 반전 래치)
    REFTH = 0.03;
    if ~ctrlState.isDLC && abs(yawRateRef) > REFTH
        s = sign(yawRateRef);
        if ctrlState.refSign == 0
            ctrlState.refSign = s;
        elseif s ~= ctrlState.refSign
            ctrlState.isDLC = true;
        end
    end
    useMPC = (abs(vx) >= 12) && ~ctrlState.isDLC;

    %% Bicycle Model 상태공간
    vxs = vx; if abs(vxs) < 1.0; vxs = sign(vxs + eps)*1.0; end
    a11=-(Cf+Cr)/(m*vxs);  a12=-vxs-(Cf*lf-Cr*lr)/(m*vxs);
    a21=-(Cf*lf-Cr*lr)/(Iz*vxs);  a22=-(Cf*lf^2+Cr*lr^2)/(Iz*vxs);
    A=[a11 a12;a21 a22];  B=[Cf/m; Cf*lf/Iz];

    ss=-A\B;
    if abs(ss(2))>1e-6; Kbeta=ss(1)/ss(2); else; Kbeta=0; end
    vy_ref=Kbeta*yawRateRef;
    vy=vx*tan(slipAngle);
    e_vy=vy-vy_ref;  e_r=yawRate-yawRateRef;

    %% AFS 제어
    if useMPC
        N=20; w_vy=0.3; w_r=30; ru=80; qdu=300;
        MP=local_mpc_matrices(A,B,dt,N,w_vy,w_r,ru,qdu,LIM.MAX_STEER_ANGLE,LIM.MAX_STEER_RATE*dt,vxs);
        uprev=[ctrlState.prevDelta; zeros(N-1,1)];
        f=MP.ThetaQ*[e_vy;e_r]-MP.TtQdu*uprev;
        bin=MP.bin_const+[zeros(2*N,1);uprev;-uprev];
        U=local_hildreth(MP.H,2*f,MP.Ain,bin,MP.Hinv,MP.P);
        steerCmd=U(1);
        if ~isfinite(steerCmd); steerCmd=ctrlState.prevDelta; end
    else
        if abs(vx)<12; Q=diag([12,80,40]); R=6; else; Q=diag([0.3,30,1]); R=12; end
        K=local_lqr_gain(A,B,Q,R,dt,vxs);
        xi=ctrlState.intError;
        steerCmd=-(K(1)*e_vy+K(2)*e_r+K(3)*xi);
        if ~(abs(steerCmd)>=LIM.MAX_STEER_ANGLE && sign(steerCmd)==sign(e_r))
            ctrlState.intError=xi+e_r*dt;
            ctrlState.intError=max(-CTRL.LAT.intMax,min(CTRL.LAT.intMax,ctrlState.intError));
        end
    end
    steerCmd=max(-LIM.MAX_STEER_ANGLE,min(LIM.MAX_STEER_ANGLE,steerCmd));
    ctrlState.prevDelta=steerCmd;
    ctrlState.prevError=-e_r;

    %% ESC 슬립 리미터
    BETA_TH=deg2rad(2.5); BETA_GAIN=9000;
    if abs(slipAngle)>BETA_TH
        yawMoment=-BETA_GAIN*(slipAngle-sign(slipAngle)*BETA_TH);
    else
        yawMoment=0;
    end
    yawMoment=yawMoment*min(vx/20,2.0);

    deltaAdd.steerAngle=steerCmd;
    deltaAdd.yawMoment=yawMoment;
end

%% ============================================================
function K = local_lqr_gain(A,B,Q,R,dt,vx)
    persistent CACHE
    if isempty(CACHE); CACHE=containers.Map('KeyType','double','ValueType','any'); end
    vxKey=round(max(abs(vx),1.0)*2)/2;
    if isKey(CACHE,vxKey); K=CACHE(vxKey); return; end
    Aaug=[A,[0;0];0 1 0]; Baug=[B;0];
    M=expm([Aaug,Baug;zeros(1,4)]*dt); Ad=M(1:3,1:3); Bd=M(1:3,4);
    K=dlqr(Ad,Bd,Q,R); CACHE(vxKey)=K;
end

%% ============================================================
function MP = local_mpc_matrices(A,B,dt,N,w_vy,w_r,ru,qdu,dmax,dRate,vx)
    persistent CACHE
    if isempty(CACHE); CACHE=containers.Map('KeyType','char','ValueType','any'); end
    vxKey=round(max(abs(vx),1.0)*2)/2; key=sprintf('%.1f_%d',vxKey,N);
    if isKey(CACHE,key); MP=CACHE(key); return; end
    Md=expm([A,B;zeros(1,3)]*dt); Ad=Md(1:2,1:2); Bd=Md(1:2,3);
    n=2; Psi=zeros(n*N,n); Theta=zeros(n*N,N); Apow=eye(n);
    for k=1:N; Apow=Apow*Ad; Psi(n*(k-1)+1:n*k,:)=Apow; end
    for k=1:N; for j=1:k; Theta(n*(k-1)+1:n*k,j)=(Ad^(k-j))*Bd; end; end
    Qbar=kron(eye(N),diag([w_vy,w_r])); Rbar=ru*eye(N);
    T=eye(N)-diag(ones(N-1,1),-1); Qdu=qdu*eye(N);
    H=2*(Theta'*Qbar*Theta+Rbar+T'*Qdu*T); H=(H+H')/2;
    MP.H=H; MP.Hinv=H\eye(N);
    MP.ThetaQ=Theta'*Qbar*Psi; MP.TtQdu=T'*Qdu;
    MP.Ain=[eye(N);-eye(N);T;-T];
    MP.bin_const=[dmax*ones(N,1);dmax*ones(N,1);dRate*ones(N,1);dRate*ones(N,1)];
    MP.P=MP.Ain*MP.Hinv*MP.Ain'; CACHE(key)=MP;
end

%% ============================================================
function x = local_hildreth(H,f,Ain,b,Hinv,P)
    x=-Hinv*f;
    if all(Ain*x-b<=1e-9); return; end
    d=b+Ain*Hinv*f; m=numel(d); lam=zeros(m,1);
    Pd=diag(P); Pd(Pd<1e-12)=1e-12;
    for it=1:120
        lam0=lam;
        for i=1:m; w=P(i,:)*lam-P(i,i)*lam(i)+d(i); lam(i)=max(0,-w/Pd(i)); end
        if norm(lam-lam0,inf)<1e-9; break; end
    end
    x=-Hinv*(f+Ain'*lam);
end
