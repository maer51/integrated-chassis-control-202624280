function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 PI + ABS slip-limiting)
%
%   속도 추종(PI) + anti-lock braking(휠 슬립 제한)을 통합한다.
%   wheel slip 은 runner 가 매 step ctrlState.wheelSlip(4×1) 에 캐시해 전달한다.
%
%       속도추종:  e_v=vxRef-vx,  Fx = m·(Kp·e_v + Ki·∫e_v)   (anti-windup)
%       ABS:       감속 중(ax<0) & |κ|>κ_target(0.12) → 제동력 감쇠 (bang-bang)
%       저크 제한:  |dFx/dt| ≤ MAX_JERK·m
%
%   Outputs: forceCmd.Fx_total [N] (양수 가속/음수 제동), forceCmd.brakeRatio [0~1]

    m = 1500;
    kTgt = 0.12;

    if ~isfield(ctrlState,'intError')||isempty(ctrlState.intError); ctrlState.intError=0; end
    if ~isfield(ctrlState,'prevForce'); ctrlState.prevForce=0; end
    if ~isfield(ctrlState,'wheelSlip')||numel(ctrlState.wheelSlip)~=4; ctrlState.wheelSlip=zeros(4,1); end

    %% 속도 추종 PI
    e_v = vxRef - vx;
    ctrlState.intError = ctrlState.intError + e_v*dt;
    ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    Fx = m*(CTRL.LON.Kp*e_v + CTRL.LON.Ki*ctrlState.intError);

    %% ABS — 감속 중 슬립 과대 시 제동 demand 감쇠
    kappa = ctrlState.wheelSlip;
    absActive = (ax < 0) && any(abs(kappa) > kTgt);
    if absActive && Fx < 0
        Fx = Fx * 0.5;        % bang-bang 제동력 감소
    end

    %% 저크/가속도 포화 + anti-windup
    MAXF = LIM.MAX_AX*m;  MAXR = LIM.MAX_JERK*m;
    dF = (Fx - ctrlState.prevForce)/dt;
    if abs(dF) > MAXR; Fx = ctrlState.prevForce + sign(dF)*MAXR*dt; end
    Fx = max(-MAXF, min(MAXF, Fx));
    if (Fx>=MAXF && e_v>0) || (Fx<=-MAXF && e_v<0)   % 포화 시 적분 정지
        ctrlState.intError = ctrlState.intError - e_v*dt;
    end
    ctrlState.prevForce = Fx;

    %% 출력
    forceCmd.Fx_total = Fx;
    if Fx >= 0
        forceCmd.brakeRatio = 0;
    else
        forceCmd.brakeRatio = min(abs(Fx)/MAXF, 1.0);
    end
end
