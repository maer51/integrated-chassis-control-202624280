function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 PI + 휠별 가산 슬립 타겟 ABS)
%
%   속도 추종(PI) + per-wheel "additive" ABS 를 통합한다.
%   harness 가 시나리오 제동을 가산만 허용(brake_total = brk_scenario + brakeESC)하므로,
%   잠긴 휠은 줄일 수 없지만 **덜 잠긴(여유 있는) 휠에 제동을 추가**하여 peak slip
%   (|κ|≈0.12)까지 활용해 정지거리를 단축한다.  wheel slip 은 runner 가
%   ctrlState.wheelSlip(4×1)로 전달한다.
%
%   Outputs:
%       forceCmd.Fx_total  [N]   (속도 추종, 양수 가속/음수 제동)
%       forceCmd.brakeRatio
%       forceCmd.brakeAdd  4×1 [Nm]  휠별 추가 제동 (coordinator 가 가산)

    m = 1500;
    kTgt = 0.12;     % peak-μ 목표 슬립

    if ~isfield(ctrlState,'intError')||isempty(ctrlState.intError); ctrlState.intError=0; end
    if ~isfield(ctrlState,'prevForce'); ctrlState.prevForce=0; end
    if ~isfield(ctrlState,'wheelSlip')||numel(ctrlState.wheelSlip)~=4; ctrlState.wheelSlip=zeros(4,1); end
    if ~isfield(ctrlState,'absAddInt')||numel(ctrlState.absAddInt)~=4; ctrlState.absAddInt=zeros(4,1); end

    %% 속도 추종 PI
    e_v = vxRef - vx;
    ctrlState.intError = ctrlState.intError + e_v*dt;
    ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    Fx = m*(CTRL.LON.Kp*e_v + CTRL.LON.Ki*ctrlState.intError);

    %% 저크/가속도 포화 + anti-windup
    MAXF = LIM.MAX_AX*m;  MAXR = LIM.MAX_JERK*m;
    dF = (Fx - ctrlState.prevForce)/dt;
    if abs(dF) > MAXR; Fx = ctrlState.prevForce + sign(dF)*MAXR*dt; end
    Fx = max(-MAXF, min(MAXF, Fx));
    if (Fx>=MAXF && e_v>0) || (Fx<=-MAXF && e_v<0)
        ctrlState.intError = ctrlState.intError - e_v*dt;
    end
    ctrlState.prevForce = Fx;

    %% 휠별 가산 슬립 타겟 ABS — 제동 중 여유 있는 휠을 peak 까지 활용
    kappa = ctrlState.wheelSlip;
    brakeAdd = zeros(4,1);
    KP_ADD = 30000;   KI_ADD = 250000;   ADD_MAX = 3000;
    % 직진 판정: 좌우 휠 슬립 대칭 (선회면 inner/outer 비대칭)
    straight = (abs(kappa(1)-kappa(2)) < 0.05) && (abs(kappa(3)-kappa(4)) < 0.05);
    if ax < -5.0 && straight   % 강한 직진 제동에서만 (선회 감속 A1/D1/A7 제외)
        for i = 1:4
            e = kTgt - abs(kappa(i));            % >0: 덜 잠김 → 제동 추가 / <0: 잠김 → 추가 0
            ctrlState.absAddInt(i) = ctrlState.absAddInt(i) + KI_ADD*e*dt;
            ctrlState.absAddInt(i) = max(0, min(ctrlState.absAddInt(i), ADD_MAX));
            brakeAdd(i) = max(0, min(KP_ADD*e + ctrlState.absAddInt(i), ADD_MAX));
        end
    else
        ctrlState.absAddInt = zeros(4,1);
    end

    %% 출력
    forceCmd.Fx_total = Fx;
    if Fx >= 0
        forceCmd.brakeRatio = 0;
    else
        forceCmd.brakeRatio = min(abs(Fx)/MAXF, 1.0);
    end
    forceCmd.brakeAdd = brakeAdd;
end
