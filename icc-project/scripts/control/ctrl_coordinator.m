function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기 명령(yaw moment, Fx_total, damping)을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping)로 변환한다.
%
%       조향:   latCmd.steerAngle 통과 + 포화
%       제동:   ESC yaw moment → 좌우 차동 제동(전후 60:40)
%               + 종방향 제동(lonCmd.Fx_total<0) 4륜 균등
%       댐퍼:   verCmd 통과
%       안전:   마찰원 초과 시 제동 스케일다운, [0,MAX_BRAKE] 클리핑

    %% 1. 조향 (AFS) 통과 + 포화
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                                  min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% 2. 제동 배분
    Mz  = latCmd.yawMoment;
    htf = VEH.track_f/2;  htr = VEH.track_r/2;

    % ESC: 요모멘트 → 좌우 차동 (전축 60 : 후축 40)
    dTf = Mz/(2*htf)*0.6;
    dTr = Mz/(2*htr)*0.4;

    % 종방향 제동: Fx_total<0 → 4륜 균등 (force→torque)
    if lonCmd.Fx_total < 0
        brakePW = abs(lonCmd.Fx_total) * VEH.rw / 4;
    else
        brakePW = 0;
    end

    T = zeros(4,1);
    T(1) = brakePW + max(0,  dTf);   % FL
    T(2) = brakePW + max(0, -dTf);   % FR
    T(3) = brakePW + max(0,  dTr);   % RL
    T(4) = brakePW + max(0, -dTr);   % RR

    % 휠별 가산 ABS — 여유 있는(덜 잠긴) 휠에 제동 추가
    if isfield(lonCmd, 'brakeAdd') && numel(lonCmd.brakeAdd) == 4
        T = T + lonCmd.brakeAdd(:);
    end

    %% 3. 마찰원 실현가능성 (간이) — 총가속도 한계 초과 시 스케일다운
    totalAccel = sqrt((lonCmd.Fx_total/VEH.mass)^2 + (Mz/VEH.Iz*vx)^2);
    if totalAccel > LIM.MAX_AY
        T = T * (LIM.MAX_AY/totalAccel);
    end

    T = max(0, min(LIM.MAX_BRAKE_TRQ, T));
    actuatorCmd.brakeTorque = T;

    %% 4. 댐퍼 통과
    actuatorCmd.dampingCoeff = verCmd;
end
