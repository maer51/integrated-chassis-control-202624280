function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (반능동 서스펜션) — 연속 Skyhook per-wheel 가변 감쇠
%
%   이상적 Skyhook 힘 F=-c_sky·zs_dot 를 소산만 가능한 반능동 댐퍼로 근사한다.
%   댐퍼 힘 -c·(zs_dot-zu_dot) 이 Skyhook 힘과 같은 방향일 때만 c 를 키운다.
%
%       zs_dot·(zs_dot-zu_dot) > 0 :  c = clamp(c_sky·|zs_dot/(zs_dot-zu_dot)|, cMin, cMax)
%       else                       :  c = cMin
%
%   Inputs:
%       suspState - .zs_dot(4) sprung vel, .zu_dot(4) unsprung vel, .zs(4), .zu(4)
%       CTRL.VER  - .cMin, .cMax, .skyGain
%   Output: dampingCmd 4×1 [Ns/m]

    cMin = CTRL.VER.cMin;  cMax = CTRL.VER.cMax;  cSky = CTRL.VER.skyGain;

    dampingCmd = cMin * ones(4,1);

    if isfield(suspState,'zs_dot') && ~isempty(suspState.zs_dot)
        zsd = suspState.zs_dot(:);
        if isfield(suspState,'zu_dot') && ~isempty(suspState.zu_dot)
            zud = suspState.zu_dot(:);
        else
            zud = zeros(4,1);
        end
        vrel = zsd - zud;                      % 서스펜션 상대속도
        for i = 1:4
            if zsd(i)*vrel(i) > 0              % Skyhook 실현 가능 영역
                c = cSky * abs(zsd(i)) / max(abs(vrel(i)), 1e-3);
                c = min(cMax, c);
            else
                c = cMin;
            end
            dampingCmd(i) = max(cMin, min(cMax, c));
        end
    end
end
