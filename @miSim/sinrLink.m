function [sinr, grad] = sinrLink(obj, positions, rxIdx, txIdx)
% SINRLINK  Linear SINR of the link txIdx -> rxIdx and its position gradient.
%
% Computes the signal-to-interference-plus-noise ratio seen at receiver rxIdx
% for the signal transmitted by txIdx, treating every other agent as an
% interferer at that receiver. Isotropic transmit/receive (no antenna gains).
%
% Returns the linear SINR and (optionally) its gradient with respect to every
% agent position, grad(a, :) = d(SINR)/d(p_a). The value is used by
% updateAdjacency (connectivity) and both the value and gradient by
% constrainMotion (the SINR CBF barrier).
%
% Model (linear power domain), receiver r = rxIdx, transmitter t = txIdx:
%   S    = P_t / (K * d_tr^n)                 % received signal at r from t (W)
%   I    = sum_{k != t,r} P_k / (K * d_kr^n)  % interference at r from others (W)
%   N    = k_B * T * B                        % thermal noise (W)
%   SINR = S / (N + I)
% with K = (4*pi*f_c/c)^2 and physical constants baked in below. The only
% tunables are per-agent txPower (P), pathLossExponent (n), ambientTemp (T),
% all read from obj; the SINR threshold is applied by the callers.

    arguments (Input)
        obj (1, 1) {mustBeA(obj, "miSim")};
        positions (:, 3) double;
        rxIdx (1, 1) double;
        txIdx (1, 1) double;
    end
    arguments (Output)
        sinr (1, 1) double;
        grad (:, 3) double;
    end

    % Universal physical constants
    k_B = 1.380649e-23;          % Boltzmann constant (J/K)
    c   = 3e8;                   % speed of light (m/s)

    % System parameters
    f_c = obj.centerFreq;        % carrier frequency (Hz)
    B   = obj.bandwidth;         % channel bandwidth (Hz)
    K   = (4 * pi * f_c / c)^2;  % free-space path-loss reference constant

    n  = obj.pathLossExponent;
    T  = obj.ambientTemp;
    d0 = 1.0;                    % near-field distance floor (m)

    nAgents = size(positions, 1);
    grad = zeros(nAgents, 3);

    r  = rxIdx;
    t  = txIdx;
    pr = positions(r, :);

    % Thermal noise (always > 0, so the denominator never vanishes)
    N = k_B * T * B;

    % Signal term S (source = transmitter t)
    P_t    = obj.agents{t}.txPower;
    dvec_t = positions(t, :) - pr;
    d_t    = max(norm(dvec_t), d0);
    S      = P_t / (K * d_t^n);

    % Interference I = sum of received powers from every other agent at r
    I = 0.0;
    for k = 1:nAgents
        if k == r || k == t
            continue;
        end
        d_k = max(norm(positions(k, :) - pr), d0);
        I   = I + obj.agents{k}.txPower / (K * d_k^n);
    end

    D    = N + I;       % denominator: noise + interference
    sinr = S / D;

    % ---- Gradient (skip unless requested) -------------------------------
    % For a received term P_x/(K*d_xr^n):
    %   d(term)/dp_x = -n*term/d_xr^2 * (p_x - p_r),  d(term)/dp_r = -d(term)/dp_x
    if nargout > 1
        % Transmitter affects S only: dSINR/dp_t = (dS/dp_t)/D
        dS_dpt = (-n * S / (d_t^2)) * dvec_t;
        grad(t, :) = dS_dpt / D;

        % Interferers affect I only; accumulate the receiver's I-contribution
        dI_dpr = zeros(1, 3);
        for k = 1:nAgents
            if k == r || k == t
                continue;
            end
            dvec_k    = positions(k, :) - pr;
            d_k       = max(norm(dvec_k), d0);
            term_k    = obj.agents{k}.txPower / (K * d_k^n);
            dterm_dpk = (-n * term_k / (d_k^2)) * dvec_k;
            grad(k, :) = (-S / (D^2)) * dterm_dpk;      % dSINR/dp_k
            dI_dpr     = dI_dpr - dterm_dpk;            % d(term_k)/dp_r = -d(term_k)/dp_k
        end

        % Receiver affects both S and I
        dS_dpr = -dS_dpt;
        grad(r, :) = dS_dpr / D - (S / (D^2)) * dI_dpr;
    end
end
