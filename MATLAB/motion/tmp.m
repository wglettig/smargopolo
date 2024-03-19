clear all;
syms xT x0 v0 vT t0 t1 t2 t3 tT vmax amax dmax attn s_acc s_decc;

% area under the v-t curve must be xT-x0
% solve( xT-x0==attn*amax*(t1-t0)/2 + attn*dmax*(t3-t1)/2, attn)

%s_acc  == v0*(vmax-v0)/attn*amax + (vmax-v0)^2/2/attn*amax;
%s_decc == vmax*(vT-vmax)/attn*dmax + (vT-vmax)^2/2/attn*dmax;
%xT-x0 == s_acc + s_decc;
%solve(xT-x0 == v0*(vmax-v0)/attn*amax + (vmax-v0)^2/2/attn*amax + vmax*(vT-vmax)/attn*dmax + (vT-vmax)^2/2/attn*dmax, attn)
solve(xT-x0 == v0*((tT-t0)-(vT-vmax)/(attn*dmax)) + (vmax-v0)^2/2/(attn*amax) + vmax*(vT-vmax)/(attn*dmax) + (vT-vmax)^2/2/(attn*dmax),attn)


%xT-x0 == v0*((tT-t0)-(vT-vmax)/(attn*dmax)) + (vmax-v0)^2/2/(attn*amax) + vmax*(vT-vmax)/(attn*dmax) + (vT-vmax)^2/2/(attn*dmax);
%(tT-t0)-(vT-vmax)/(attn*dmax) = (vmax-v0)/(attn*amax);