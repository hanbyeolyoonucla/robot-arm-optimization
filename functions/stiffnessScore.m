function stiffnessNormalized = stiffnessScore(L1, L2, L3, L4, L5)

w = 1:5; % weight on each link
linkLb = [0.064 0.082 0.100 0.040 0.090];
linkUb = [0.3 0.3 0.3 0.3 0.3];
link = [L1 L2 L3 L4 L5];
stiffnessMax = 0;
stiffnessMin = 0;
stiffness = 0;
for ii = 1:5
    stiffnessMax = stiffnessMax + w(ii)*linkUb(ii)^3;
    stiffnessMin = stiffnessMin + w(ii)*linkLb(ii)^3;
    stiffness = stiffness + w(ii)*link(ii)^3;
end

stiffnessNormalized = (stiffnessMax - stiffness) / (stiffnessMax - stiffnessMin);
end