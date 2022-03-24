% N: number of samples
% dutyCycle: real number betweeb 0 and 1
function y = myprbs( N, dutyCycle )
if(numel(N) == 1)
    N = [N 1];
end
y = rand(N);
y = double(y > 1 - dutyCycle);
end

