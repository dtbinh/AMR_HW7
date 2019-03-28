function d = dist_func(p1, p2)
% helper function to calc distance between two 2-d points
d = norm(p1-p2);
if(d==0), d=1e-6; end % to avoid divisions by zero
end

