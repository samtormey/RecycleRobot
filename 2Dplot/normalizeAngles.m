function angles = normalizeAngle(in)

for i = 1:numel(in)
    if in(i) > pi
        in(i) = in(i) - 2*pi;
    end
    if in(i) < -pi
        in(i) = in(i) + 2*pi;
    end
end

angles = in;

end