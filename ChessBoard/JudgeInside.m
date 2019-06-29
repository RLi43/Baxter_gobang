function mask = JudgeInside(img , dot)
[M N] = size(img);
mask = zeros(size(img));
for i = 1:N
    for j = 1:M
        A = [dot(1,1)-i,dot(1,2)-j];
        B = [dot(2,1)-i,dot(2,2)-j];
        C = [dot(3,1)-i,dot(3,2)-j];
        D = [dot(4,1)-i,dot(4,2)-j];
        if(acosx(A,B)+acosx(B,D)+acosx(C,D)+acosx(A,C)> 6.25)
            mask(j,i) = 1;
        end
    end
end
end

function angle = acosx(A ,B)
    angle = acos(dot(A,B)/(norm(A)*norm(B)));
end