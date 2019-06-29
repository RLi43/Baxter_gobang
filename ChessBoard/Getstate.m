function [state, flag] = Getstate(I)
thresholdmask = 0.57;%ͨ�����ȷָ�ǰ��������ֵ
% ddd xd ddx
Blue_h = 0.5;
Blue_s = 0.2; %0.4
Blue_v = 0.65;
Yellow_h = 0.15;
Yellow_s = 0.1; %0.2
Purple_h = 0.63;
Purple_s = 0.2;
Purple_v = 0.65;

% I= imread('chessBoard_di3.png');
hsv = rgb2hsv(I);
% img = hsv(:,:,3);

% 6ͨ��ͼ
% figure(1)
% ax(1)=subplot(2,3,1);imshow(I(:,:,1));
% ax(2)=subplot(2,3,2);imshow(I(:,:,2));
% ax(3)=subplot(2,3,3);imshow(I(:,:,3));
% ax(4)=subplot(2,3,4);imshow(hsv(:,:,1));
% ax(5)=subplot(2,3,5);imshow(hsv(:,:,2));
% ax(6)=subplot(2,3,6);imshow(hsv(:,:,3));
% linkaxes(ax);

%�õ�ǰ���ָ�����
se = strel('square',4);
mask = hsv(:,:,3) > thresholdmask;
mask = imopen(mask,se);
mask = imfill(mask,'holes');
[mx my] = find(mask);
mx = [mx;max([min(mx)-60,1])];
% [xx yy]=size(mask);
% mx = [1,xx];
% my = [1,yy];
maskn = mask(min(mx):max(mx),min(my):max(my));%��ǰ���ָ����������

% figure(2),imshow(mask);

%�õ���ɫ����ɫ����ɫ��������
ChessBlue = (hsv(min(mx):max(mx),min(my):max(my),1)>Blue_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Blue_s).*(hsv(min(mx):max(mx),min(my):max(my),3)>Blue_v);
ChessYellow = (hsv(min(mx):max(mx),min(my):max(my),1)<Yellow_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Yellow_s);
Purple = (hsv(min(mx):max(mx),min(my):max(my),1)>Purple_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Purple_s).*(hsv(min(mx):max(mx),min(my):max(my),3)<Purple_v);

ChessBlue = ChessBlue.*maskn;
ChessYellow = ChessYellow.*maskn;
Purple = Purple.*maskn;

%������ȥ��������
se = strel('square',5);
ChessBlue = imopen(ChessBlue,se);
se = strel('square',3);
ChessYellow = imopen(ChessYellow,se);
% se = strel('square',6);
Purple = imopen(Purple,se);

% ����
% se = strel('square',3);
% ChessBlue = imdilate(ChessBlue,se);
% ChessYellow = imdilate(ChessYellow,se);

% ����ɫ�ָ�ͼ
% figure(3)
% ax(1)=subplot(2,2,1);imshow(I(min(mx):max(mx),min(my):max(my),:));
% ax(2)=subplot(2,2,2);imshow(ChessBlue);
% ax(3)=subplot(2,2,3);imshow(ChessYellow);
% ax(4)=subplot(2,2,4);imshow(Purple);
% linkaxes(ax);

%% ��ȡ�ĽǶ���
[height width] = size(Purple);
midheight = round(height/2);
midwidth = round(width/2);
mask1 = zeros(size(Purple));mask1(1:midheight,1:midwidth) = 1;
mask2 = zeros(size(Purple));mask2(1:midheight,midwidth:width) = 1;
mask3 = zeros(size(Purple));mask3(midheight:height,1:midwidth) = 1;
mask4 = zeros(size(Purple));mask4(midheight:height,midwidth:width) = 1;

if max(max(Purple.*mask1)) == 0 ||  max(max(Purple.*mask2)) == 0  ||  max(max(Purple.*mask3)) == 0  ||  max(max(Purple.*mask4)) == 0
    flag = 1;%�Ľ�ʶ�����
else 
    Purple1 = GetMaxCon(Purple.*mask1);%��ȡ�����ͨ����
    Purple2 = GetMaxCon(Purple.*mask2);%��ȡ�����ͨ����
    Purple3 = GetMaxCon(Purple.*mask3);%��ȡ�����ͨ����
    Purple4 = GetMaxCon(Purple.*mask4);%��ȡ�����ͨ����

%     �ұ߽��ָ�ͼ
%     figure(4)
%     ax(1)=subplot(2,2,1);imshow(Purple1);
%     ax(2)=subplot(2,2,2);imshow(Purple2);
%     ax(3)=subplot(2,2,3);imshow(Purple3);
%     ax(4)=subplot(2,2,4);imshow(Purple4);
%     linkaxes(ax);

    %��ȡ�ĸ���������
    purpledot = zeros(4,2);

    [x y] = find(Purple1);
    z = y + x;
    id = find(z == max(z));
    purpledot(1,1) = x(id(1));
    purpledot(1,2) = y(id(1));

    [x y] = find(Purple2);
    z = x - y;
    id = find(z == max(z));
    purpledot(2,1) = x(id(1));
    purpledot(2,2) = y(id(1));

    [x y] = find(Purple3);
    z = - x + y;
    id = find(z == max(z));
    purpledot(3,1) = x(id(1));
    purpledot(3,2) = y(id(1));

    [x y] = find(Purple4);
    z = - x - y;
    id = find(z == max(z));
    purpledot(4,1) = x(id(1));
    purpledot(4,2) = y(id(1));

    %ͼ���ϵĵ���matlab��xy�෴
    dot = zeros(4,2);
    dot(:,1) = purpledot(:,2);
    dot(:,2) = purpledot(:,1);

%     figure(5),imshow(Purple);
%     hold on
%     plot(dot(:,1),dot(:,2),'*')
    %% �ж��Ƿ�������
    maskinside = JudgeInside(maskn , dot);
    if(sum(sum(maskinside.* (1 - maskn))) > 50)
        flag = 2; % ������
    else
        %% ͸�ӱ任
        [ChessBlueYellow,dot2] = Switch(ChessBlue - ChessYellow, dot);

        gapx = abs(dot2(1,2)-dot2(2,2))/14;
        gapy = abs(dot2(1,1)-dot2(2,1))/14;

        %����
        se = strel('square',5);
        ChessBlueNew = ChessBlueYellow > 0;
        ChessYellowNew = ChessBlueYellow < 0; 
        ChessBlueNew = imdilate(ChessBlueNew,se);
        ChessYellowNew = imdilate(ChessYellowNew,se);
        ChessBlueYellowNew = ChessBlueNew - ChessYellowNew;

        figure(10);
        imshow(ChessBlueYellowNew,[]);
        hold on
        state = zeros(15);
        for i = 0:14
            for j = 0:14
                if(ChessBlueYellowNew(round(dot2(1,1)+j*gapy),round(dot2(1,2)+i*gapx)) > 0)
                    state(i+1,j+1) = 1;%Blue 
                else if(ChessBlueYellowNew(round(dot2(1,1)+j*gapy),round(dot2(1,2)+i*gapx)) < 0)
                      state(i+1,j+1) = -1;%Yellow
                    end
                end
                plot(dot2(1,2)+i*gapx,dot2(1,1)+j*gapy,'yx')
            end
        end
        flag = 0;
    end
end
end
%%
function [p1] = GetMaxCon(p)

L = bwlabel(p);% ����ͨ������б��
stats = regionprops(L);
Ar = cat(1, stats.Area);
ind = find(Ar ==max(Ar));%�ҵ������ͨ����ı��
p(find(L~=ind))=0;%������������Ϊ0
p1 = p;
% figure,imshow(Purple1);%��ʾ�����ͨ����
end

function [imgn ,dot2] = Switch(img , dot)
[M N] = size(img);
dot2 = zeros(2,2);
w=round(sqrt((dot(1,1)-dot(2,1))^2+(dot(1,2)-dot(2,2))^2));     %��ԭ�ı��λ���¾��ο�
h=round(sqrt((dot(1,1)-dot(3,1))^2+(dot(1,2)-dot(3,2))^2));     %��ԭ�ı��λ���¾��θ�

y=[dot(1,1) dot(2,1) dot(3,1) dot(4,1)];        %�ĸ�ԭ����
x=[dot(1,2) dot(2,2) dot(3,2) dot(4,2)];

%�������µĶ��㣬��ȡ�ľ���,Ҳ����������������״
%�����ԭͼ���Ǿ��Σ���ͼ���Ǵ�dot��ȡ�õĵ���ɵ������ı���.:)
Y=[dot(1,1) dot(1,1) dot(1,1)+h dot(1,1)+h];     
X=[dot(1,2) dot(1,2)+w dot(1,2) dot(1,2)+w];

B=[X(1) Y(1) X(2) Y(2) X(3) Y(3) X(4) Y(4)]';   %�任����ĸ����㣬�����ұߵ�ֵ
%�����ⷽ���飬���̵�ϵ��
A=[x(1) y(1) 1 0 0 0 -X(1)*x(1) -X(1)*y(1);             
0 0 0 x(1) y(1) 1 -Y(1)*x(1) -Y(1)*y(1);
   x(2) y(2) 1 0 0 0 -X(2)*x(2) -X(2)*y(2);
0 0 0 x(2) y(2) 1 -Y(2)*x(2) -Y(2)*y(2);
   x(3) y(3) 1 0 0 0 -X(3)*x(3) -X(3)*y(3);
0 0 0 x(3) y(3) 1 -Y(3)*x(3) -Y(3)*y(3);
   x(4) y(4) 1 0 0 0 -X(4)*x(4) -X(4)*y(4);
0 0 0 x(4) y(4) 1 -Y(4)*x(4) -Y(4)*y(4)];

fa=inv(A)*B;        %���ĵ���õķ��̵Ľ⣬Ҳ��ȫ�ֱ任ϵ��
a=fa(1);b=fa(2);c=fa(3);
d=fa(4);e=fa(5);f=fa(6);
g=fa(7);h=fa(8);

rot=[d e f;
     a b c;
     g h 1];        %��ʽ�е�һ������x,Matlab��һ����ʾy�������Ҿ���1,2�л�����

pix1=rot*[1 1 1]'/(g*1+h*1+1);  %�任��ͼ�����ϵ�
pix2=rot*[1 N 1]'/(g*1+h*N+1);  %�任��ͼ�����ϵ�
pix3=rot*[M 1 1]'/(g*M+h*1+1);  %�任��ͼ�����µ�
pix4=rot*[M N 1]'/(g*M+h*N+1);  %�任��ͼ�����µ�

height=round(max([pix1(1) pix2(1) pix3(1) pix4(1)])-min([pix1(1) pix2(1) pix3(1) pix4(1)]));     %�任��ͼ��ĸ߶�
width=round(max([pix1(2) pix2(2) pix3(2) pix4(2)])-min([pix1(2) pix2(2) pix3(2) pix4(2)]));      %�任��ͼ��Ŀ��
delta_y=round(abs(min([pix1(1) pix2(1) pix3(1) pix4(1)])));            %ȡ��y����ĸ��ᳬ����ƫ����
delta_x=round(abs(min([pix1(2) pix2(2) pix3(2) pix4(2)])));            %ȡ��x����ĸ��ᳬ����ƫ����
inv_rot=inv(rot);
imgn=zeros(height,width);
dot2(1,1) = 0;
dot2(2,2) = width;

for i = height-delta_y:-1:1-delta_y                  %�ӱ任ͼ���з���Ѱ��ԭͼ��ĵ㣬������ֿն�������ת�Ŵ�ԭ��һ��
    for j = 1-delta_x:width-delta_x
        if i > dot2(1,1) -delta_y - 3 && j < dot2(2,2) - delta_x + 2
            pix=inv_rot*[i j 1]';       %��ԭͼ�������꣬��Ϊ[YW XW W]=fa*[y x 1],�������������[YW XW W],W=gy+hx+1;
            pix=inv([g*pix(1)-1 h*pix(1);g*pix(2) h*pix(2)-1])*[-pix(1) -pix(2)]'; %�൱�ڽ�[pix(1)*(gy+hx+1) pix(2)*(gy+hx+1)]=[y x],����һ�����̣���y��x�����pix=[y x];

            if pix(1)>=0.5 && pix(2)>=0.5 && pix(1)<=M && pix(2)<=N
                imgn(i+delta_y,j+delta_x)=img(round(pix(1)),round(pix(2)));     %���ڽ���ֵ,Ҳ������˫���Ի�˫������ֵ
            end  
            if abs(round(pix(1)) - dot(1, 2)) <= 1 && abs(round(pix(2)) - dot(1, 1)) <= 1
                dot2(1,1) = i+delta_y;
                dot2(1,2) = j+delta_x;
            else
                if abs(round(pix(1)) - dot(4, 2)) <= 1 && abs(round(pix(2)) - dot(4, 1)) <= 1
                dot2(2,1) = i+delta_y;
                dot2(2,2) = j+delta_x;
                end
            end
        end
    end
end
end
