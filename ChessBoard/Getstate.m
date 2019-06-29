function [state, flag] = Getstate(I)
thresholdmask = 0.57;%通过亮度分割前景区域阈值
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

% 6通道图
% figure(1)
% ax(1)=subplot(2,3,1);imshow(I(:,:,1));
% ax(2)=subplot(2,3,2);imshow(I(:,:,2));
% ax(3)=subplot(2,3,3);imshow(I(:,:,3));
% ax(4)=subplot(2,3,4);imshow(hsv(:,:,1));
% ax(5)=subplot(2,3,5);imshow(hsv(:,:,2));
% ax(6)=subplot(2,3,6);imshow(hsv(:,:,3));
% linkaxes(ax);

%得到前景分割区域
se = strel('square',4);
mask = hsv(:,:,3) > thresholdmask;
mask = imopen(mask,se);
mask = imfill(mask,'holes');
[mx my] = find(mask);
mx = [mx;max([min(mx)-60,1])];
% [xx yy]=size(mask);
% mx = [1,xx];
% my = [1,yy];
maskn = mask(min(mx):max(mx),min(my):max(my));%将前景分割区域提出来

% figure(2),imshow(mask);

%得到蓝色、黄色、紫色三个区域
ChessBlue = (hsv(min(mx):max(mx),min(my):max(my),1)>Blue_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Blue_s).*(hsv(min(mx):max(mx),min(my):max(my),3)>Blue_v);
ChessYellow = (hsv(min(mx):max(mx),min(my):max(my),1)<Yellow_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Yellow_s);
Purple = (hsv(min(mx):max(mx),min(my):max(my),1)>Purple_h).*(hsv(min(mx):max(mx),min(my):max(my),2)>Purple_s).*(hsv(min(mx):max(mx),min(my):max(my),3)<Purple_v);

ChessBlue = ChessBlue.*maskn;
ChessYellow = ChessYellow.*maskn;
Purple = Purple.*maskn;

%开运算去除孤立点
se = strel('square',5);
ChessBlue = imopen(ChessBlue,se);
se = strel('square',3);
ChessYellow = imopen(ChessYellow,se);
% se = strel('square',6);
Purple = imopen(Purple,se);

% 扩张
% se = strel('square',3);
% ChessBlue = imdilate(ChessBlue,se);
% ChessYellow = imdilate(ChessYellow,se);

% 各颜色分割图
% figure(3)
% ax(1)=subplot(2,2,1);imshow(I(min(mx):max(mx),min(my):max(my),:));
% ax(2)=subplot(2,2,2);imshow(ChessBlue);
% ax(3)=subplot(2,2,3);imshow(ChessYellow);
% ax(4)=subplot(2,2,4);imshow(Purple);
% linkaxes(ax);

%% 获取四角顶点
[height width] = size(Purple);
midheight = round(height/2);
midwidth = round(width/2);
mask1 = zeros(size(Purple));mask1(1:midheight,1:midwidth) = 1;
mask2 = zeros(size(Purple));mask2(1:midheight,midwidth:width) = 1;
mask3 = zeros(size(Purple));mask3(midheight:height,1:midwidth) = 1;
mask4 = zeros(size(Purple));mask4(midheight:height,midwidth:width) = 1;

if max(max(Purple.*mask1)) == 0 ||  max(max(Purple.*mask2)) == 0  ||  max(max(Purple.*mask3)) == 0  ||  max(max(Purple.*mask4)) == 0
    flag = 1;%四角识别错误
else 
    Purple1 = GetMaxCon(Purple.*mask1);%获取最大连通分量
    Purple2 = GetMaxCon(Purple.*mask2);%获取最大连通分量
    Purple3 = GetMaxCon(Purple.*mask3);%获取最大连通分量
    Purple4 = GetMaxCon(Purple.*mask4);%获取最大连通分量

%     找边界点分割图
%     figure(4)
%     ax(1)=subplot(2,2,1);imshow(Purple1);
%     ax(2)=subplot(2,2,2);imshow(Purple2);
%     ax(3)=subplot(2,2,3);imshow(Purple3);
%     ax(4)=subplot(2,2,4);imshow(Purple4);
%     linkaxes(ax);

    %获取四个顶点坐标
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

    %图像上的点与matlab点xy相反
    dot = zeros(4,2);
    dot(:,1) = purpledot(:,2);
    dot(:,2) = purpledot(:,1);

%     figure(5),imshow(Purple);
%     hold on
%     plot(dot(:,1),dot(:,2),'*')
    %% 判断是否有杂物
    maskinside = JudgeInside(maskn , dot);
    if(sum(sum(maskinside.* (1 - maskn))) > 50)
        flag = 2; % 有杂物
    else
        %% 透视变换
        [ChessBlueYellow,dot2] = Switch(ChessBlue - ChessYellow, dot);

        gapx = abs(dot2(1,2)-dot2(2,2))/14;
        gapy = abs(dot2(1,1)-dot2(2,1))/14;

        %扩张
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

L = bwlabel(p);% 对连通区域进行标记
stats = regionprops(L);
Ar = cat(1, stats.Area);
ind = find(Ar ==max(Ar));%找到最大连通区域的标号
p(find(L~=ind))=0;%将其他区域置为0
p1 = p;
% figure,imshow(Purple1);%显示最大联通区域
end

function [imgn ,dot2] = Switch(img , dot)
[M N] = size(img);
dot2 = zeros(2,2);
w=round(sqrt((dot(1,1)-dot(2,1))^2+(dot(1,2)-dot(2,2))^2));     %从原四边形获得新矩形宽
h=round(sqrt((dot(1,1)-dot(3,1))^2+(dot(1,2)-dot(3,2))^2));     %从原四边形获得新矩形高

y=[dot(1,1) dot(2,1) dot(3,1) dot(4,1)];        %四个原顶点
x=[dot(1,2) dot(2,2) dot(3,2) dot(4,2)];

%这里是新的顶点，我取的矩形,也可以做成其他的形状
%大可以原图像是矩形，新图像是从dot中取得的点组成的任意四边形.:)
Y=[dot(1,1) dot(1,1) dot(1,1)+h dot(1,1)+h];     
X=[dot(1,2) dot(1,2)+w dot(1,2) dot(1,2)+w];

B=[X(1) Y(1) X(2) Y(2) X(3) Y(3) X(4) Y(4)]';   %变换后的四个顶点，方程右边的值
%联立解方程组，方程的系数
A=[x(1) y(1) 1 0 0 0 -X(1)*x(1) -X(1)*y(1);             
0 0 0 x(1) y(1) 1 -Y(1)*x(1) -Y(1)*y(1);
   x(2) y(2) 1 0 0 0 -X(2)*x(2) -X(2)*y(2);
0 0 0 x(2) y(2) 1 -Y(2)*x(2) -Y(2)*y(2);
   x(3) y(3) 1 0 0 0 -X(3)*x(3) -X(3)*y(3);
0 0 0 x(3) y(3) 1 -Y(3)*x(3) -Y(3)*y(3);
   x(4) y(4) 1 0 0 0 -X(4)*x(4) -X(4)*y(4);
0 0 0 x(4) y(4) 1 -Y(4)*x(4) -Y(4)*y(4)];

fa=inv(A)*B;        %用四点求得的方程的解，也是全局变换系数
a=fa(1);b=fa(2);c=fa(3);
d=fa(4);e=fa(5);f=fa(6);
g=fa(7);h=fa(8);

rot=[d e f;
     a b c;
     g h 1];        %公式中第一个数是x,Matlab第一个表示y，所以我矩阵1,2行互换了

pix1=rot*[1 1 1]'/(g*1+h*1+1);  %变换后图像左上点
pix2=rot*[1 N 1]'/(g*1+h*N+1);  %变换后图像右上点
pix3=rot*[M 1 1]'/(g*M+h*1+1);  %变换后图像左下点
pix4=rot*[M N 1]'/(g*M+h*N+1);  %变换后图像右下点

height=round(max([pix1(1) pix2(1) pix3(1) pix4(1)])-min([pix1(1) pix2(1) pix3(1) pix4(1)]));     %变换后图像的高度
width=round(max([pix1(2) pix2(2) pix3(2) pix4(2)])-min([pix1(2) pix2(2) pix3(2) pix4(2)]));      %变换后图像的宽度
delta_y=round(abs(min([pix1(1) pix2(1) pix3(1) pix4(1)])));            %取得y方向的负轴超出的偏移量
delta_x=round(abs(min([pix1(2) pix2(2) pix3(2) pix4(2)])));            %取得x方向的负轴超出的偏移量
inv_rot=inv(rot);
imgn=zeros(height,width);
dot2(1,1) = 0;
dot2(2,2) = width;

for i = height-delta_y:-1:1-delta_y                  %从变换图像中反向寻找原图像的点，以免出现空洞，和旋转放大原理一样
    for j = 1-delta_x:width-delta_x
        if i > dot2(1,1) -delta_y - 3 && j < dot2(2,2) - delta_x + 2
            pix=inv_rot*[i j 1]';       %求原图像中坐标，因为[YW XW W]=fa*[y x 1],所以这里求的是[YW XW W],W=gy+hx+1;
            pix=inv([g*pix(1)-1 h*pix(1);g*pix(2) h*pix(2)-1])*[-pix(1) -pix(2)]'; %相当于解[pix(1)*(gy+hx+1) pix(2)*(gy+hx+1)]=[y x],这样一个方程，求y和x，最后pix=[y x];

            if pix(1)>=0.5 && pix(2)>=0.5 && pix(1)<=M && pix(2)<=N
                imgn(i+delta_y,j+delta_x)=img(round(pix(1)),round(pix(2)));     %最邻近插值,也可以用双线性或双立方插值
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
