rosinit('011606P0006.local','NodeHost','192.168.0.8')

%����
sub = rossubscriber('/cameras/left_hand_camera/image');
% for i = 1:204
% image = readImage(sub.LatestMessage);
% imshow(image);
% %     sub.LatestMessage.show;
% end

chatpub = rospublisher('/posi/human','gobang/ChessMove');
msg = rosmessage(chatpub);
pause(5)
% msg.Row = 7;
% msg.Col = 7;
% msg.State = 78;
% send(chatpub,msg);
% latchpub = rospublisher('/posi/human','sLatching', false);
% msg.Row = 7;
% msg.Col = 7;
% msg.State = 81
% send(chatpub,msg);
% ��ɫΪ-1����ɫΪ1
stateodd = zeros(15);
while 1
	image = readImage(sub.LatestMessage);
%     imshow(image);   
    [statenew, flag] = Getstate(image);
    statetemp = stateodd - statenew;
    if flag == 1
        disp("�ĽǶ�λ���ڵ�");
        pause(1);
    else
        if flag == 2
            disp("���������ϰ���")
            pause(1);
        else
            if (sum(sum(abs(statetemp))) == 1 && sum(sum(statetemp)) == 1)  || (sum(sum(abs(statetemp))) == 2 &&   sum(sum(statetemp)) == 0)
                [x y] = find(statetemp == 1);
                if stateodd(x,y) == 0
                    msg.Row = x - 1;
                    msg.Col = 15 - y;
                    msg.State = 78;
                    send(chatpub,msg);
                    stateodd = statenew;
                end
            end
        end
    end
end
end
clear
rosshutdown
