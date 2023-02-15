clc;
clear;
hold on;
I = csvread("data.csv");

plot3(0,0,0,'k*');

L=[-15.59049,  -29.633337,  28.85207 ;
 13.090608, -31.966821,  25.842173;
-30.262151, -22.784739,  21.988585;
 24.123169, -21.318642,  16.792984;
-30.200880625182418, -13.593387838082322 , 8.134705225021412;
20.86860584395294 , -8.095281115842548 , 10.22757801621854;
-16.913920709351583 , -3.5426354308919064 , 1.6359649600150157;
8.364206671324332 , -4.476015699053317 , -1.0948872031393384];
for i=1:8
    plot3(L(:,1),L(:,2),L(:,3),'o');
end

plane = cross(L(1,:)-L(2,:),L(1,:)-L(7,:));
% plane = (normalize(cross(L(7,:)-L(1,:),L(8,:)-L(7,:)))+normalize(cross(L(8,:)-L(7,:),L(2,:)-L(8,:)))+normalize(cross(L(2,:)-L(8,:),L(1,:)-L(2,:)))+normalize(cross(L(1,:)-L(2,:),L(7,:)-L(1,:))))/4;
[n,m] = size(I);
len = 50;
% for i=1:n
% %     I(i,2:4)=I(2,2:4);
% %     if I(i,1)==1
%         plot3(I(i,2),I(i,3),I(i,4),'r*');
%         if I(i,7)>0
%             plot3([I(i,2),I(i,2)-len*I(i,6)],[I(i,3),I(i,3)-len*I(i,7)],[I(i,4),I(i,4)-len*I(i,8)],'b-');
%         else
%             plot3([I(i,2),I(i,2)+len*I(i,6)],[I(i,3),I(i,3)+len*I(i,7)],[I(i,4),I(i,4)+len*I(i,8)],'b-');
%         end  
% %     end
% end

% for i=1:9
%     plot3(I(i,10),I(i,11),I(i,12),'b*');
%     plot3([I(i,10),I(i,10)+len*I(i,14)],[I(i,11),I(i,11)+len*I(i,15)],[I(i,12),I(i,12)+len*I(i,16)],'b-');
% end

res = zeros(9,3);
index = 1;
temp = [0,0,0];
count = 0;
for i=1:n
    p1 = I(2,2:4);
    v1 = I(i,6:8);
    v2 = plane;
    p2 = L(4,:);
    t = (dot(p2,v2)-dot(p1,v2))/dot(v1,v2);
    pos= p1+t*v1;
    if index == I(i,1)
        temp=temp+pos;
        count=count+1;
    else
        res(index,:)=temp./count;
        temp = [0,0,0];
        count = 0;
        index = index + 1;
        if index == I(i,1)
            temp=temp+pos;
            count=count+1;
        end
    end

    
%     if I(i,1)==1 || I(i,1)==2 || I(i,1)==3
%         plot3(pos(1),pos(2),pos(3),'bo');
%     elseif I(i,1)==4 || I(i,1)==5 || I(i,1)==6
%         plot3(pos(1),pos(2),pos(3),'ro');
%     else
%         plot3(pos(1),pos(2),pos(3),'go');
%     end
       
end
res(index,:)=temp./count;
for i = 1:9
    if res(i,1)~=0
        if i==1 || i==2 || i==3
            plot3(res(i,1),res(i,2),res(i,3),'ro');
        elseif i==4 || i==5 || i==6
            plot3(res(i,1),res(i,2),res(i,3),'go');
        else
            plot3(res(i,1),res(i,2),res(i,3),'bo');
        end
        
    end
end





