tic
%初始化车的参数
Xo=[0 0];%起点位置
X1=[0 1];
k=15;%计算引力需要的增益系数
m=3;%计算斥力的增益系数，都是自己设定的。
Po=1.5;%障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响。也是自己设定。
n=5;%障碍个数
l=0.2;%步长
J=1000;%循环迭代次数
%如果不能实现预期目标，可能也与初始的增益系数，Po设置的不合适有关。
%end 
%给出障碍和目标信息
Xsum=[4 4;
    4 5;
    1 0.6;
    2 3.5;
    1.4 2.1;
    3.5 3;
    3 2];
%这个向量是(n+1)*2维，其中第一个点[4 4]是目标位置，剩下的都是障碍的位置。
XX0=Xo;%j=1循环初始，将车的起始坐标赋给XXX
XX1=X1;
%***************初始化结束，开始主体循环******************
for j=1:J %循环开始
    goal0(j,1)=XX0(1); %Goal是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
    goal0(j,2)=XX0(2);
    goal1(j+1,1)=XX1(1);
    goal1(j+1,2)=XX1(2);
    for i=1:n+1   %计算物体和障碍物、目标点的向量
         deltaX0(i)=Xsum(i,1)-XX0(1);
         deltaY0(i)=Xsum(i,2)-XX0(2);
         deltaX1(i)=Xsum(i+1,1)-XX1(1);
         deltaY1(i)=Xsum(i+1,2)-XX1(2);
         r0(i)=sqrt(deltaX0(i)^2+deltaY0(i)^2);
         r1(i)=sqrt(deltaX1(i)^2+deltaY1(i)^2);
    end
    Rgoal0=sqrt((XX0(1)-Xsum(1,1))^2+(XX0(2)-Xsum(1,2))^2);   %路径点和目标的距离
    Rgoal1=sqrt((XX1(1)-Xsum(1,1))^2+(XX1(2)-Xsum(1,2))^2);
    %目标点对路径点的引力
    Fatx0=k*Rgoal0*(deltaX0(1)/Rgoal0);
    Faty0=k*Rgoal0*(deltaY0(1)/Rgoal0);
    Fatx1=k*Rgoal1*(deltaX1(1)/Rgoal1);
    Faty1=k*Rgoal1*(deltaY1(1)/Rgoal1);
    %各个障碍物对路径点的斥力
    for i=1:n
        if r0(i+1)>Po
            Frex0(i)=0;
            Frey0(i)=0;
        else
            Frex0(i)=-m*(1/r0(i+1)-1/Po)/r0(i+1)/r0(i+1)*(deltaX0(i+1)/r0(i+1));
            Frey0(i)=-m*(1/r0(i+1)-1/Po)/r0(i+1)/r0(i+1)*(deltaY0(i+1)/r0(i+1));
        end

        if r1(i+1)>Po
            Frex1(i)=0;
            Frey1(i)=0;
        else
            Frex1(i)=-m*(1/r1(i+1)-1/Po)/r1(i+1)/r1(i+1)*(deltaX1(i+1)/r1(i+1));
            Frey1(i)=-m*(1/r1(i+1)-1/Po)/r1(i+1)/r1(i+1)*(deltaY1(i+1)/r1(i+1));
        end
    end
    %计算合力
    F0sumx=Fatx0+sum(Frex0);
    F0sumy=Faty0+sum(Frey0);
    F0=sqrt(F0sumx^2+F0sumy^2);
    F1sumx=Fatx1+sum(Frex1);
    F1sumy=Faty1+sum(Frey1);
    F1=sqrt(F1sumx^2+F1sumy^2);
    %求解下一个路径点
    Xnext0(1)=(XX0(1)+l*F0sumx/F0);   %式子中的l是步长
    Xnext0(2)=(XX0(2)+l*F0sumy/F0);
    XX0=Xnext0;
    Xnext1(1)=(XX1(1)+l*F1sumx/F1);   %式子中的l是步长
    Xnext1(2)=(XX1(2)+l*F1sumy/F1);
    XX1=Xnext1;
    if (sqrt((XX0(1)-Xsum(1,1))^2+(XX0(2)-Xsum(1,2))^2)<0.1) && (sqrt((XX1(1)-Xsum(1,1))^2+(XX1(2)-Xsum(1,2))^2)<0.1)%当物体接近目标点时
        k=j;   %迭代次数
        break;
    end
end

K=j;
goal0(K,1)=Xsum(1,1);%把路径向量的最后一个点赋值为目标
goal0(K,2)=Xsum(1,2);
goal1(K,1)=Xsum(1,1);%把路径向量的最后一个点赋值为目标
goal1(K,2)=Xsum(1,2);
%***********************************画出障碍，起点，目标，路径点*************************
%画出路径
X0=goal0(:,1);
Y0=goal0(:,2);
X1=goal1(:,1);
Y1=goal1(:,2);
%路径向量Goal是二维数组,X,Y分别是数组的x,y元素的集合，是两个一维数组。
%x=[1 3 4 3 6 5.5 8];%障碍的x坐标
%y=[1.2 2.5 4.5 6 2 5.5 8.5];

x=Xsum(3:n+1,1);
y=Xsum(3:n+1,2);
plot(x,y,'o',4,4,'v',4,5,'v',0,0,'ms',0,1,'ms',X0,Y0,'.r',X1,Y1,'.b');
toc